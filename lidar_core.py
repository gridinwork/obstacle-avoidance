"""LiDAR acquisition and preprocessing."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Iterable, List

import serial.tools.list_ports

import config
from settings import Settings
from shared_data import SharedData, Point

try:
    from rplidar import RPLidar
except Exception:  # pragma: no cover - dependency may be missing on dev host
    RPLidar = None  # type: ignore


MIN_DIST_CM = 15
MAX_DIST_CM = 400
SCAN_ARC_DEG = 160
SCAN_BUFFER = 500  # v12-stable buffer size
RECONNECT_DELAY = 1.0
LOOP_PAUSE = 1.0
VISUALIZATION_DELAY_SEC = 1.0
CLUSTER_DISTANCE_TOLERANCE_CM = 5.0
CLUSTER_ANGLE_TOLERANCE_DEG = 3.0
CLUSTER_MIN_POINTS = 3
CLUSTER_PERSISTENCE_SEC = 1.0


def _find_lidar_port() -> str | None:
    """Return the first USB/ACM port that looks like the RPLidar."""
    for port in serial.tools.list_ports.comports():
        dev = getattr(port, "device", "") or getattr(port, "name", "")
        if "USB" in dev or "ACM" in dev:
            return dev
    return None


def _connect_lidar(port: str) -> RPLidar:
    """Connect using the V4 pattern: create, flush buffer, start motor."""
    lidar = RPLidar(port, baudrate=config.LIDAR_BAUDRATE)
    lidar._serial.reset_input_buffer()
    lidar.start_motor()
    return lidar


def _filter_scan(scan: Iterable[tuple[int, float, float]], boundaries: tuple[int, int, int]) -> List[Point]:
    """Filter raw scan data into the front-facing arc and boundary box."""
    _, green_boundary_cm, green_limit_cm = boundaries
    points: List[Point] = []
    for sample in scan:
        try:
            if hasattr(sample, "__len__") and len(sample) >= 3:
                _, angle, dist_mm = sample[:3]
            elif hasattr(sample, "__len__") and len(sample) == 2:
                angle, dist_mm = sample  # type: ignore[misc]
            else:
                # Accept tuples/iterables with at least angle and distance.
                angle, dist_mm = sample[1], sample[2]  # type: ignore[index]
        except Exception:
            # Skip malformed samples without aborting the scan loop.
            continue
        dist_cm = dist_mm / 10.0
        if not (MIN_DIST_CM < dist_cm < MAX_DIST_CM):
            continue
        normalized = (angle + 180) % 360 - 180
        if not (-SCAN_ARC_DEG / 2 <= normalized <= SCAN_ARC_DEG / 2):
            continue

        # Convert to cartesian (cm) to clip against the green boundaries.
        x_cm = dist_cm * math.sin(math.radians(normalized))
        y_cm = dist_cm * math.cos(math.radians(normalized))
        if abs(x_cm) > green_boundary_cm:
            continue
        if y_cm > green_limit_cm:
            continue
        points.append((normalized, dist_cm))
    return points


@dataclass
class _Cluster:
    points: List[Point]
    angle_center: float
    min_distance: float

    @property
    def size(self) -> int:
        return len(self.points)


class LidarCore:
    """LiDAR manager rebuilt from the known-good V4 implementation."""

    def __init__(self, shared: SharedData, settings: Settings, log_fn=print) -> None:
        self.shared = shared
        self.settings = settings
        self.log = log_fn
        self._stream_stop = threading.Event()
        self._stream_thread: threading.Thread | None = None
        self._lidar: RPLidar | None = None  # type: ignore[assignment]
        self._connect_lock = threading.Lock()
        self._connected = False
        self._last_port: str | None = None
        self._shutdown_on_stop = False
        self._cluster_history: deque[tuple[float, List[_Cluster]]] = deque()
        self._estimated_hz = float(self.shared.lidar_read_hz or config.DEFAULT_LIDAR_READ_HZ)

    def connect(self) -> bool:
        """Begin connecting to LiDAR (spins up the worker thread)."""
        if RPLidar is None:
            self.shared.update_status(
                lidar_connected=False,
                lidar_active=False,
                lidar_status="error",
                lidar_error="RPLidar package missing",
                status_message="RPLidar package missing",
            )
            return False

        port = self._resolve_port()
        if not port:
            self.shared.update_status(
                lidar_connected=False,
                lidar_status="error",
                lidar_error="No LiDAR port found",
                status_message="LiDAR not found",
            )
            return False

        self._last_port = port
        self.shared.update_status(
            lidar_status="connecting",
            lidar_error="",
            status_message=f"Connecting LiDAR on {port}...",
        )
        return self.start_stream()

    def start(self) -> None:
        """Compatibility wrapper: start streaming."""
        self.start_stream()

    def start_stream(self) -> bool:
        if self._stream_thread and self._stream_thread.is_alive():
            return True
        if RPLidar is None:
            self.shared.update_status(
                lidar_connected=False,
                lidar_active=False,
                lidar_status="error",
                lidar_error="RPLidar package missing",
                status_message="RPLidar package missing",
            )
            return False
        self._stream_stop.clear()
        self._shutdown_on_stop = False
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()
        self.log("LiDAR streaming thread started")
        return True

    def stop(self) -> None:
        self.stop_stream(keep_connection=False)

    def stop_stream(self, keep_connection: bool = True) -> None:
        self._shutdown_on_stop = not keep_connection
        self._stream_stop.set()
        if self._stream_thread and self._stream_thread.is_alive():
            self._stream_thread.join(timeout=2)
        self._stream_thread = None
        if not keep_connection:
            self._cleanup_lidar(full_shutdown=True)
            self.shared.update_status(
                lidar_connected=False,
                lidar_active=False,
                lidar_status="disconnected",
                lidar_error="",
                status_message="LiDAR disconnected",
            )
            self.shared.set_validated_lidar_points([])
            self.shared.clear_lidar_ready()
        else:
            self.shared.update_status(lidar_active=False)

    def disconnect(self) -> None:
        """Fully release the LiDAR handle and reset status."""
        self.stop_stream(keep_connection=False)
        self.shared.set_lidar_points([])
        self.shared.set_validated_lidar_points([])

    # Runtime tuning helpers ----------------------------------------------------
    def _scan_buffer_size(self) -> int:
        """Clamp requested buffer size to the stable v12 range."""
        try:
            buf, _ = self.shared.get_lidar_config()
            buf_int = int(buf)
        except Exception:
            buf_int = config.DEFAULT_LIDAR_BUFFER_SIZE
        return max(80, min(buf_int, SCAN_BUFFER))

    def _current_boundaries(self) -> tuple[int, int, int]:
        return self.shared.get_boundaries()

    # Internal ------------------------------------------------------------------
    def _stream_loop(self) -> None:
        while not self._stream_stop.is_set() and self.shared.running:
            port = self._resolve_port()
            if not port:
                self.shared.update_status(
                    lidar_connected=False,
                    lidar_active=False,
                    lidar_status="error",
                    lidar_error="No LiDAR port found",
                    status_message="LiDAR not found",
                )
                time.sleep(RECONNECT_DELAY)
                continue

            force_shutdown = False
            try:
                with self._connect_lock:
                    if self._lidar is None or not self._connected or self._last_port != port:
                        self._lidar = _connect_lidar(port)
                        self._connected = True
                        self._last_port = port

                self.shared.update_status(
                    lidar_connected=True,
                    lidar_status="connected",
                    lidar_error="",
                    status_message=f"LiDAR connected on {port}",
                    lidar_buffer_size=self._scan_buffer_size(),
                )
                self.shared.mark_lidar_ready_after(VISUALIZATION_DELAY_SEC)
                self.log(f"[LIDAR] Connected on {port} at {config.LIDAR_BAUDRATE} baud")

                assert self._lidar is not None  # for type checkers
                scan_buffer = self._scan_buffer_size()
                restart_stream = False
                first_scan = True
                for scan in self._lidar.iter_scans(max_buf_meas=scan_buffer):
                    if self._stream_stop.is_set() or not self.shared.running:
                        break
                    loop_start = time.time()

                    boundaries = self._current_boundaries()
                    try:
                        filtered = _filter_scan(scan, boundaries)
                    except Exception as exc:
                        self.log(f"[LIDAR] scan parse error: {exc}")
                        self.shared.update_status(lidar_error=str(exc), lidar_status="error")
                        continue
                    # Simplify: no clustering to avoid stalls; publish immediately.
                    self.shared.set_lidar_points(filtered)
                    self.shared.set_validated_lidar_points(filtered)

                    if first_scan:
                        self.shared.mark_lidar_ready_after(0.0)
                        first_scan = False

                    elapsed = time.time() - loop_start
                    hz = 0.0
                    if elapsed > 0:
                        hz = min(20.0, 1.0 / elapsed)
                        self._estimated_hz = 0.6 * self._estimated_hz + 0.4 * hz
                    self.shared.update_status(
                        lidar_active=True,
                        lidar_connected=True,
                        lidar_status="connected",
                        lidar_buffer_size=scan_buffer,
                        lidar_read_hz=hz,
                        lidar_error="",
                    )

                    desired_buffer = self._scan_buffer_size()
                    if desired_buffer != scan_buffer:
                        scan_buffer = desired_buffer
                        restart_stream = True
                        break

                if self._stream_stop.is_set():
                    break

                if restart_stream:
                    self.log("[LIDAR] Applying updated LiDAR buffer size")
                    continue

                # If we reach here without a stop request, treat as a stream failure.
                force_shutdown = True
                self._connected = False

            except Exception as exc:
                force_shutdown = True
                self._connected = False
                msg = f"LIDAR ERROR: {exc}"
                self.shared.update_status(
                    lidar_active=False,
                    lidar_connected=False,
                    lidar_status="error",
                    lidar_error=str(exc),
                    status_message=msg,
                )
                self.log(msg)
            finally:
                self._cleanup_lidar(full_shutdown=self._shutdown_on_stop or force_shutdown)
                self.shared.update_status(
                    lidar_active=False,
                    lidar_connected=self._connected,
                    lidar_status="connected" if self._connected else "not_connected",
                )
                if self._shutdown_on_stop or force_shutdown:
                    self.shared.clear_lidar_ready()

            if self._stream_stop.is_set() or self._shutdown_on_stop:
                break

            time.sleep(LOOP_PAUSE)

    def _resolve_port(self) -> str | None:
        configured = (self.settings.lidar_port or "").strip()
        if configured:
            return configured
        hint = str(self.settings.get("lidar_port_hint", "") or "").strip()
        if hint:
            return hint
        return _find_lidar_port()

    def _cleanup_lidar(self, full_shutdown: bool) -> None:
        lidar = self._lidar
        if lidar is None:
            return
        try:
            lidar.stop()
        except Exception:
            pass
        if full_shutdown:
            try:
                lidar.stop_motor()
            except Exception:
                pass
            try:
                lidar.disconnect()
            except Exception:
                pass
            with self._connect_lock:
                self._lidar = None
                self._connected = False
            self._reset_cluster_history()

    # Noise filtering -----------------------------------------------------------
    def _reset_cluster_history(self) -> None:
        self._cluster_history.clear()
        self._update_cluster_debug(0, False, 0, self._required_frames(), reason="")

    def _filter_noise(self, points: List[Point]) -> tuple[List[Point], dict[str, object]]:
        now = time.time()
        clusters = self._build_clusters(points)
        self._cluster_history.append((now, clusters))
        cutoff = now - CLUSTER_PERSISTENCE_SEC
        while self._cluster_history and self._cluster_history[0][0] < cutoff:
            self._cluster_history.popleft()

        required_frames = self._required_frames()
        stable_clusters: List[_Cluster] = []
        frames_stable = 0

        for cluster in clusters:
            occurrences = self._count_matching_frames(cluster)
            frames_stable = max(frames_stable, occurrences)
            if occurrences >= required_frames:
                stable_clusters.append(cluster)

        stable_points = [pt for cluster in stable_clusters for pt in cluster.points]
        largest_cluster = max((c.size for c in clusters), default=0)

        if stable_clusters:
            reason = "Obstacle cluster validated"
        elif not clusters:
            reason = "Noise filtered — insufficient points"
        elif frames_stable < required_frames:
            reason = "Noise filtered — unstable cluster"
        else:
            reason = "Noise filtered — insufficient points"

        debug = {
            "cluster_size": largest_cluster,
            "validated": bool(stable_clusters),
            "frames_stable": frames_stable,
            "frames_required": required_frames,
            "reason": reason,
        }
        return stable_points, debug

    def _build_clusters(self, points: List[Point]) -> List[_Cluster]:
        if not points:
            return []
        pts = sorted(points, key=lambda p: p[0])
        clusters: List[_Cluster] = []
        current: List[Point] = [pts[0]]
        for pt in pts[1:]:
            if self._points_connected(current[-1], pt):
                current.append(pt)
            else:
                if len(current) >= CLUSTER_MIN_POINTS:
                    clusters.append(self._cluster_from_points(current))
                current = [pt]
        if len(current) >= CLUSTER_MIN_POINTS:
            clusters.append(self._cluster_from_points(current))
        return clusters

    def _points_connected(self, a: Point, b: Point) -> bool:
        ang_gap = abs(a[0] - b[0])
        dist_gap = abs(a[1] - b[1])
        return ang_gap <= CLUSTER_ANGLE_TOLERANCE_DEG and dist_gap <= CLUSTER_DISTANCE_TOLERANCE_CM

    def _cluster_from_points(self, points: List[Point]) -> _Cluster:
        angles = [p[0] for p in points]
        distances = [p[1] for p in points]
        angle_center = sum(angles) / len(angles)
        min_distance = min(distances)
        return _Cluster(points=list(points), angle_center=angle_center, min_distance=min_distance)

    def _count_matching_frames(self, target: _Cluster) -> int:
        matches = 0
        for _, clusters in self._cluster_history:
            if any(self._clusters_close(target, candidate) for candidate in clusters):
                matches += 1
        return matches

    def _clusters_close(self, a: _Cluster, b: _Cluster) -> bool:
        return (
            abs(a.angle_center - b.angle_center) <= CLUSTER_ANGLE_TOLERANCE_DEG
            and abs(a.min_distance - b.min_distance) <= CLUSTER_DISTANCE_TOLERANCE_CM
        )

    def _required_frames(self) -> int:
        hz = float(self._estimated_hz or self.shared.lidar_read_hz or config.DEFAULT_LIDAR_READ_HZ)
        hz = max(1.0, min(20.0, hz))
        return max(CLUSTER_MIN_POINTS, int(round(hz * CLUSTER_PERSISTENCE_SEC)))

    def _update_cluster_debug(
        self, cluster_size: int, validated: bool, frames_stable: int, frames_required: int, reason: str
    ) -> None:
        message = reason or ("Obstacle cluster validated" if validated else "")
        self.shared.update_status(
            lidar_cluster_size=int(cluster_size),
            lidar_cluster_valid=bool(validated),
            lidar_frames_stable=int(frames_stable),
            lidar_frames_required=int(frames_required),
            lidar_noise_reason=message,
        )

