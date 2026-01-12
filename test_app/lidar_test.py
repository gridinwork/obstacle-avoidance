from __future__ import annotations

import math
import random
from typing import List, Tuple

from PyQt5 import QtCore

try:
    from rplidar import RPLidar
except Exception:  # pragma: no cover - optional dependency on dev machines
    RPLidar = None

Point = Tuple[float, float]


class LidarWorker(QtCore.QObject):
    """Background LiDAR reader that emits point clouds."""

    points_ready = QtCore.pyqtSignal(object)
    status = QtCore.pyqtSignal(bool, str)
    finished = QtCore.pyqtSignal()

    def __init__(self, port: str | None, simulated: bool = False) -> None:
        super().__init__()
        self.port = port
        self.simulated = simulated
        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def _emit_simulated_points(self) -> None:
        self.status.emit(True, "Simulation Mode")
        while not self._stop:
            points: List[Point] = []
            for angle in range(0, 360, 4):
                distance = random.uniform(500.0, 2500.0)
                rad = math.radians(angle)
                points.append((distance * math.cos(rad), distance * math.sin(rad)))
            self.points_ready.emit(points)
            QtCore.QThread.msleep(80)
        self.status.emit(True, "LiDAR stream stopped")

    def run(self) -> None:
        if self.simulated or RPLidar is None:
            self._emit_simulated_points()
            self.finished.emit()
            return

        lidar = None
        try:
            lidar = RPLidar(self.port, baudrate=115200)
            lidar.start_motor()
            lidar.set_pwm(660)
            self.status.emit(True, f"Streaming {self.port}")
            for scan in lidar.iter_scans(max_buf_meas=500):
                if self._stop:
                    break
                points: List[Point] = []
                for _, angle, distance in scan:
                    rad = math.radians(angle)
                    points.append((distance * math.cos(rad), distance * math.sin(rad)))
                self.points_ready.emit(points)
        except Exception as exc:
            self.status.emit(False, f"LiDAR error: {exc}")
            # On failure, continue with a simulated cloud so the UI keeps running.
            self._emit_simulated_points()
        finally:
            self.status.emit(True, "LiDAR stream stopped")
            self.finished.emit()


class LidarHandler(QtCore.QObject):
    """Manages LiDAR connection state and worker threads."""

    points_ready = QtCore.pyqtSignal(object)
    status_changed = QtCore.pyqtSignal(bool, str)

    def __init__(self) -> None:
        super().__init__()
        self.port: str | None = None
        self.connected = False
        self.simulated = False
        self._streaming = False
        self._thread: QtCore.QThread | None = None
        self._worker: LidarWorker | None = None

    def connect_device(self, port: str) -> bool:
        """Try to connect to the real device; fall back to simulation on failure."""
        self.port = port
        self.simulated = False
        self.connected = False

        if RPLidar is None:
            self.simulated = True
            self.status_changed.emit(True, "Simulation Mode (rplidar not installed)")
            return False

        try:
            probe = RPLidar(port, baudrate=115200)
            self.connected = True
            self.status_changed.emit(True, f"Connected to {port}")
        except Exception as exc:
            self.simulated = True
            self.connected = False
            self.status_changed.emit(True, f"Simulation Mode (connect failed: {exc})")

        return self.connected

    def start_stream(self) -> None:
        if not self.port:
            self.status_changed.emit(False, "No LiDAR port selected")
            return
        if self._streaming:
            return
        if not self.connected and not self.simulated:
            self.connect_device(self.port)

        self._thread = QtCore.QThread()
        self._worker = LidarWorker(self.port, simulated=self.simulated or not self.connected)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.points_ready.connect(self.points_ready)
        self._worker.status.connect(self.status_changed)
        self._worker.finished.connect(self._thread.quit)
        self._worker.finished.connect(self._worker.deleteLater)
        self._thread.finished.connect(self._thread.deleteLater)
        self._thread.finished.connect(self._on_thread_finished)

        self._streaming = True
        self._thread.start()

    def _on_thread_finished(self) -> None:
        self._thread = None
        self._worker = None
        self._streaming = False

    def stop_stream(self) -> None:
        if self._worker:
            self._worker.stop()
        if self._thread:
            self._thread.quit()
            self._thread.wait()

    def disconnect(self) -> None:
        self.stop_stream()
        self.connected = False
        self.simulated = False
        self.port = None
        self.status_changed.emit(False, "LiDAR disconnected")

    def is_streaming(self) -> bool:
        return self._streaming

