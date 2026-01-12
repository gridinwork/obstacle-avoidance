"""Obstacle avoidance logic driven by LiDAR point cloud.
Version 76: Clean RC-override-only model, no GPS/compass/angle-based movement."""

from __future__ import annotations

import math
import random
import threading
import time
from typing import Iterable, Tuple

from PyQt5 import QtCore

from lidar_core import MAX_DIST_CM
from shared_data import Point, SharedData
from settings import Settings
from mavlink_comm import MavlinkComm
from motion_control import MotionControl
from config import ROTATION_TIME_SEC, FORWARD_TIME_SEC, SPEED_CAP_PERCENT


class AvoidanceController(QtCore.QObject):
    """Three-zone obstacle avoidance using time-based RC override commands only."""

    statusChanged = QtCore.pyqtSignal(str)

    CLEAR_THRESHOLD_CM = 300.0  # >3.0 m
    GREEN_THRESHOLD_CM = 200.0  # 2.0–3.0 m
    YELLOW_THRESHOLD_CM = 100.0  # 1.0–2.0 m
    # Primary Forward Corridor (PFC) distance boundaries aligned to thresholds
    RED_LINE_CM = YELLOW_THRESHOLD_CM  # 0–RED_LINE => red
    ORANGE_LINE_CM = GREEN_THRESHOLD_CM  # RED_LINE–ORANGE_LINE => yellow
    GREEN_LINE_CM = CLEAR_THRESHOLD_CM  # ORANGE_LINE–GREEN_LINE => green

    def __init__(self, shared: SharedData, settings: Settings, mav: MavlinkComm, motion: MotionControl, log_fn=print) -> None:
        super().__init__()
        self.shared = shared
        self.settings = settings
        self.mav = mav
        self.motion = motion
        self.log = log_fn
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._executing = False
        self.state = "clear"
        self.selected_direction = "RIGHT"
        self._last_status_text = ""
        self._side_counts: tuple[int, int] = (0, 0)
        self._last_status_emit_ts: float = 0.0
        self._last_command_ts: float = 0.0
        self._zone_entry_ts: float = 0.0
        self._last_min_dist: float = float("inf")
        self._dwell_required_sec: float = 0.5
        self._last_check_ts: float = 0.0
        self._last_mode_change_ts: float = 0.0
        self._last_dir_change_ts: float = 0.0

    # Lifecycle -----------------------------------------------------------------
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.log("Avoidance controller thread started")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2)
        self._last_status_text = ""
        self.state = "clear"
        self._executing = False

    # Core loop -----------------------------------------------------------------
    def _loop(self) -> None:
        while not self._stop_event.is_set() and self.shared.running:
            now_global = time.time()
            if (now_global - self._last_check_ts) < 0.5:
                time.sleep(0.05)
                continue
            self._last_check_ts = now_global

            # Respect action lockout: do not analyze or issue new commands during maneuvers
            if self.shared.action_in_progress:
                remaining = self.shared.action_end_time - time.time()
                if remaining > 0:
                    label = self.shared.action_label or "Action in progress"
                    # Skip LIDAR processing during maneuvers
                    self._emit_status(f"{label} — LIDAR scanning paused ({remaining:.1f}s remaining)")
                    time.sleep(0.05)
                    continue
                # Maneuver finished, resume LIDAR scanning
                self.shared.update_status(action_in_progress=False, action_end_time=0.0, action_label="")
                self._emit_status("Maneuver finished — LIDAR scanning resumed")
                time.sleep(0.05)
                continue

            if self.shared.manual_override or self.shared.manual_control:
                time.sleep(0.1)
                continue

            if not self.shared.obstacle_avoidance_on:
                self._reset_state(clear_status=True)
                time.sleep(0.1)
                continue

            if not self.shared.lidar_connected:
                time.sleep(0.1)
                continue

            points = self.shared.get_validated_lidar_points() or self.shared.get_lidar_points()
            if not points:
                time.sleep(0.05)
                continue

            min_dist, left_count, right_count = self._analyze_points(points)
            self.shared.update_status(last_obstacle_cm=min_dist)
            self.selected_direction = self._pick_direction(left_count, right_count)

            # Dwell + approaching filter (0.5s minimum)
            now = time.time()
            if min_dist > self.CLEAR_THRESHOLD_CM:
                self._zone_entry_ts = 0.0
                self._last_min_dist = float("inf")
                self._handle_clear()
                time.sleep(0.05)
                continue

            if self._zone_entry_ts <= 0:
                self._zone_entry_ts = now
                self._last_min_dist = min_dist
                time.sleep(0.05)
                continue

            # Require dwell
            if (now - self._zone_entry_ts) < self._dwell_required_sec:
                self._last_min_dist = min_dist
                time.sleep(0.05)
                continue

            # Require approaching (distance decreasing)
            if min_dist >= (self._last_min_dist - 0.5):
                self._last_min_dist = min_dist
                time.sleep(0.05)
                continue

            self._last_min_dist = min_dist

            if min_dist <= self.RED_LINE_CM:
                self._handle_red()
            elif min_dist <= self.GREEN_THRESHOLD_CM:
                self._handle_yellow()
            elif min_dist <= self.CLEAR_THRESHOLD_CM:
                self._handle_green()
            else:
                self._handle_clear()

            time.sleep(0.05)

    # State handlers ------------------------------------------------------------
    def _handle_clear(self) -> None:
        now = time.time()
        if (now - self._last_mode_change_ts) < 0.5:
            return
        self._last_mode_change_ts = now
        target_mode = self._resume_mode_target()
        self._emit_status(f"{target_mode} MODE — Clear path")
        self.state = "clear"
        self._executing = False
        if self._can_command():
            self._restore_previous_mode(target_mode)

    def _handle_green(self) -> None:
        """Green zone: obstacle detected but far enough, continue normal operation."""
        now = time.time()
        if (now - self._last_mode_change_ts) < 0.5:
            return
        self._last_mode_change_ts = now
        self.state = "green"
        msg = (
            f"GREEN ZONE — Obstacle detected but clear "
            f"(Side: L={self._side_counts[0]} R={self._side_counts[1]})"
        )
        self._emit_status(msg)
        # No movement commands in green zone - let normal mission continue

    def _handle_yellow(self) -> None:
        """Yellow zone: execute timed turn and forward movement maneuver."""
        if self._executing or self.shared.action_in_progress:
            return
        now = time.time()
        if (now - self._last_mode_change_ts) < 0.5:
            return
        self._last_mode_change_ts = now
        self._executing = True
        self.state = "yellow"
        turn_dir = self.selected_direction
        
        if not self._can_command():
            self._executing = False
            return
        
        # Calculate total maneuver time
        total_time = ROTATION_TIME_SEC + FORWARD_TIME_SEC + 0.5  # Add buffer
        
        # STOP LIDAR scanning during maneuver
        self.shared.update_status(
            action_in_progress=True,
            action_end_time=time.time() + total_time,
            action_label="YELLOW ZONE maneuver in progress"
        )
        
        # Clear LIDAR points to prevent processing during maneuver
        with self.shared.lidar_lock:
            self.shared.set_lidar_points([])
            self.shared.set_validated_lidar_points([])
        
        self._remember_mode_before_guided()
        self.mav.ensure_guided()
        
        # Sync connection if needed
        if self.motion.movement_controller.connection is None and self.mav.master:
            self.motion.movement_controller.connection = self.mav.master
        
        try:
            # Execute timed maneuver based on obstacle side
            if turn_dir == "RIGHT":
                # Obstacle on left side -> turn RIGHT
                self._emit_status("YELLOW ZONE — Turning RIGHT (timed)")
                self.motion.movement_controller.timed_turn_right(ROTATION_TIME_SEC, SPEED_CAP_PERCENT)
            else:
                # Obstacle on right side -> turn LEFT
                self._emit_status("YELLOW ZONE — Turning LEFT (timed)")
                self.motion.movement_controller.timed_turn_left(ROTATION_TIME_SEC, SPEED_CAP_PERCENT)
            
            # After turn, move forward for a timed duration
            self.motion.movement_controller.forward_for(FORWARD_TIME_SEC, SPEED_CAP_PERCENT)
            
            # Stop after forward movement
            self.motion.movement_controller.stop()
            
        except Exception as exc:
            self.log(f"Yellow zone maneuver failed: {exc}")
            self.motion.movement_controller.stop()
        finally:
            # Resume LIDAR scanning after maneuver
            self.shared.update_status(action_in_progress=False, action_end_time=0.0, action_label="")
            self._restore_previous_mode()
            self._executing = False

    def _handle_red(self) -> None:
        """Red zone: stop and reverse."""
        if self._executing or self.shared.action_in_progress:
            return
        now = time.time()
        if (now - self._last_mode_change_ts) < 0.5:
            return
        self._last_mode_change_ts = now
        self._executing = True
        prev_state = self.state
        self.state = "red"
        if prev_state == "red":
            self._emit_status("Obstacle still in RED zone — repeating reverse")
        else:
            self._emit_status("RED ZONE — STOPPING then REVERSE")
        if not self._can_command():
            self._executing = False
            return
        
        # Stop first
        if self.motion.movement_controller.connection is None and self.mav.master:
            self.motion.movement_controller.connection = self.mav.master
        self.motion.movement_controller.stop()
        
        self._remember_mode_before_guided()
        self.mav.ensure_guided()
        self._reverse_until_yellow()
        self._executing = False

    # Helpers -------------------------------------------------------------------
    def _remember_mode_before_guided(self) -> None:
        """Capture the active mode before switching to GUIDED for avoidance."""
        mode = (getattr(self.shared, "current_mode", "") or "").strip().upper()
        if not mode or mode == "UNKNOWN":
            return
        if getattr(self.shared, "pre_avoidance_mode", None):
            return
        self.shared.update_status(pre_avoidance_mode=mode)

    def _resume_mode_target(self) -> str:
        """Determine which mode to return to after avoidance."""
        return self.mav.preferred_mode(
            getattr(self.shared, "user_return_mode", None),
            getattr(self.shared, "pre_avoidance_mode", None),
            getattr(self.shared, "initial_mode", None),
            "AUTO",
        )

    def _restore_previous_mode(self, target_mode: str | None = None) -> None:
        desired_mode = self.mav.preferred_mode(
            getattr(self.shared, "user_return_mode", None),
            target_mode,
            getattr(self.shared, "pre_avoidance_mode", None),
            getattr(self.shared, "initial_mode", None),
            "AUTO",
        )
        current_mode = (self.shared.current_mode or "").strip().upper()
        if desired_mode and current_mode == desired_mode:
            self.shared.update_status(pre_avoidance_mode=None)
            return
        restore_fn = self.mav.return_to_auto if desired_mode == "AUTO" else lambda: self.mav.set_mode(desired_mode)
        executed = self._run_command(f"Return to {desired_mode}", restore_fn)
        if executed:
            self.shared.update_status(pre_avoidance_mode=None)

    def _emit_status(self, message: str) -> None:
        now = time.time()
        min_gap = max(2.0, float(self.shared.command_cooldown_sec or 0.0))
        if message == self._last_status_text and (now - self._last_status_emit_ts) < min_gap:
            return
        allowed = (
            message.endswith("Clear path")
            or message.startswith("GREEN ZONE")
            or message.startswith("YELLOW ZONE")
            or message.startswith("RED ZONE")
            or message.startswith("Obstacle still in RED")
            or "maneuver" in message.lower()
            or "LIDAR" in message
        )
        if not allowed:
            return
        if message and (now - self._last_status_emit_ts) >= min_gap:
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
            self.log(f"{ts} {message}")
        self.statusChanged.emit(message)
        self.shared.update_status(status_message=message)
        self._last_status_text = message
        self._last_status_emit_ts = now

    def _reset_state(self, clear_status: bool = False) -> None:
        self.state = "clear"
        self.selected_direction = "RIGHT"
        self._executing = False
        if clear_status:
            self._emit_status("")

    def _analyze_points(self, points: Iterable[Point]) -> tuple[float, int, int]:
        """Analyze LIDAR points to determine minimum distance and side counts."""
        min_dist = self._min_distance_corridor(points)
        left = 0
        right = 0
        blue_boundary_cm, green_boundary_cm, _ = self.shared.get_boundaries()
        for ang, dist in points:
            if not self._is_front(ang):
                continue
            x_cm, _ = self._to_xy(ang, dist)
            # Side windows between blue and green lines only for direction
            if -green_boundary_cm <= x_cm < -blue_boundary_cm:
                left += 1
            elif blue_boundary_cm < x_cm <= green_boundary_cm:
                right += 1
        return min_dist, left, right

    def _pick_direction(self, left_count: int, right_count: int) -> str:
        """Pick turn direction based on obstacle side counts."""
        now = time.time()
        if left_count > right_count:
            choice = "RIGHT"
        elif right_count > left_count:
            choice = "LEFT"
        else:
            choice = random.choice(["LEFT", "RIGHT"])
        if (now - self._last_dir_change_ts) >= 0.5:
            self._last_dir_change_ts = now
            self._side_counts = (left_count, right_count)
            self.selected_direction = choice
            self._emit_status(f"Side check: Left={left_count} Right={right_count} → turning {choice}")
        return getattr(self, "selected_direction", choice)

    def _reverse_until_yellow(self) -> None:
        """Reverse until the obstacle exits the red zone (enters yellow/green) or timeout."""
        if not self._can_command():
            return
        
        reverse_pct = max(0.0, min(getattr(self.settings, "speed_limit3_pct", 40), 100))
        max_reverse = max(1.0, float(self.shared.reverse_duration_sec or 0.0))
        deadline = time.time() + max_reverse
        label = "RED ZONE — reversing until yellow"
        
        # Pause LIDAR during reverse
        self.shared.update_status(
            action_in_progress=True,
            action_end_time=deadline,
            action_label=label,
            movement_state="REVERSING",
        )
        self._emit_status(label)
        
        # Clear LIDAR points
        with self.shared.lidar_lock:
            self.shared.set_lidar_points([])
            self.shared.set_validated_lidar_points([])
        
        try:
            # Sync connection if needed
            if self.motion.movement_controller.connection is None and self.mav.master:
                self.motion.movement_controller.connection = self.mav.master
            
            if self.motion.movement_controller.connection:
                # Use timed backward movement
                self.motion.movement_controller.backward_for(max_reverse, reverse_pct)
            
            # Monitor obstacle distance while reversing (using last known points)
            while time.time() < deadline and self.shared.running:
                # Check if we can get new points (after pause ends)
                if not self.shared.action_in_progress:
                    points = self.shared.get_validated_lidar_points() or self.shared.get_lidar_points()
                    if points:
                        current = self._current_min_distance_from_points(points)
                        if current > self.RED_LINE_CM:
                            self._emit_status("RED ZONE — exited red, now yellow/clear")
                            break
                if not self._can_command():
                    break
                time.sleep(0.1)
        finally:
            self.motion.movement_controller.stop()
            self.shared.update_status(action_in_progress=False, action_end_time=0.0, action_label="")

    def _current_min_distance(self) -> float:
        """Get current minimum distance from LIDAR points."""
        pts = self.shared.get_validated_lidar_points() or self.shared.get_lidar_points()
        return self._min_distance_corridor(pts)

    def _current_min_distance_from_points(self, points: Iterable[Point]) -> float:
        """Get minimum distance from provided points."""
        return self._min_distance_corridor(points)

    def _min_distance_corridor(self, points: Iterable[Point]) -> float:
        """Only consider obstacles inside the Primary Forward Corridor (blue lines)."""
        blue_boundary_cm, _, _ = self.shared.get_boundaries()
        min_dist = float("inf")
        for ang, dist in points:
            if not self._is_front(ang):
                continue
            x_cm, _ = self._to_xy(ang, dist)
            if abs(x_cm) > blue_boundary_cm:
                # Outside PFC: ignore for threat distance but still useful for side counts
                continue
            min_dist = min(min_dist, dist)
        if min_dist == float("inf"):
            return MAX_DIST_CM
        return min_dist

    def _to_xy(self, ang_deg: float, dist_cm: float) -> Tuple[float, float]:
        """Convert angle and distance to x,y coordinates (for obstacle detection only, not movement)."""
        rad = math.radians(ang_deg)
        x_cm = dist_cm * math.sin(rad)
        y_cm = dist_cm * math.cos(rad)
        return x_cm, y_cm

    def _is_front(self, ang_deg: float) -> bool:
        """Check if angle is in front sector."""
        return -90.0 <= ang_deg <= 90.0

    def _can_command(self) -> bool:
        """Check if MAVLink commands can be sent."""
        return bool(self.shared.mav_connected)

    def _command_cooldown_passed(self) -> bool:
        """Check if command cooldown period has passed."""
        gap = max(2.0, float(self.shared.command_cooldown_sec or 0.0))
        return (time.time() - self._last_command_ts) >= gap

    def _run_command(self, label: str, fn) -> bool:
        """Execute a command with cooldown check."""
        if not self._command_cooldown_passed():
            return False
        try:
            fn()
        finally:
            self._last_command_ts = time.time()
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self._last_command_ts))
            self.log(f"{ts} {label}")
        return True
