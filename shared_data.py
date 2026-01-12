"""Thread-safe shared state across GUI, LiDAR, avoidance, and MAVLink."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import List, Tuple

import config
from settings import Settings

Point = Tuple[float, float]  # (angle_deg, distance_cm)


@dataclass
class SharedData:
    settings: Settings
    running: bool = True
    lidar_points: List[Point] = field(default_factory=list)
    lidar_points_validated: List[Point] = field(default_factory=list)
    lidar_active: bool = False
    lidar_connected: bool = False
    mav_connected: bool = False
    camera_connected: bool = False
    mission_active: bool = False
    avoidance_enabled: bool = False
    obstacle_avoidance_on: bool = False
    show_avoidance_arrows: bool = True
    show_lidar: bool = True
    show_camera: bool = False
    visualization_running: bool = False
    current_mode: str = "UNKNOWN"
    user_return_mode: str | None = None
    initial_mode: str | None = None
    pre_avoidance_mode: str | None = None
    gps_fix: bool = False
    base_speed_pwm: int = 1500
    limit1_cm: int = 120
    limit2_cm: int = 70
    limit3_cm: int = 50
    speed_limit1_pct: int = 70
    speed_limit2_pct: int = 40
    speed_limit3_pct: int = 20
    avoidance_turn_deg: int = 90
    avoidance_distance_m: float = 2.0
    avoidance_backoff_m: float = 1.0
    next_direction: str = "NONE"
    next_distance_m: float = 0.0
    next_angle_deg: float = 0.0
    avoidance_attempts: int = 0
    avoidance_warning: str = ""
    movement_state: str = "STOPPED"
    last_obstacle_cm: float | None = None
    last_free_sector_deg: float | None = None
    desired_heading_deg: float | None = None
    current_heading_deg: float | None = None
    status_message: str = ""
    camera_source: str = "OFF"
    camera_mode: str = "OFF"
    camera_device: str = "OFF"
    camera_backend: str = "auto"
    lidar_status: str = "not_connected"
    mav_status: str = "not_connected"
    camera_status: str = "not_connected"
    lidar_error: str = ""
    mav_error: str = ""
    camera_error: str = ""
    last_lidar_ts: float = 0.0
    manual_control: bool = False
    manual_override: bool = False
    start_fullscreen: bool = True
    blue_boundary_cm: int = config.DEFAULT_BLUE_BOUNDARY_CM
    green_boundary_cm: int = config.DEFAULT_GREEN_BOUNDARY_CM
    green_limit_cm: int = config.GREEN_MAX_CM
    lidar_buffer_size: int = config.DEFAULT_LIDAR_BUFFER_SIZE
    lidar_read_hz: float = config.DEFAULT_LIDAR_READ_HZ
    lidar_draw_ready_ts: float = 0.0
    avoidance_cooldown_ts: float = 0.0
    lidar_cluster_size: int = 0
    lidar_cluster_valid: bool = False
    lidar_noise_reason: str = ""
    lidar_frames_stable: int = 0
    lidar_frames_required: int = 0
    action_in_progress: bool = False
    action_end_time: float = 0.0
    action_label: str = ""
    reverse_duration_sec: float = 2.0
    post_turn_lock_duration_sec: float = 3.0
    command_cooldown_sec: float = 1.0
    movement_coefficient: float = 1.0

    lidar_lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    camera_lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    state_lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    camera_frame: object | None = None
    manual_channels: dict = field(
        default_factory=lambda: {"ch1": 1500, "ch3": 1500, "ch5": 1000, "ch6": 1000, "ch8": 1000}
    )

    def __post_init__(self) -> None:
        self.show_lidar = self.settings.show_lidar
        self.base_speed_pwm = self.settings.base_speed_pwm
        self.limit1_cm = self.settings.limit1_cm
        self.limit2_cm = self.settings.limit2_cm
        self.limit3_cm = self.settings.limit3_cm
        self.speed_limit1_pct = self.settings.speed_limit1_pct
        self.speed_limit2_pct = self.settings.speed_limit2_pct
        self.speed_limit3_pct = self.settings.speed_limit3_pct
        self.avoidance_turn_deg = int(self.settings.avoidance_turn_deg)
        self.avoidance_distance_m = float(self.settings.avoidance_distance_m)
        self.avoidance_backoff_m = float(self.settings.avoidance_backoff_m)
        self.camera_device = self.settings.camera_device
        self.camera_backend = self.settings.camera_backend
        self.camera_source = self.settings.camera_device
        self.camera_mode = self.settings.camera_device
        self.show_camera = self.settings.show_camera
        self.show_avoidance_arrows = bool(self.settings.get("show_avoidance_arrows", True))
        self.start_fullscreen = self.settings.start_fullscreen
        self.blue_boundary_cm = int(self.settings.blue_boundary_cm)
        self.green_boundary_cm = int(self.settings.green_boundary_cm)
        self.green_limit_cm = int(self.settings.green_limit_cm)
        self.lidar_buffer_size = int(self.settings.lidar_buffer_size)
        self.lidar_read_hz = float(self.settings.lidar_read_hz)
        self.obstacle_avoidance_on = bool(self.settings.get("obstacle_avoidance_on", False))
        self.reverse_duration_sec = float(self.settings.get("reverse_duration_sec", 2.0))
        self.post_turn_lock_duration_sec = float(self.settings.get("post_turn_lock_duration_sec", 3.0))
        self.command_cooldown_sec = float(self.settings.get("command_cooldown_sec", 1.0))
        self.movement_coefficient = float(self.settings.get("movement_coefficient", 1.0))
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self.running = False
        self._stop_event.set()

    def stopped(self) -> bool:
        return self._stop_event.is_set()

    def set_lidar_points(self, pts: List[Point]) -> None:
        with self.lidar_lock:
            self.lidar_points = pts
            self.last_lidar_ts = time.time()

    def get_lidar_points(self) -> List[Point]:
        with self.lidar_lock:
            return list(self.lidar_points)

    def set_validated_lidar_points(self, pts: List[Point]) -> None:
        with self.lidar_lock:
            self.lidar_points_validated = pts

    def get_validated_lidar_points(self) -> List[Point]:
        with self.lidar_lock:
            return list(self.lidar_points_validated)

    def update_status(self, **kwargs) -> None:
        with self.state_lock:
            for k, v in kwargs.items():
                setattr(self, k, v)

    def set_camera_frame(self, frame) -> None:
        with self.camera_lock:
            self.camera_frame = frame

    def get_camera_frame(self):
        with self.camera_lock:
            return None if self.camera_frame is None else self.camera_frame.copy()

    def get_boundaries(self) -> tuple[int, int, int]:
        with self.state_lock:
            return self.blue_boundary_cm, self.green_boundary_cm, self.green_limit_cm

    def get_lidar_config(self) -> tuple[int, float]:
        with self.state_lock:
            return int(self.lidar_buffer_size), float(self.lidar_read_hz)

    # Visualization timing -----------------------------------------------------
    def mark_lidar_ready_after(self, delay_sec: float) -> None:
        """Schedule LiDAR visualization to become eligible after delay_sec."""
        with self.state_lock:
            self.lidar_draw_ready_ts = time.time() + max(0.0, delay_sec)

    def clear_lidar_ready(self) -> None:
        """Reset visualization readiness timestamp."""
        with self.state_lock:
            self.lidar_draw_ready_ts = 0.0

    def lidar_ready(self) -> bool:
        """Return True when visualization may render LiDAR points."""
        with self.state_lock:
            ts = self.lidar_draw_ready_ts
        return ts <= 0 or time.time() >= ts

    def lidar_ready_at(self) -> float:
        """Return the absolute timestamp when drawing is allowed."""
        with self.state_lock:
            return float(self.lidar_draw_ready_ts)

    def get_next_maneuver(self) -> tuple[str, float, float]:
        """Return upcoming avoidance maneuver details for visualization."""
        with self.state_lock:
            return (
                str(self.next_direction),
                float(self.next_distance_m),
                float(self.next_angle_deg),
            )

