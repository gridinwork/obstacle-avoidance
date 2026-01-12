import glob
import json
import os
import platform
from pathlib import Path
from typing import Any, Iterable, List

import config

PIXHAWK_DEFAULT_PORT = "/dev/serial0"  # Fixed UART on Raspberry Pi
LIDAR_DEFAULT_PORT = (
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)
DEFAULT_CAMERA_DEVICE = "PI_CAMERA"

# --------------------------------------------------------------------------- #
# Helpers for Linux/Raspberry Pi device discovery
# --------------------------------------------------------------------------- #


def is_raspberry_pi() -> bool:
    """Best-effort detection for Raspberry Pi boards."""
    try:
        model = Path("/proc/device-tree/model")
        if model.exists():
            return "raspberry pi" in model.read_text(encoding="utf-8").lower()
    except Exception:
        pass
    return False


def _dedup_preserve(items: Iterable[str]) -> List[str]:
    seen = set()
    deduped: List[str] = []
    for item in items:
        if not item or item in seen:
            continue
        seen.add(item)
        deduped.append(item)
    return deduped


def _glob_devices(patterns: Iterable[str]) -> List[str]:
    """Return existing device paths for the provided glob patterns."""
    devices: List[str] = []
    for pattern in patterns:
        for path in glob.glob(pattern):
            if os.path.exists(path):
                devices.append(path)
    return devices


def _list_serial_with_prefix(prefixes: tuple[str, ...]) -> List[str]:
    """
    v48: Pixhawk is fixed to UART; we no longer scan USB. Return the fixed port.
    """
    return [PIXHAWK_DEFAULT_PORT]


def list_pixhawk_ports() -> List[str]:
    """v48: Pixhawk is fixed on UART; return the fixed port only."""
    return [PIXHAWK_DEFAULT_PORT]


def list_lidar_ports() -> List[str]:
    """Return only LiDAR-style USB serial ports (/dev/ttyUSB*)."""
    ports = _list_serial_with_prefix(("/dev/ttyUSB", "/dev/ttyAMA", "/dev/serial/by-id"))
    ports.extend(
        _glob_devices(
            (
                "/dev/ttyUSB0",
                "/dev/ttyUSB1",
                "/dev/serial/by-id/*",
            )
        )
    )
    return _dedup_preserve(ports)


def list_camera_devices() -> List[str]:
    """Detect USB cameras exposed via V4L2."""
    if os.name == "nt":
        return []
    return sorted(_dedup_preserve(p for p in glob.glob("/dev/video*") if os.path.exists(p)))


# Default configuration persisted to settings.json.
DEFAULTS = {
    "mavlink_port": PIXHAWK_DEFAULT_PORT,
    "lidar_port": LIDAR_DEFAULT_PORT,
    "lidar_port_hint": LIDAR_DEFAULT_PORT,
    "mavlink_baud": 57600,
    "pixhawk_series": "Series 0",
    "auto_start": False,
    "base_speed_pwm": 1500,
    "limit1_cm": 120,
    "limit2_cm": 70,
    "limit3_cm": 50,
    "speed_limit1_pct": 70,
    "speed_limit2_pct": 40,
    "speed_limit3_pct": 20,
    "avoidance_turn_deg": 60,
    "avoidance_distance_m": 2.0,
    "avoidance_backoff_m": 0.7,
    "reverse_duration_sec": 1.0,
    "post_turn_lock_duration_sec": 1.0,
    "command_cooldown_sec": 1.0,
    "show_avoidance_arrows": True,
    "obstacle_avoidance_on": False,
    "show_lidar": True,
    "camera_device": DEFAULT_CAMERA_DEVICE,  # PI_CAMERA | OFF | /dev/videoX
    "camera_backend": "auto",  # auto | libcamera | v4l2
    "show_camera": False,
    "start_fullscreen": True,
    "blue_boundary_cm": config.DEFAULT_BLUE_BOUNDARY_CM,
    "green_boundary_cm": config.DEFAULT_GREEN_BOUNDARY_CM,
    "green_limit_cm": config.GREEN_MAX_CM,
    "lidar_buffer_size": config.DEFAULT_LIDAR_BUFFER_SIZE,
    "lidar_read_hz": config.DEFAULT_LIDAR_READ_HZ,
    "movement_coefficient": 1.0,  # m/s for no-GPS distance-to-time conversion
}


class Settings:
    """Load/save persistent configuration to JSON."""

    def __init__(self, path: str | Path | None = None) -> None:
        self.path = Path(path) if path else Path(__file__).with_name("settings.json")
        self.data: dict[str, Any] = DEFAULTS.copy()
        self.load()

    def load(self) -> None:
        if not self.path.exists():
            return
        try:
            self.data.update(json.loads(self.path.read_text(encoding="utf-8")))
            self._migrate_keys()
        except Exception:
            # Keep defaults if file is malformed.
            self.data = DEFAULTS.copy()

    def save(self) -> None:
        try:
            self.path.write_text(json.dumps(self.data, indent=2), encoding="utf-8")
        except Exception:
            # Silent fail; GUI will continue using in-memory values.
            pass

    def get(self, key: str, fallback: Any | None = None) -> Any:
        if key in self.data:
            return self.data[key]
        return fallback if fallback is not None else DEFAULTS.get(key)

    def set(self, key: str, value: Any) -> None:
        self.data[key] = value
        self.save()

    # Migration helpers ---------------------------------------------------------
    def _migrate_keys(self) -> None:
        """Normalize legacy keys from earlier versions."""
        if "auto_start" not in self.data and "auto_start_after_5s" in self.data:
            self.data["auto_start"] = bool(self.data.get("auto_start_after_5s"))
        # Camera migration: collapse legacy source/index fields into camera_device.
        if "camera_device" not in self.data:
            legacy = str(self.data.get("camera_source", self.data.get("camera_mode", "OFF"))).upper()
            if legacy in {"", "OFF", "NONE"}:
                self.data["camera_device"] = "OFF"
            elif legacy == "INTERNAL":
                self.data["camera_device"] = "PI_CAMERA"
            elif legacy.startswith("USB"):
                idx = int(self.data.get("camera_index_usb1", self.data.get("camera_index_usb", 1)))
                self.data["camera_device"] = f"/dev/video{max(0, idx - 1)}"
            else:
                self.data["camera_device"] = legacy
        if "camera_backend" not in self.data:
            self.data["camera_backend"] = "auto"
        if "lidar_port" not in self.data and "lidar_port_hint" in self.data:
            self.data["lidar_port"] = self.data.get("lidar_port_hint", "")
        # v48: force Pixhawk to UART defaults (no USB / autodetect).
        self.data["mavlink_port"] = PIXHAWK_DEFAULT_PORT
        self.data["mavlink_baud"] = DEFAULTS["mavlink_baud"]
        if str(self.data.get("lidar_port", "")).strip() in {"/dev/ttyUSB0", "/dev/ttyAMA0", ""}:
            self.data["lidar_port"] = LIDAR_DEFAULT_PORT
        if not str(self.data.get("lidar_port_hint", "")).strip():
            self.data["lidar_port_hint"] = self.data.get("lidar_port", LIDAR_DEFAULT_PORT)
        if "show_camera" not in self.data:
            self.data["show_camera"] = False
        if "start_fullscreen" not in self.data:
            self.data["start_fullscreen"] = is_raspberry_pi()
        if "reverse_duration_sec" not in self.data:
            self.data["reverse_duration_sec"] = DEFAULTS["reverse_duration_sec"]
        if "post_turn_lock_duration_sec" not in self.data:
            self.data["post_turn_lock_duration_sec"] = DEFAULTS["post_turn_lock_duration_sec"]
        if "command_cooldown_sec" not in self.data:
            self.data["command_cooldown_sec"] = DEFAULTS["command_cooldown_sec"]

    # Convenience properties
    @property
    def mavlink_port(self) -> str:
        return str(self.get("mavlink_port"))

    @property
    def lidar_port(self) -> str:
        return str(self.get("lidar_port", ""))

    @property
    def mavlink_baud(self) -> int:
        return int(self.get("mavlink_baud"))

    @property
    def base_speed_pwm(self) -> int:
        return int(self.get("base_speed_pwm"))

    @property
    def limit1_cm(self) -> int:
        return int(self.get("limit1_cm"))

    @property
    def limit2_cm(self) -> int:
        return int(self.get("limit2_cm"))

    @property
    def limit3_cm(self) -> int:
        return int(self.get("limit3_cm"))

    @property
    def speed_limit1_pct(self) -> int:
        return int(self.get("speed_limit1_pct"))

    @property
    def speed_limit2_pct(self) -> int:
        return int(self.get("speed_limit2_pct"))

    @property
    def speed_limit3_pct(self) -> int:
        return int(self.get("speed_limit3_pct"))

    @property
    def show_lidar(self) -> bool:
        return bool(self.get("show_lidar", True))

    @property
    def camera_device(self) -> str:
        value = self.get("camera_device", "OFF")
        return str(value)

    @property
    def camera_backend(self) -> str:
        return str(self.get("camera_backend", "auto"))

    @property
    def auto_start(self) -> bool:
        return bool(self.get("auto_start", False))

    @property
    def show_camera(self) -> bool:
        return bool(self.get("show_camera", False))

    @property
    def start_fullscreen(self) -> bool:
        return bool(self.get("start_fullscreen", is_raspberry_pi()))

    @property
    def blue_boundary_cm(self) -> int:
        return int(self.get("blue_boundary_cm", config.DEFAULT_BLUE_BOUNDARY_CM))

    @property
    def green_boundary_cm(self) -> int:
        return int(self.get("green_boundary_cm", config.DEFAULT_GREEN_BOUNDARY_CM))

    @property
    def green_limit_cm(self) -> int:
        return int(self.get("green_limit_cm", config.GREEN_MAX_CM))

    @property
    def lidar_buffer_size(self) -> int:
        return int(self.get("lidar_buffer_size", config.DEFAULT_LIDAR_BUFFER_SIZE))

    @property
    def lidar_read_hz(self) -> float:
        try:
            return float(self.get("lidar_read_hz", config.DEFAULT_LIDAR_READ_HZ))
        except Exception:
            return float(config.DEFAULT_LIDAR_READ_HZ)

    @property
    def avoidance_turn_deg(self) -> int:
        try:
            return int(self.get("avoidance_turn_deg", 90))
        except Exception:
            return 90

    @property
    def avoidance_distance_m(self) -> float:
        try:
            return float(self.get("avoidance_distance_m", 2.0))
        except Exception:
            return 2.0

    @property
    def avoidance_backoff_m(self) -> float:
        try:
            return float(self.get("avoidance_backoff_m", 1.0))
        except Exception:
            return 1.0

    @property
    def reverse_duration_sec(self) -> float:
        try:
            return float(self.get("reverse_duration_sec", DEFAULTS["reverse_duration_sec"]))
        except Exception:
            return float(DEFAULTS["reverse_duration_sec"])

    @property
    def post_turn_lock_duration_sec(self) -> float:
        try:
            return float(self.get("post_turn_lock_duration_sec", DEFAULTS["post_turn_lock_duration_sec"]))
        except Exception:
            return float(DEFAULTS["post_turn_lock_duration_sec"])

    @property
    def command_cooldown_sec(self) -> float:
        try:
            return float(self.get("command_cooldown_sec", DEFAULTS["command_cooldown_sec"]))
        except Exception:
            return float(DEFAULTS["command_cooldown_sec"])

    @property
    def movement_coefficient(self) -> float:
        """Movement coefficient in m/s for converting distance to time when GPS unavailable."""
        try:
            return float(self.get("movement_coefficient", DEFAULTS["movement_coefficient"]))
        except Exception:
            return float(DEFAULTS["movement_coefficient"])

    # Legacy compatibility
    @property
    def camera_source(self) -> str:
        """Kept for backward compatibility with older modules."""
        return self.camera_device
