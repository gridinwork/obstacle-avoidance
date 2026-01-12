"""MAVLink communication wrapper with auto-reconnect and RC helpers."""

from __future__ import annotations

import threading
import time
import math
from typing import Callable, Optional

from settings import Settings
from shared_data import SharedData

try:
    from pymavlink import mavutil
except Exception:  # pragma: no cover - dependency may be missing on dev host
    mavutil = None


LogFn = Callable[[str], None]

# Modes that enable avoidance logic.
AVOIDANCE_MODES = {"AUTO", "MISSION", "LOITER", "GUIDED"}


class MavlinkComm:
    def __init__(self, shared: SharedData, settings: Settings, log_fn: LogFn = print) -> None:
        self.shared = shared
        self.settings = settings
        self.log = log_fn
        self.master: Optional[mavutil.mavlink_connection] = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    # Public API -----------------------------------------------------------------
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._connection_loop, daemon=True)
        self._thread.start()
        self.shared.update_status(mav_status="connecting", mav_connected=False)
        self.log("MAVLink listener thread started")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2)
        self.release_rc_override("stop")
        self._close_master()
        self.shared.update_status(mav_connected=False, mav_status="not_connected")

    def disconnect(self) -> None:
        """Hard stop the MAVLink loop and release the serial handle."""
        self.stop()
        self.shared.update_status(
            mav_connected=False,
            mav_status="disconnected",
            mission_active=False,
            avoidance_enabled=False,
            status_message="MAVLink disconnected",
        )

    def reconnect(self) -> None:
        """Force reconnection using latest settings."""
        self.stop()
        self.start()

    def arm(self) -> None:
        if not self.master:
            return
        try:
            self.master.arducopter_arm()
            self._send_rc_override(ch5=1900)  # ARM channel high
            self.log("ARM command sent")
        except Exception as exc:
            self.log(f"ARM failed: {exc}")

    def disarm(self) -> None:
        """
        Soft stop without DISARM: hold both throttle channels at ~1% (no movement).
        Avoids sending any arming-state change so subsequent commands remain valid.
        """
        if not self.master:
            return
        try:
            low_pwm = 1010  # ~1% throttle; should be stationary
            self._send_rc_override(ch3=low_pwm, ch6=low_pwm)
            self.shared.update_status(
                movement_state="STOPPED",
                status_message="Soft stop (DISARM suppressed, 1% throttle hold)",
            )
            self.log("Soft stop issued: throttle channels set to 1%; DISARM not sent")
        except Exception as exc:
            self.log(f"Soft stop failed: {exc}")

    def set_mode(self, mode: str | None) -> None:
        """Request a flight mode change if the target exists on this vehicle."""
        if not self.master:
            return
        target = self._normalize_mode_name(mode)
        if not target:
            self.log(f"Ignoring invalid mode request: {mode}")
            return
        try:
            mapping = {k.upper(): v for k, v in (self.master.mode_mapping() or {}).items()}
            mode_id = mapping.get(target)
            if mode_id is None:
                self.log(f"Mode {target} not available")
                return
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.log(f"Mode change requested -> {target}")
        except Exception as exc:
            self.log(f"Mode change failed: {exc}")

    def start_mission(self) -> None:
        """Arm, set AUTO mode. Motion is now controlled via MAVLink commands."""
        if not self.master:
            self.log("No MAVLink connection; cannot start mission")
            return
        self.arm()
        self.set_mode("AUTO")
        # Only set aux channels (ch5, ch8) if needed for arming/aux functions
        # Motion channels (ch1-ch4) are no longer controlled via RC override
        self._send_rc_override(ch5=2000, ch8=2000)
        self.release_rc_override("mission handoff")
        self.shared.update_status(mission_active=True, avoidance_enabled=True, status_message="Mission started")
        self.log("Mission start sequence triggered; avoidance activated")

    def ensure_guided(self) -> None:
        """Switch vehicle to GUIDED mode for temporary control."""
        self.set_mode("GUIDED")

    def return_to_auto(self) -> None:
        """
        Return control to the preferred/initial mode (fallback: AUTO).
        """
        target = self.preferred_mode(
            self.shared.user_return_mode,
            self.shared.pre_avoidance_mode,
            self.shared.initial_mode,
            "AUTO",
        )
        self._command_set_mode(target)
        self.release_rc_override("post-avoidance handoff")
        if self.shared.pre_avoidance_mode:
            self.shared.update_status(pre_avoidance_mode=None)

    def yaw_relative(self, delta_deg: float, rate_dps: float = 30.0) -> bool:
        """Rotate rover using MAV_CMD_CONDITION_YAW."""
        if mavutil is None or not self.master:
            self.log("Yaw command skipped: MAVLink not connected")
            return False
        direction = 1 if delta_deg >= 0 else -1
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                abs(delta_deg),
                abs(rate_dps),
                direction,
                1,  # relative
                0,
                0,
                0,
            )
            self.log(f"Yaw CONDITION_YAW {delta_deg} deg @ {rate_dps} dps")
            return True
        except Exception as exc:
            self.log(f"Yaw command failed: {exc}")
            return False

    def move_body(self, distance_m: float, speed_mps: float = 0.6) -> bool:
        """
        Move forward (positive distance) or backward (negative distance) in body frame using velocity setpoints.
        """
        if mavutil is None or not self.master:
            self.log("Move command skipped: MAVLink not connected")
            return False
        speed = max(0.05, speed_mps)
        duration = abs(distance_m) / speed
        direction = 1.0 if distance_m >= 0 else -1.0
        end_time = time.time() + duration
        try:
            while time.time() < end_time and not self.shared.stopped():
                self._send_velocity_target(vx=direction * speed, vy=0.0, vz=0.0)
                time.sleep(0.2)
            # send zero velocity to brake/hold
            self._send_velocity_target(vx=0.0, vy=0.0, vz=0.0)
            return True
        except Exception as exc:
            self.log(f"Move command failed: {exc}")
            return False

    def move_forward(self, distance_m: float, speed_mps: float = 0.6) -> bool:
        return self.move_body(distance_m=distance_m, speed_mps=speed_mps)

    def move_backward(self, distance_m: float, speed_mps: float = 0.6) -> bool:
        return self.move_body(distance_m=-abs(distance_m), speed_mps=speed_mps)

    def set_manual_rc(
        self,
        ch1: int | None = None,
        ch3: int | None = None,
        ch5: int | None = None,
        ch6: int | None = None,
        ch8: int | None = None,
    ) -> None:
        """
        Update manual RC channels and send override.
        
        WARNING: Motion channels (ch1, ch3, ch6) should NOT be used for movement control.
        Use MAVLink motion commands via mavlink_motion_commands module instead.
        Only aux channels (ch5, ch8) may be needed for arming/aux functions.
        """
        # Only allow aux channels (ch5, ch8) for RC override
        # Motion channels (ch1, ch3, ch6) are deprecated for movement control
        if ch1 is not None or ch3 is not None or ch6 is not None:
            self.log("WARNING: set_manual_rc called with motion channels (ch1/ch3/ch6). Use MAVLink motion commands instead.")
        
        with self.shared.state_lock:
            if ch1 is not None:
                self.shared.manual_channels["ch1"] = ch1
            if ch3 is not None:
                self.shared.manual_channels["ch3"] = ch3
            if ch5 is not None:
                self.shared.manual_channels["ch5"] = ch5
            if ch6 is not None:
                self.shared.manual_channels["ch6"] = ch6
            if ch8 is not None:
                self.shared.manual_channels["ch8"] = ch8
        # Only send RC override for aux channels, not motion channels
        self._send_rc_override(ch5=ch5, ch8=ch8)

    def set_speed_limit_percent(self, percent: float) -> None:
        """
        DEPRECATED: Set throttle PWM based on percent (0-100) to limit speed.
        
        This function is deprecated. Motion control should now use MAVLink commands
        via mavlink_motion_commands module instead of RC override.
        """
        # No longer using RC override for motion channels
        self.log("set_speed_limit_percent: deprecated, use MAVLink motion commands instead")

    def nudge_heading(self, steer_pwm: int, throttle_pwm: int) -> None:
        """
        DEPRECATED: Helper for avoidance module.
        
        This function is deprecated. Motion control should now use MAVLink commands
        via mavlink_motion_commands module instead of RC override.
        """
        # No longer using RC override for motion channels
        self.log("nudge_heading: deprecated, use MAVLink motion commands instead")

    def release_rc_override(self, reason: str | None = None) -> None:
        """Release RC override so the transmitter fully regains control (send twice)."""
        if not self.master:
            return
        payload = [0, 0, 0, 0, 0, 0, 0, 0]
        try:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *payload,
            )
            time.sleep(0.2)
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *payload,
            )
            msg = "RC override released"
            if reason:
                msg = f"{msg} ({reason})"
            self.log(msg)
        except Exception as exc:
            self.log(f"RC override release failed: {exc}")

    def _command_set_mode(self, mode: str | None) -> None:
        """Send MAV_CMD_DO_SET_MODE with fallback to set_mode."""
        target = self._normalize_mode_name(mode)
        if not target or not self.master:
            return
        try:
            mapping = {k.upper(): v for k, v in (self.master.mode_mapping() or {}).items()}
            mode_id = mapping.get(target)
            if mode_id is None:
                self.log(f"Mode {target} not available")
                return
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
                0,
                0,
                0,
                0,
                0,
            )
            self.log(f"DO_SET_MODE -> {target}")
        except Exception as exc:
            self.log(f"DO_SET_MODE failed: {exc}")
            self.set_mode(target)

    # Internal loops -------------------------------------------------------------
    def preferred_mode(self, *candidates: str | None) -> str:
        """
        Pick the first candidate that exists in the vehicle mode map.
        Fallback order: provided candidates, then AUTO, then any available mode.
        """
        mapping = {}
        try:
            mapping = {k.upper(): v for k, v in (self.master.mode_mapping() if self.master else {}).items()}
        except Exception:
            mapping = {}
        def _first_available(names: list[str]) -> str | None:
            for name in names:
                norm = self._normalize_mode_name(name)
                if norm and norm in mapping:
                    return norm
            return None
        preferred = _first_available([c for c in candidates if c is not None])
        if preferred:
            return preferred
        if "AUTO" in mapping:
            return "AUTO"
        if mapping:
            return next(iter(mapping.keys()))
        return "AUTO"

    def _normalize_mode_name(self, mode: str | None) -> str | None:
        if mode is None:
            return None
        name = str(mode).strip().upper()
        if not name or name == "UNKNOWN":
            return None
        if name.startswith("MODE("):
            return None
        return name

    def _heartbeat_mode_name(self, msg) -> str | None:
        """Translate a HEARTBEAT into a clean mode name using mode_mapping when possible."""
        try:
            mapping = {k.upper(): v for k, v in (self.master.mode_mapping() if self.master else {}).items()}
            reverse = {v: k for k, v in mapping.items()}
            custom_mode = getattr(msg, "custom_mode", None)
            if custom_mode is not None and custom_mode in reverse:
                return self._normalize_mode_name(reverse[custom_mode])
        except Exception:
            pass
        try:
            raw = mavutil.mode_string_v10(msg) if mavutil else ""
        except Exception:
            raw = ""
        return self._normalize_mode_name(raw)

    def _connection_loop(self) -> None:
        while not self._stop_event.is_set() and self.shared.running:
            if mavutil is None:
                self.shared.update_status(
                    mav_connected=False,
                    mav_status="error",
                    mav_error="pymavlink missing",
                    status_message="pymavlink not installed",
                )
                time.sleep(1.0)
                continue

            try:
                ports_to_try = ["/dev/serial0", "/dev/ttyAMA0"]
                baud = int(getattr(self.settings, "mavlink_baud", 57600) or 57600)
                connected = False
                last_exc: Exception | None = None
                for port in ports_to_try:
                    try:
                        self.log(f"Connecting MAVLink on {port} @ {baud}")
                        master = mavutil.mavlink_connection(port, baud=baud)
                        master.wait_heartbeat(timeout=5)
                        self.master = master
                        self.settings.set("mavlink_port", port)
                        self.shared.update_status(mav_connected=True, mav_status="connected", mav_error="")
                        self.log(f"MAVLink connected on {port}")
                        connected = True
                        break
                    except Exception as exc_inner:
                        last_exc = exc_inner
                        self.log(f"MAVLink attempt failed on {port}: {exc_inner}")
                        time.sleep(0.5)

                if not connected:
                    raise last_exc or RuntimeError("Pixhawk not detected on UART")

                self._listen_loop()
            except PermissionError as exc:
                msg = (
                    "Pixhawk not detected on UART (/dev/serial0 or /dev/ttyAMA0): "
                    f"{exc} — ensure UART login shell is disabled and user is in 'dialout'."
                )
                self.shared.update_status(
                    mav_connected=False,
                    mission_active=False,
                    status_message=msg,
                    mav_status="error",
                    mav_error=str(exc),
                )
                self.log(msg)
                time.sleep(2.0)
            except Exception as exc:
                msg = (
                    "Pixhawk not detected on UART (/dev/serial0 or /dev/ttyAMA0): "
                    f"{exc}"
                )
                self.shared.update_status(
                    mav_connected=False,
                    mission_active=False,
                    mav_status="error",
                    mav_error=str(exc),
                    status_message=msg,
                )
                self.log(msg)
                time.sleep(1.5)

    def _listen_loop(self) -> None:
        """Process incoming MAVLink messages and detect link loss."""
        assert self.master is not None
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,
            1,
        )
        while not self._stop_event.is_set() and self.shared.running:
            try:
                msg = self.master.recv_match(blocking=False)
            except Exception:
                msg = None
            if msg is None:
                time.sleep(0.05)
                continue
            mtype = msg.get_type()
            if mtype == "BAD_DATA":
                continue
            if mtype == "HEARTBEAT":
                mode = self._heartbeat_mode_name(msg) or "UNKNOWN"
                if not self.shared.initial_mode or self.shared.initial_mode == "UNKNOWN":
                    self.shared.update_status(initial_mode=mode)
                mission_active = mode in AVOIDANCE_MODES
                self.shared.update_status(
                    current_mode=mode,
                    mission_active=mission_active,
                    avoidance_enabled=mission_active and self.shared.obstacle_avoidance_on,
                    mav_status="connected",
                    mav_error="",
                )
            elif mtype == "ATTITUDE":
                yaw_rad = getattr(msg, "yaw", 0.0)
                heading_deg = (yaw_rad * 180 / 3.141592653589793) % 360
                self.shared.update_status(current_heading_deg=heading_deg)
            elif mtype == "RC_CHANNELS":
                ch3 = getattr(msg, "chan3_raw", None)
                if ch3 is not None:
                    moving = "MOVING" if ch3 > 1520 else "STOPPED"
                    self.shared.update_status(movement_state=moving)

            # Check link health
            if not self.master.target_system:
                self.shared.update_status(mav_connected=False)
                break

    def _resolve_port(self) -> str:
        """Deprecated in v48: Pixhawk uses fixed UART /dev/serial0."""
        return "/dev/serial0"

    def _send_attitude_target(self, yaw_rad: float) -> None:
        """Send SET_ATTITUDE_TARGET with yaw-only quaternion."""
        if mavutil is None or not self.master:
            return
        q = mavutil.mavlink.quaternion_from_euler(0.0, 0.0, yaw_rad)
        mask = (
            getattr(mavutil.mavlink, "ATTITUDE_TARGET_TYPEMASK_ROLL_RATE_IGNORE", 0)
            | getattr(mavutil.mavlink, "ATTITUDE_TARGET_TYPEMASK_PITCH_RATE_IGNORE", 0)
            | getattr(mavutil.mavlink, "ATTITUDE_TARGET_TYPEMASK_YAW_RATE_IGNORE", 0)
            | getattr(mavutil.mavlink, "ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE", 0)
            | getattr(mavutil.mavlink, "ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE", 0)
            | getattr(mavutil.mavlink, "ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE", 0)
            | getattr(mavutil.mavlink, "ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE", 0)
        )
        self.master.mav.set_attitude_target_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            self.master.target_system,
            self.master.target_component,
            mask,
            q,
            0.0,
            0.0,
            0.0,
            0.5,
        )

    def _send_position_target_local_ned(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        yaw: float | None = None,
        yaw_rate: float | None = None,
        mask: int | None = None,
        frame: int | None = None,
    ) -> None:
        if mavutil is None or not self.master:
            return
        default_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        )
        if yaw is None:
            default_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        if yaw_rate is None:
            default_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        self.master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            self.master.target_system,
            self.master.target_component,
            frame or mavutil.mavlink.MAV_FRAME_BODY_NED,
            default_mask if mask is None else mask,
            0.0,
            0.0,
            0.0,
            vx,
            vy,
            vz,
            0.0,
            0.0,
            0.0,
            0.0 if yaw is None else yaw,
            0.0 if yaw_rate is None else yaw_rate,
        )

    def _send_velocity_target(self, vx: float, vy: float, vz: float) -> None:
        """Velocity-only SET_POSITION_TARGET_LOCAL_NED in body frame."""
        if mavutil is None or not self.master:
            return
        mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self._send_position_target_local_ned(
            vx=vx,
            vy=vy,
            vz=vz,
            mask=mask,
            frame=mavutil.mavlink.MAV_FRAME_BODY_NED,
        )

    def _send_yaw_setpoint(self, yaw_rad: float, yaw_rate: float | None = None) -> None:
        """Yaw-only SET_POSITION_TARGET_LOCAL_NED to align with heading."""
        if mavutil is None or not self.master:
            return
        mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        )
        if yaw_rate is None:
            mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        self._send_position_target_local_ned(
            yaw=yaw_rad,
            yaw_rate=0.0 if yaw_rate is None else yaw_rate,
            mask=mask,
            frame=mavutil.mavlink.MAV_FRAME_BODY_NED,
        )

    def _send_rc_override(
        self,
        ch1: int | None = None,
        ch3: int | None = None,
        ch5: int | None = None,
        ch6: int | None = None,
        ch8: int | None = None,
    ) -> None:
        if not self.master:
            return
        try:
            current = self.shared.manual_channels.copy()
            payload = [
                ch1 if ch1 is not None else current["ch1"],
                0,
                ch3 if ch3 is not None else current["ch3"],
                0,
                ch5 if ch5 is not None else current["ch5"],
                ch6 if ch6 is not None else current["ch6"],
                ch8 if ch8 is not None else current.get("ch8", 1000),
                0,
            ]
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *payload,
            )
        except Exception as exc:
            self.log(f"RC override failed: {exc}")

    def _close_master(self) -> None:
        try:
            if self.master:
                self.master.close()
        except Exception:
            pass
        self.master = None


