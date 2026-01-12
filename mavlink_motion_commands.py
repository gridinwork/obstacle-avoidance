"""
MAVLink motion command execution system.

This module loads command definitions from commands.json and executes
movement commands using proper MAVLink messages instead of RC override.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Callable, Dict, Optional

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None


class MavlinkMotionCommands:
    """Execute MAVLink motion commands based on JSON command definitions."""

    def __init__(
        self,
        mav_connection,
        commands_file: str | Path | None = None,
        movement_coefficient: float = 1.0,
        gps_available: bool = False,
        log_fn: Callable[[str], None] = print,
    ) -> None:
        """
        Initialize the motion command executor.

        Args:
            mav_connection: MAVLink connection object (mavutil.mavlink_connection)
            commands_file: Path to commands.json file (default: commands.json in same directory)
            movement_coefficient: Seconds per meter for no-GPS mode (default: 1.0)
            gps_available: Whether GPS fix is available
            log_fn: Logging function
        """
        self.mav = mav_connection
        self.movement_coefficient = movement_coefficient
        self.gps_available = gps_available
        self.log = log_fn

        # Load command definitions
        if commands_file is None:
            commands_file = Path(__file__).parent / "commands.json"
        self.commands = self._load_commands(commands_file)

    def _load_commands(self, file_path: str | Path) -> Dict[str, Any]:
        """Load command definitions from JSON file."""
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as exc:
            self.log(f"Failed to load commands.json: {exc}")
            return {}

    def execute(self, command_dict: Dict[str, Any]) -> bool:
        """
        Execute a command from a JSON-like dictionary.

        Args:
            command_dict: Dictionary with "command" and "params" keys
                Example: {"command": "move_forward", "params": {"speed": 0.7, "distance_m": 1.0}}

        Returns:
            True if command executed successfully, False otherwise
        """
        if not self.mav:
            self.log("MAVLink connection not available")
            return False

        command_name = command_dict.get("command")
        params = command_dict.get("params", {})

        if command_name not in self.commands:
            self.log(f"Unknown command: {command_name}")
            return False

        # Route to appropriate handler
        handlers = {
            "move_forward": self.move_forward,
            "move_backward": self.move_backward,
            "turn_left": self.turn_left,
            "turn_right": self.turn_right,
            "stop": self.stop_movement,
        }

        handler = handlers.get(command_name)
        if not handler:
            self.log(f"No handler for command: {command_name}")
            return False

        try:
            return handler(**params)
        except Exception as exc:
            self.log(f"Command execution failed: {exc}")
            return False

    def move_forward(
        self,
        speed: float,
        distance_m: Optional[float] = None,
        time_s: Optional[float] = None,
    ) -> bool:
        """
        Move the vehicle forward using MAVLink velocity commands.

        Args:
            speed: Forward velocity in m/s
            distance_m: Distance in meters (used if GPS available)
            time_s: Duration in seconds (used if no GPS or explicitly provided)

        Returns:
            True if command executed successfully
        """
        if mavutil is None or not self.mav:
            self.log("Move forward skipped: MAVLink not connected")
            return False

        try:
            # Determine movement duration
            if time_s is not None:
                duration = time_s
            elif distance_m is not None:
                if self.gps_available:
                    # GPS available: use distance directly
                    duration = abs(distance_m) / max(0.1, speed)
                else:
                    # No GPS: convert distance to time using coefficient
                    duration = abs(distance_m) * self.movement_coefficient
                    self.log(f"GPS unavailable: {distance_m}m -> {duration:.2f}s (coeff={self.movement_coefficient})")
            else:
                # Default: 1 second
                duration = 1.0

            # Ensure GUIDED mode
            self._ensure_guided_mode()

            # Execute velocity-based forward movement
            end_time = time.time() + duration
            speed = max(0.05, min(speed, 5.0))  # Clamp speed

            while time.time() < end_time:
                self._send_velocity_target(vx=speed, vy=0.0, vz=0.0)
                time.sleep(0.2)  # 5Hz command rate

            # Stop by sending zero velocity
            self._send_velocity_target(vx=0.0, vy=0.0, vz=0.0)
            self.log(f"Move forward completed: {duration:.2f}s @ {speed} m/s")
            return True

        except Exception as exc:
            self.log(f"Move forward failed: {exc}")
            return False

    def move_backward(
        self,
        speed: float,
        distance_m: Optional[float] = None,
        time_s: Optional[float] = None,
    ) -> bool:
        """
        Move the vehicle backward using MAVLink velocity commands.

        Args:
            speed: Backward velocity (positive number, direction is negative)
            distance_m: Distance in meters
            time_s: Duration in seconds

        Returns:
            True if command executed successfully
        """
        if mavutil is None or not self.mav:
            self.log("Move backward skipped: MAVLink not connected")
            return False

        try:
            # Determine movement duration
            if time_s is not None:
                duration = time_s
            elif distance_m is not None:
                if self.gps_available:
                    duration = abs(distance_m) / max(0.1, speed)
                else:
                    duration = abs(distance_m) * self.movement_coefficient
                    self.log(f"GPS unavailable: {distance_m}m -> {duration:.2f}s (coeff={self.movement_coefficient})")
            else:
                duration = 1.0

            # Ensure GUIDED mode
            self._ensure_guided_mode()

            # Execute velocity-based backward movement (negative vx)
            end_time = time.time() + duration
            speed = max(0.05, min(speed, 5.0))  # Clamp speed

            while time.time() < end_time:
                self._send_velocity_target(vx=-speed, vy=0.0, vz=0.0)
                time.sleep(0.2)  # 5Hz command rate

            # Stop by sending zero velocity
            self._send_velocity_target(vx=0.0, vy=0.0, vz=0.0)
            self.log(f"Move backward completed: {duration:.2f}s @ {speed} m/s")
            return True

        except Exception as exc:
            self.log(f"Move backward failed: {exc}")
            return False

    def turn_left(
        self,
        angle_deg: float,
        turn_speed_deg: float = 30.0,
    ) -> bool:
        """
        Rotate left (counter-clockwise) around yaw axis.

        Args:
            angle_deg: Angle to rotate in degrees
            turn_speed_deg: Yaw rotation speed in degrees per second

        Returns:
            True if command executed successfully
        """
        if mavutil is None or not self.mav:
            self.log("Turn left skipped: MAVLink not connected")
            return False

        try:
            self._ensure_guided_mode()

            # MAV_CMD_CONDITION_YAW: direction 1 = left (counter-clockwise)
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,  # confirmation
                abs(angle_deg),  # param1: target angle
                abs(turn_speed_deg),  # param2: angular speed
                1,  # param3: direction (1 = CCW/left, -1 = CW/right)
                1,  # param4: relative (1 = relative to current heading)
                0,  # param5-7: unused
                0,
                0,
            )

            self.log(f"Turn left {abs(angle_deg)}° @ {turn_speed_deg} dps (relative)")
            return True

        except Exception as exc:
            self.log(f"Turn left failed: {exc}")
            return False

    def turn_right(
        self,
        angle_deg: float,
        turn_speed_deg: float = 30.0,
    ) -> bool:
        """
        Rotate right (clockwise) around yaw axis.

        Args:
            angle_deg: Angle to rotate in degrees
            turn_speed_deg: Yaw rotation speed in degrees per second

        Returns:
            True if command executed successfully
        """
        if mavutil is None or not self.mav:
            self.log("Turn right skipped: MAVLink not connected")
            return False

        try:
            self._ensure_guided_mode()

            # MAV_CMD_CONDITION_YAW: direction -1 = right (clockwise)
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,  # confirmation
                abs(angle_deg),  # param1: target angle
                abs(turn_speed_deg),  # param2: angular speed
                -1,  # param3: direction (1 = CCW/left, -1 = CW/right)
                1,  # param4: relative (1 = relative to current heading)
                0,  # param5-7: unused
                0,
                0,
            )

            self.log(f"Turn right {abs(angle_deg)}° @ {turn_speed_deg} dps (relative)")
            return True

        except Exception as exc:
            self.log(f"Turn right failed: {exc}")
            return False

    def stop_movement(self) -> bool:
        """
        Stop all motion WITHOUT disarming motors.

        Sends zero velocity target using SET_POSITION_TARGET_LOCAL_NED.
        Does NOT send DISARM or override RC channels.

        Returns:
            True if command executed successfully
        """
        if mavutil is None or not self.mav:
            self.log("Stop movement skipped: MAVLink not connected")
            return False

        try:
            # Send zero velocity to stop movement
            self._send_velocity_target(vx=0.0, vy=0.0, vz=0.0)
            self.log("Stop movement: zero velocity sent (no disarm)")
            return True

        except Exception as exc:
            self.log(f"Stop movement failed: {exc}")
            return False

    # Internal helper methods ----------------------------------------------------

    def _ensure_guided_mode(self) -> None:
        """Ensure vehicle is in GUIDED mode for command-based control."""
        try:
            mapping = {k.upper(): v for k, v in (self.mav.mode_mapping() or {}).items()}
            if "GUIDED" not in mapping:
                self.log("GUIDED mode not available")
                return

            mode_id = mapping.get("GUIDED")
            if mode_id is not None:
                self.mav.mav.set_mode_send(
                    self.mav.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id,
                )
        except Exception:
            pass  # Mode may already be correct

    def _send_velocity_target(self, vx: float, vy: float, vz: float) -> None:
        """
        Send velocity target using SET_POSITION_TARGET_LOCAL_NED.

        Uses type_mask 0b0000111111000111 to enable ONLY velocity control.
        Frame: MAV_FRAME_LOCAL_NED
        """
        if mavutil is None or not self.mav:
            return

        # Type mask: 0b0000111111000111
        # This enables ONLY velocity control (vx, vy, vz)
        # All position, acceleration, and yaw components are ignored
        type_mask = 0b0000111111000111

        self.mav.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            type_mask,  # type_mask
            0.0,  # x (ignored)
            0.0,  # y (ignored)
            0.0,  # z (ignored)
            vx,  # vx: forward velocity
            vy,  # vy: rightward velocity
            vz,  # vz: upward velocity
            0.0,  # afx (ignored)
            0.0,  # afy (ignored)
            0.0,  # afz (ignored)
            0.0,  # yaw (ignored)
            0.0,  # yaw_rate (ignored)
        )


# Convenience function for direct execution
def execute(
    mav_connection,
    command_dict: Dict[str, Any],
    movement_coefficient: float = 1.0,
    gps_available: bool = False,
    log_fn: Callable[[str], None] = print,
) -> bool:
    """
    Convenience function to execute a command directly.

    Args:
        mav_connection: MAVLink connection object
        command_dict: Command dictionary with "command" and "params"
        movement_coefficient: Seconds per meter for no-GPS mode
        gps_available: Whether GPS is available
        log_fn: Logging function

    Returns:
        True if command executed successfully
    """
    executor = MavlinkMotionCommands(
        mav_connection,
        movement_coefficient=movement_coefficient,
        gps_available=gps_available,
        log_fn=log_fn,
    )
    return executor.execute(command_dict)
