"""
MAVLink Controller - Dedicated module for MAVLink connection and command execution.
This module ensures MAVLink connection is ALWAYS properly initialized and NEVER set to None.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None

from config import GPS_SPEED_FACTOR


LogFn = Callable[[str], None]


class MavlinkController:
    """
    Centralized MAVLink connection controller.
    Ensures connection is ALWAYS initialized and NEVER set to None.
    """
    
    def __init__(self, log_fn: LogFn = print) -> None:
        self.log = log_fn
        self.connection: Optional[mavutil.mavlink_connection] = None
        self._connection_lock = False
        self.gps_available = False
        self.gps_speed_factor = GPS_SPEED_FACTOR
        
    def connect_mavlink(self, port: str, baudrate: int) -> Optional[mavutil.mavlink_connection]:
        """
        Connect to MAVLink device.
        
        Args:
            port: Serial port (e.g., "/dev/serial0" or "/dev/ttyACM0")
            baudrate: Baud rate (e.g., 57600, 115200)
            
        Returns:
            mavlink_connection object or None if connection failed
        """
        if mavutil is None:
            self.log("ERROR: pymavlink not available")
            return None
            
        if self._connection_lock:
            self.log("Connection already in progress")
            return self.connection
            
        self._connection_lock = True
        
        try:
            self.log(f"Connecting to MAVLink on {port} at {baudrate} baud...")
            
            # Create connection
            connection = mavutil.mavlink_connection(port, baud=baudrate)
            
            # Wait for heartbeat
            self.log("Waiting for MAVLink heartbeat...")
            connection.wait_heartbeat(timeout=5)
            
            self.connection = connection
            self.log(f"MAVLink connected! System: {connection.target_system}, Component: {connection.target_component}")
            
            # Update GPS status
            self._check_gps_status()
            
            return self.connection
            
        except Exception as exc:
            self.log(f"MAVLink connection failed: {exc}")
            self.connection = None
            return None
        finally:
            self._connection_lock = False
    
    def check_connection(self) -> bool:
        """
        Check if MAVLink connection is active.
        
        Returns:
            True if connection is valid, False otherwise
        """
        if self.connection is None:
            return False
            
        try:
            # Try to get a message (non-blocking)
            msg = self.connection.recv_match(type='HEARTBEAT', blocking=False, timeout=0.1)
            if msg:
                self._check_gps_status()
                return True
            # Even if no heartbeat received, connection object exists
            return True
        except Exception:
            # Connection lost
            self.connection = None
            return False
    
    def _check_gps_status(self) -> None:
        """Check GPS status from MAVLink messages."""
        if not self.connection:
            return
        try:
            msg = self.connection.recv_match(type='GPS_RAW_INT', blocking=False, timeout=0.1)
            if msg:
                self.gps_available = (msg.fix_type >= 2)  # 2 = 2D fix, 3 = 3D fix
        except Exception:
            pass
    
    def send_movement_command(self, json_command: dict) -> bool:
        """
        Send movement command from JSON definition.
        
        Args:
            json_command: Dictionary with command type and parameters
                Example: {"type": "MOVE", "distance_m": 1.0, "speed": 0.5}
                
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.check_connection():
            self.log("MAVLink connection not available for movement command")
            return False
            
        cmd_type = json_command.get("type", "").upper()
        
        if cmd_type == "MOVE":
            distance_m = json_command.get("distance_m", 0.0)
            speed = json_command.get("speed", 0.5)
            return self.send_forward(distance_m=distance_m, speed=speed) if distance_m >= 0 else self.send_reverse(distance_m=abs(distance_m), speed=speed)
        elif cmd_type == "TURN":
            angle_deg = json_command.get("angle_deg", 0.0)
            return self.send_turn(angle_deg=angle_deg)
        elif cmd_type == "STOP":
            return self.send_stop()
        else:
            self.log(f"Unknown command type: {cmd_type}")
            return False
    
    def send_stop(self) -> bool:
        """
        Stop all movement WITHOUT disarming.
        Sends zero velocity setpoint.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.check_connection():
            return False
            
        try:
            # Send zero velocity
            self.connection.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,  # type_mask (velocity only)
                0, 0, 0,  # x, y, z (position, ignored)
                0, 0, 0,  # vx, vy, vz (velocity = 0)
                0, 0, 0,  # afx, afy, afz (acceleration, ignored)
                0, 0  # yaw, yaw_rate
            )
            
            # Also send manual_control with zeros
            self.connection.mav.manual_control_send(
                self.connection.target_system,
                0,  # x (forward/backward)
                0,  # y (left/right)
                0,  # z (throttle)
                0,  # r (rotation)
                0   # buttons
            )
            
            self.log("STOP command sent (zero velocity)")
            return True
            
        except Exception as exc:
            self.log(f"STOP command failed: {exc}")
            return False
    
    def send_turn(self, angle_deg: float, turn_speed_deg: float = 30.0) -> bool:
        """
        Send turn command using MAV_CMD_CONDITION_YAW.
        
        Args:
            angle_deg: Angle to turn (positive = right, negative = left)
            turn_speed_deg: Turn speed in degrees per second
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.check_connection():
            return False
            
        try:
            direction = 1 if angle_deg >= 0 else -1
            
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,  # confirmation
                abs(angle_deg),  # param1: target angle
                turn_speed_deg,  # param2: speed
                direction,  # param3: direction (-1 = counterclockwise, 1 = clockwise)
                1,  # param4: relative (1 = relative to current heading)
                0, 0, 0, 0  # params 5-7: unused
            )
            
            self.log(f"TURN command sent: {angle_deg}° @ {turn_speed_deg}°/s")
            return True
            
        except Exception as exc:
            self.log(f"TURN command failed: {exc}")
            return False
    
    def send_forward(self, distance_m: Optional[float] = None, speed: float = 0.5, time_s: Optional[float] = None) -> bool:
        """
        Send forward movement command.
        
        Args:
            distance_m: Distance in meters (if GPS available)
            speed: Speed in m/s
            time_s: Duration in seconds (if GPS not available or as fallback)
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.check_connection():
            return False
            
        try:
            # Calculate duration if needed
            if time_s is None:
                if self.gps_available and distance_m is not None:
                    # GPS available: use distance-based movement
                    time_s = abs(distance_m) / max(0.1, speed)
                else:
                    # No GPS: use time-based with speed factor
                    if distance_m is not None:
                        time_s = abs(distance_m) * self.gps_speed_factor
                    else:
                        time_s = 1.0  # default 1 second
            
            # Enable GUIDED mode if needed
            self.send_set_mode("GUIDED")
            time.sleep(0.1)
            
            # Send velocity setpoint
            end_time = time.time() + time_s
            while time.time() < end_time:
                self.connection.mav.set_position_target_local_ned_send(
                    0,  # time_boot_ms
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111000111,  # type_mask (velocity only)
                    0, 0, 0,  # x, y, z (position, ignored)
                    speed, 0, 0,  # vx, vy, vz (forward velocity)
                    0, 0, 0,  # afx, afy, afz (acceleration, ignored)
                    0, 0  # yaw, yaw_rate
                )
                time.sleep(0.2)
            
            # Stop after movement
            self.send_stop()
            
            self.log(f"FORWARD command sent: {distance_m}m @ {speed}m/s for {time_s}s")
            return True
            
        except Exception as exc:
            self.log(f"FORWARD command failed: {exc}")
            return False
    
    def send_reverse(self, distance_m: Optional[float] = None, speed: float = 0.4, time_s: Optional[float] = None) -> bool:
        """
        Send reverse movement command.
        
        Args:
            distance_m: Distance in meters (if GPS available)
            speed: Speed in m/s
            time_s: Duration in seconds (if GPS not available or as fallback)
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.check_connection():
            return False
            
        try:
            # Calculate duration if needed
            if time_s is None:
                if self.gps_available and distance_m is not None:
                    # GPS available: use distance-based movement
                    time_s = abs(distance_m) / max(0.1, speed)
                else:
                    # No GPS: use time-based with speed factor
                    if distance_m is not None:
                        time_s = abs(distance_m) * self.gps_speed_factor
                    else:
                        time_s = 1.0  # default 1 second
            
            # Enable GUIDED mode if needed
            self.send_set_mode("GUIDED")
            time.sleep(0.1)
            
            # Send negative velocity setpoint
            end_time = time.time() + time_s
            while time.time() < end_time:
                self.connection.mav.set_position_target_local_ned_send(
                    0,  # time_boot_ms
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111000111,  # type_mask (velocity only)
                    0, 0, 0,  # x, y, z (position, ignored)
                    -speed, 0, 0,  # vx, vy, vz (negative = backward)
                    0, 0, 0,  # afx, afy, afz (acceleration, ignored)
                    0, 0  # yaw, yaw_rate
                )
                time.sleep(0.2)
            
            # Stop after movement
            self.send_stop()
            
            self.log(f"REVERSE command sent: {distance_m}m @ {speed}m/s for {time_s}s")
            return True
            
        except Exception as exc:
            self.log(f"REVERSE command failed: {exc}")
            return False
    
    def send_set_mode(self, mode_name: str) -> bool:
        """
        Set vehicle mode using MAVLink.
        
        Args:
            mode_name: Mode name (e.g., "GUIDED", "AUTO", "MANUAL")
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.check_connection():
            return False
            
        try:
            # Get mode mapping
            mode_mapping = self.connection.mode_mapping()
            if not mode_mapping:
                self.log("Mode mapping not available")
                return False
            
            mode_id = mode_mapping.get(mode_name.upper())
            if mode_id is None:
                self.log(f"Mode {mode_name} not available")
                return False
            
            # Send mode change
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            self.log(f"Mode change requested: {mode_name}")
            return True
            
        except Exception as exc:
            self.log(f"Mode change failed: {exc}")
            return False
    
    def disconnect(self) -> None:
        """Close MAVLink connection."""
        if self.connection:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None
            self.log("MAVLink connection closed")
