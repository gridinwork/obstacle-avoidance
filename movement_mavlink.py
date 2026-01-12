"""
MAVLink Movement Controller for Differential Tracked Robot.
Controls movement via RC channel overrides for relay switching and motor control.

Version 76: Clean RC-override-only model, no GPS/compass/angle-based logic.

Relay Mapping:
- RC5 (ENABLE):  1100 = ON, 1900 = OFF
- RC6 (REVERSE): 1100 = FORWARD, 2000 = BACKWARD
- RC7 (ARMING):  1900 = ARM, 1100 = DISARM

Motor Control:
- CH1 (STEERING): 1000 = LEFT, 1500 = CENTER, 2000 = RIGHT
- CH3 (THROTTLE): 1000-2000, formula: 1000 + speed_percent × 10
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None

try:
    from config import ROTATION_TIME_SEC, FORWARD_TIME_SEC, SPEED_CAP_PERCENT
except ImportError:
    # Fallback defaults if config not available
    ROTATION_TIME_SEC = 1.5
    FORWARD_TIME_SEC = 0.8
    SPEED_CAP_PERCENT = 25.0


LogFn = Callable[[str], None]


class MovementController:
    """
    Movement controller for differential tracked robot with relay control.
    Uses ONLY RC channel overrides - no MAVLink position/velocity commands.
    """
    
    # RC Channel mappings
    RC_ENABLE = 5   # RC5: 1100 = ON, 1900 = OFF
    RC_REVERSE = 6  # RC6: 1100 = FORWARD, 2000 = BACKWARD
    RC_ARMING = 7   # RC7: 1900 = ARM, 1100 = DISARM
    CH_STEERING = 1 # CH1: 1000 = LEFT, 1500 = CENTER, 2000 = RIGHT
    CH_THROTTLE = 3 # CH3: 1000-2000, formula: 1000 + speed_percent × 10
    
    # Relay values
    RELAY_ON = 1100
    RELAY_OFF = 1900
    ARM_HIGH = 1900
    ARM_LOW = 1100
    REVERSE_ON = 2000  # REVERSE ON = 2000 (not 1900)
    REVERSE_OFF = 1100  # REVERSE OFF (forward) = 1100
    
    # Steering values
    STEER_LEFT = 1000
    STEER_CENTER = 1500
    STEER_RIGHT = 2000
    
    # Throttle range
    THROTTLE_MIN = 1000
    THROTTLE_MAX = 2000
    
    def __init__(self, log_fn: LogFn = print) -> None:
        self.log = log_fn
        self.connection: Optional[mavutil.mavlink_connection] = None
        self._enable_thread: Optional[threading.Thread] = None
        self._enable_event = threading.Event()
        self._enable_active = False
        self._lock = threading.Lock()
        
        # Current state
        self._current_enable = self.RELAY_OFF
        self._current_reverse = self.REVERSE_OFF  # OFF = forward
        self._current_steering = self.STEER_CENTER
        self._current_throttle = self.THROTTLE_MIN
        self._is_armed = False
        
    def connect(self, port: str, baud: int) -> bool:
        """
        Connect to MAVLink device.
        
        Args:
            port: Serial port (e.g., "/dev/serial0" or "/dev/ttyACM0")
            baud: Baud rate (e.g., 57600, 115200)
            
        Returns:
            True if connection successful, False otherwise
        """
        if mavutil is None:
            self.log("ERROR: pymavlink not available")
            return False
            
        try:
            self.log(f"Connecting to MAVLink on {port} at {baud} baud...")
            connection = mavutil.mavlink_connection(port, baud=baud)
            connection.wait_heartbeat(timeout=5)
            self.connection = connection
            self.log(f"MAVLink connected! System: {connection.target_system}, Component: {connection.target_component}")
            return True
        except Exception as exc:
            self.log(f"MAVLink connection failed: {exc}")
            self.connection = None
            return False
    
    def disconnect(self) -> None:
        """Disconnect and stop all movement."""
        self.stop()
        if self.connection:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None
            self.log("MAVLink connection closed")
    
    def _send_rc_override(self, ch1: Optional[int] = None, ch3: Optional[int] = None,
                          ch5: Optional[int] = None, ch6: Optional[int] = None,
                          ch7: Optional[int] = None) -> None:
        """
        Send RC channels override message.
        
        Args:
            ch1: Steering channel (1000=LEFT, 1500=CENTER, 2000=RIGHT)
            ch3: Throttle channel (1000-2000)
            ch5: Enable relay (1100=ON, 1900=OFF)
            ch6: Reverse relay (1100=FORWARD, 2000=BACKWARD)
            ch7: Arming relay (1900=ARM, 1100=DISARM)
        """
        if not self.connection:
            return
            
        try:
            # Build channels array (1-indexed, 0 = no change)
            channels = [0] * 8
            
            if ch1 is not None:
                channels[0] = max(self.THROTTLE_MIN, min(2000, ch1))
                self._current_steering = channels[0]
            else:
                channels[0] = self._current_steering
                
            if ch3 is not None:
                channels[2] = max(self.THROTTLE_MIN, min(self.THROTTLE_MAX, ch3))
                self._current_throttle = channels[2]
            else:
                channels[2] = self._current_throttle
                
            if ch5 is not None:
                channels[4] = ch5
                self._current_enable = ch5
            else:
                channels[4] = self._current_enable
                
            if ch6 is not None:
                channels[5] = ch6
                self._current_reverse = ch6
            else:
                channels[5] = self._current_reverse
                
            if ch7 is not None:
                channels[6] = ch7
            else:
                channels[6] = self.ARM_HIGH if self._is_armed else self.ARM_LOW
            
            # Send RC override
            self.connection.mav.rc_channels_override_send(
                self.connection.target_system,
                self.connection.target_component,
                *channels
            )
        except Exception as exc:
            self.log(f"RC override send failed: {exc}")
    
    def _enable_loop(self) -> None:
        """Background thread to continuously send ENABLE signal every 200ms while moving."""
        while not self._enable_event.is_set():
            if self._enable_active:
                with self._lock:
                    if self.connection:
                        self._send_rc_override(ch5=self.RELAY_ON)
            time.sleep(0.2)  # 200ms interval
    
    def _start_enable_loop(self) -> None:
        """Start the ENABLE signal loop."""
        if self._enable_thread and self._enable_thread.is_alive():
            return
        self._enable_event.clear()
        self._enable_active = True
        self._enable_thread = threading.Thread(target=self._enable_loop, daemon=True)
        self._enable_thread.start()
        self.log("ENABLE loop started")
    
    def _stop_enable_loop(self) -> None:
        """Stop the ENABLE signal loop."""
        self._enable_active = False
        self._enable_event.set()
        if self._enable_thread and self._enable_thread.is_alive():
            self._enable_thread.join(timeout=1.0)
        self.log("ENABLE loop stopped")
    
    def arm(self) -> bool:
        """
        Arm the vehicle.
        
        Returns:
            True if command sent successfully
        """
        if not self.connection:
            self.log("Cannot arm: MAVLink not connected")
            return False
            
        try:
            self._is_armed = True
            self._send_rc_override(ch7=self.ARM_HIGH)
            self.log("ARM command sent (RC7 = 1900)")
            return True
        except Exception as exc:
            self.log(f"ARM failed: {exc}")
            return False
    
    def disarm(self) -> bool:
        """
        Disarm the vehicle.
        
        Returns:
            True if command sent successfully
        """
        if not self.connection:
            self.log("Cannot disarm: MAVLink not connected")
            return False
            
        try:
            self.stop()  # Stop movement first
            self._is_armed = False
            self._send_rc_override(ch7=self.ARM_LOW)
            self.log("DISARM command sent (RC7 = 1100)")
            return True
        except Exception as exc:
            self.log(f"DISARM failed: {exc}")
            return False
    
    def _calculate_throttle(self, speed_percent: float) -> int:
        """
        Calculate throttle PWM from speed percentage.
        Formula: PWM = 1000 + speed_percent × 10
        
        Args:
            speed_percent: Speed percentage (0-100)
            
        Returns:
            Throttle PWM value (1000-2000)
        """
        speed_percent = max(0.0, min(100.0, speed_percent))
        pwm = int(1000 + speed_percent * 10)
        return max(self.THROTTLE_MIN, min(self.THROTTLE_MAX, pwm))
    
    def stop(self) -> bool:
        """
        Stop all movement.
        Sets: RC3=1000 (throttle zero), RC1=1500 (center steering), RC5=1900 (ENABLE OFF)
        Stops the ENABLE loop thread.
        
        Returns:
            True if command sent successfully
        """
        if not self.connection:
            return False
            
        try:
            # Stop ENABLE loop first
            self._stop_enable_loop()
            # ENABLE: OFF (RC5 = 1900)
            self._current_enable = self.RELAY_OFF
            # THROTTLE: 1000 (min)
            # STEERING: CENTER (CH1 = 1500)
            self._send_rc_override(
                ch1=self.STEER_CENTER,
                ch3=self.THROTTLE_MIN,
                ch5=self.RELAY_OFF
            )
            self.log("STOP — ENABLE OFF")
            return True
        except Exception as exc:
            self.log(f"Stop failed: {exc}")
            return False
    
    def timed_turn_left(self, duration_seconds: float, speed_percent: float) -> bool:
        """
        Turn left for a specified duration.
        
        Continuously sends ENABLE ON (RC5=1100), sets REVERSE OFF (RC6=1100),
        sends TURN LEFT (RC1=1000), sets throttle according to speed_percent,
        keeps turning for duration_seconds, then STOPs.
        
        Args:
            duration_seconds: How long to turn (seconds)
            speed_percent: Throttle speed percentage (0-100)
            
        Returns:
            True if command sent successfully
        """
        if not self.connection:
            self.log("Cannot turn left: MAVLink not connected")
            return False
        
        try:
            self.log(f"Timed turn LEFT started ({duration_seconds} sec)")
            # ENABLE: ON (RC5 = 1100) - will be sent continuously by loop
            self._start_enable_loop()
            self._current_enable = self.RELAY_ON
            # REVERSE: OFF (RC6 = 1100)
            self._current_reverse = self.REVERSE_OFF
            # STEERING: LEFT (RC1 = 1000)
            # THROTTLE: calculated from speed_percent
            throttle_pwm = self._calculate_throttle(speed_percent)
            self._send_rc_override(
                ch1=self.STEER_LEFT,
                ch3=throttle_pwm,
                ch5=self.RELAY_ON,
                ch6=self.REVERSE_OFF
            )
            
            # Keep turning for duration_seconds
            time.sleep(duration_seconds)
            
            # Then STOP
            self.stop()
            self.log("Timed turn LEFT finished")
            return True
        except Exception as exc:
            self.log(f"Timed turn left failed: {exc}")
            self.stop()
            return False
    
    def timed_turn_right(self, duration_seconds: float, speed_percent: float) -> bool:
        """
        Turn right for a specified duration.
        
        Continuously sends ENABLE ON (RC5=1100), sets REVERSE OFF (RC6=1100),
        sends TURN RIGHT (RC1=2000), sets throttle according to speed_percent,
        keeps turning for duration_seconds, then STOPs.
        
        Args:
            duration_seconds: How long to turn (seconds)
            speed_percent: Throttle speed percentage (0-100)
            
        Returns:
            True if command sent successfully
        """
        if not self.connection:
            self.log("Cannot turn right: MAVLink not connected")
            return False
        
        try:
            self.log(f"Timed turn RIGHT started ({duration_seconds} sec)")
            # ENABLE: ON (RC5 = 1100) - will be sent continuously by loop
            self._start_enable_loop()
            self._current_enable = self.RELAY_ON
            # REVERSE: OFF (RC6 = 1100)
            self._current_reverse = self.REVERSE_OFF
            # STEERING: RIGHT (RC1 = 2000)
            # THROTTLE: calculated from speed_percent
            throttle_pwm = self._calculate_throttle(speed_percent)
            self._send_rc_override(
                ch1=self.STEER_RIGHT,
                ch3=throttle_pwm,
                ch5=self.RELAY_ON,
                ch6=self.REVERSE_OFF
            )
            
            # Keep turning for duration_seconds
            time.sleep(duration_seconds)
            
            # Then STOP
            self.stop()
            self.log("Timed turn RIGHT finished")
            return True
        except Exception as exc:
            self.log(f"Timed turn right failed: {exc}")
            self.stop()
            return False
    
    def forward_for(self, duration_seconds: float, speed_percent: float) -> bool:
        """
        Move forward for a specified duration.
        
        Continuously sends ENABLE ON, sets REVERSE OFF, sets throttle mapped from speed_percent,
        continues for duration_seconds, then STOPs.
        
        Args:
            duration_seconds: How long to move forward (seconds)
            speed_percent: Speed percentage (0-100)
            
        Returns:
            True if command sent successfully
        """
        if not self.connection:
            self.log("Cannot move forward: MAVLink not connected")
            return False
        
        try:
            self.log(f"Driving forward for {duration_seconds} seconds")
            # ENABLE: ON (RC5 = 1100) - will be sent continuously by loop
            self._start_enable_loop()
            self._current_enable = self.RELAY_ON
            # REVERSE: OFF (RC6 = 1100)
            self._current_reverse = self.REVERSE_OFF
            # STEERING: CENTER (CH1 = 1500)
            # THROTTLE: calculated PWM (CH3)
            throttle_pwm = self._calculate_throttle(speed_percent)
            self._send_rc_override(
                ch1=self.STEER_CENTER,
                ch3=throttle_pwm,
                ch5=self.RELAY_ON,
                ch6=self.REVERSE_OFF
            )
            
            # Continue for duration_seconds
            time.sleep(duration_seconds)
            
            # Then STOP
            self.stop()
            self.log("Forward movement finished")
            return True
        except Exception as exc:
            self.log(f"Forward movement failed: {exc}")
            self.stop()
            return False
    
    def backward_for(self, duration_seconds: float, speed_percent: float) -> bool:
        """
        Move backward for a specified duration.
        
        Continuously sends ENABLE ON, sets REVERSE ON (RC6=2000), sets throttle mapped from speed_percent,
        continues for duration_seconds, then STOPs.
        
        Args:
            duration_seconds: How long to move backward (seconds)
            speed_percent: Speed percentage (0-100)
            
        Returns:
            True if command sent successfully
        """
        if not self.connection:
            self.log("Cannot move backward: MAVLink not connected")
            return False
        
        try:
            self.log(f"Driving backward for {duration_seconds} seconds")
            # ENABLE: ON (RC5 = 1100) - will be sent continuously by loop
            self._start_enable_loop()
            self._current_enable = self.RELAY_ON
            # REVERSE: ON (RC6 = 2000)
            self._current_reverse = self.REVERSE_ON
            # STEERING: CENTER (CH1 = 1500)
            # THROTTLE: calculated PWM (CH3)
            throttle_pwm = self._calculate_throttle(speed_percent)
            self._send_rc_override(
                ch1=self.STEER_CENTER,
                ch3=throttle_pwm,
                ch5=self.RELAY_ON,
                ch6=self.REVERSE_ON
            )
            
            # Continue for duration_seconds
            time.sleep(duration_seconds)
            
            # Then STOP
            self.stop()
            self.log("Backward movement finished")
            return True
        except Exception as exc:
            self.log(f"Backward movement failed: {exc}")
            self.stop()
            return False
