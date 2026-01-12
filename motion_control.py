"""Motion control helpers using MAVLink commands instead of RC override."""

from __future__ import annotations

import time
from typing import Callable

from mavlink_comm import MavlinkComm
from mavlink_motion_commands import MavlinkMotionCommands
from settings import Settings
from shared_data import SharedData


class MotionControl:
    def __init__(self, shared: SharedData, settings: Settings, mav: MavlinkComm, log_fn: Callable[[str], None] = print) -> None:
        self.shared = shared
        self.settings = settings
        self.mav = mav
        self.log = log_fn
        self.manual_enabled = False
        # Initialize MAVLink motion command executor
        self.motion_commands = MavlinkMotionCommands(
            mav_connection=mav.master,
            movement_coefficient=shared.movement_coefficient,
            gps_available=shared.gps_fix,
            log_fn=log_fn,
        )

    def _update_motion_commands(self) -> None:
        """Update motion command executor with latest settings."""
        self.motion_commands.movement_coefficient = self.shared.movement_coefficient
        self.motion_commands.gps_available = self.shared.gps_fix

    # Manual control ------------------------------------------------------------
    def enable_manual(self) -> None:
        """Arm and enable manual control mode."""
        self.manual_enabled = True
        self.shared.update_status(
            manual_control=True,
            manual_override=True,
            avoidance_enabled=False,
            mission_active=False,
            status_message="Manual control enabled",
        )
        self.mav.arm()
        # Note: ch5 and ch8 may still be needed for arming/aux functions
        # but motion channels (1-4) are now controlled via MAVLink commands
        self.log("Manual control enabled")

    def disable_manual(self) -> None:
        """Disable manual control and stop movement."""
        self.manual_enabled = False
        self.stop_vehicle()
        self.shared.update_status(
            manual_control=False,
            manual_override=False,
            movement_state="STOPPED",
            status_message="Manual control disabled",
        )
        self.log("Manual control disabled")

    def handle_key(self, direction: str) -> None:
        """Handle keyboard input for manual control using MAVLink commands."""
        if not self.manual_enabled:
            return

        self._update_motion_commands()

        # Convert speed percentage to m/s (approximate conversion)
        speed_pct = self.settings.speed_limit1_pct
        speed_mps = (speed_pct / 100.0) * 0.6  # Scale to reasonable m/s

        if direction == "UP":
            # Move forward using MAVLink command
            command = {
                "command": "move_forward",
                "params": {
                    "speed": speed_mps,
                    "distance_m": None,
                    "time_s": 0.2,  # Short duration for continuous movement
                },
            }
            self.motion_commands.execute(command)
            self.shared.update_status(movement_state="MOVING", status_message="Manual forward")

        elif direction == "DOWN":
            # Move backward using MAVLink command
            back_speed_pct = self.settings.speed_limit2_pct
            back_speed_mps = (back_speed_pct / 100.0) * 0.4
            command = {
                "command": "move_backward",
                "params": {
                    "speed": back_speed_mps,
                    "distance_m": None,
                    "time_s": 0.2,
                },
            }
            self.motion_commands.execute(command)
            self.shared.update_status(movement_state="REVERSING", status_message="Manual reverse")

        elif direction == "LEFT":
            # Turn left using MAVLink command
            command = {
                "command": "turn_left",
                "params": {
                    "angle_deg": 15.0,  # Small turn for continuous steering
                    "turn_speed_deg": 30.0,
                },
            }
            self.motion_commands.execute(command)

        elif direction == "RIGHT":
            # Turn right using MAVLink command
            command = {
                "command": "turn_right",
                "params": {
                    "angle_deg": 15.0,
                    "turn_speed_deg": 30.0,
                },
            }
            self.motion_commands.execute(command)

    def release_keys(self, horizontal: bool = False, vertical: bool = False) -> None:
        """Stop movement when keys are released."""
        if not self.manual_enabled:
            return
        if vertical:
            self.stop_vehicle()
            self.shared.update_status(movement_state="STOPPED")

    # Autonomous helpers --------------------------------------------------------
    def apply_speed(self, percent: int, steer_pwm: int = 1500) -> int:
        """
        Apply forward speed using MAVLink commands.

        Note: steer_pwm parameter is kept for compatibility but steering
        should be done via separate turn commands.
        """
        self._update_motion_commands()
        speed_mps = (percent / 100.0) * 0.6
        command = {
            "command": "move_forward",
            "params": {
                "speed": speed_mps,
                "distance_m": None,
                "time_s": 0.5,  # Short forward movement
            },
        }
        self.motion_commands.execute(command)
        movement = "MOVING" if percent > 5 else "STOPPED"
        self.shared.update_status(movement_state=movement)
        return int(1000 + (percent / 100) * 1000)  # Return PWM for compatibility

    def stop_vehicle(self) -> None:
        """Stop vehicle movement using MAVLink stop command."""
        self._update_motion_commands()
        command = {"command": "stop", "params": {}}
        self.motion_commands.execute(command)
        self.shared.update_status(movement_state="STOPPED")

    def move_backward_one_meter(self) -> None:
        """Command a short reverse movement to back away from obstacles."""
        self._update_motion_commands()
        back_speed_pct = self.settings.speed_limit3_pct
        back_speed_mps = (back_speed_pct / 100.0) * 0.4

        self.shared.update_status(movement_state="REVERSING", status_message="Backing away from obstacle")

        command = {
            "command": "move_backward",
            "params": {
                "speed": back_speed_mps,
                "distance_m": 1.0,
                "time_s": None,
            },
        }
        self.motion_commands.execute(command)

    # Low-level (kept for compatibility, but should not be used for motion) ---
    def set_channels(
        self,
        ch1: int | None = None,
        ch3: int | None = None,
        ch5: int | None = None,
        ch6: int | None = None,
        ch8: int | None = None,
    ) -> None:
        """
        Set RC channels (DEPRECATED for motion control).

        This function is kept for compatibility but motion channels (1-4)
        should now use MAVLink commands instead. Only ch5 and ch8 (aux channels)
        may still be needed for arming/aux functions.
        """
        # Only update status, do not send RC override for motion channels
        if ch5 is not None or ch8 is not None:
            # Aux channels may still be needed
            self.mav.set_manual_rc(ch5=ch5, ch8=ch8)
        self.shared.update_status(manual_override=self.manual_enabled)

    def _percent_to_pwm(self, percent: int) -> int:
        """Convert percentage to PWM (kept for compatibility)."""
        percent = max(0, min(percent, 100))
        return int(1000 + (percent / 100) * 1000)
