"""Motion control helpers using RC override only (Version 76)."""

from __future__ import annotations

from typing import Callable

from mavlink_comm import MavlinkComm
from movement_mavlink import MovementController
from settings import Settings
from shared_data import SharedData


class MotionControl:
    """Motion control using only RC override via MovementController."""
    
    def __init__(self, shared: SharedData, settings: Settings, mav: MavlinkComm, log_fn: Callable[[str], None] = print) -> None:
        self.shared = shared
        self.settings = settings
        self.mav = mav
        self.log = log_fn
        self.manual_enabled = False
        # Initialize MovementController for relay-based differential robot
        self.movement_controller = MovementController(log_fn=log_fn)
        # Sync connection when available
        if mav.master:
            self.movement_controller.connection = mav.master

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
        """Handle keyboard input for manual control using MovementController timed functions."""
        if not self.manual_enabled:
            return

        # Sync connection if needed
        if self.mav.master and not self.movement_controller.connection:
            self.movement_controller.connection = self.mav.master

        if not self.movement_controller.connection:
            self.log("MovementController not connected")
            return

        speed_pct = self.settings.speed_limit1_pct

        if direction == "UP":
            # Move forward using timed function (short duration for continuous feel)
            self.movement_controller.forward_for(0.2, float(speed_pct))
            self.shared.update_status(movement_state="MOVING", status_message="Manual forward")

        elif direction == "DOWN":
            # Note: backward_for() not implemented yet - using stop for now
            self.log("Backward movement not implemented in manual control")
            self.shared.update_status(movement_state="STOPPED", status_message="Backward not available")

        elif direction == "LEFT":
            # Turn left using timed function
            self.movement_controller.timed_turn_left(0.3, float(speed_pct))

        elif direction == "RIGHT":
            # Turn right using timed function
            self.movement_controller.timed_turn_right(0.3, float(speed_pct))

    def release_keys(self, horizontal: bool = False, vertical: bool = False) -> None:
        """Stop movement when keys are released."""
        if not self.manual_enabled:
            return
        if vertical:
            self.stop_vehicle()
            self.shared.update_status(movement_state="STOPPED")

    # Autonomous helpers --------------------------------------------------------
    def stop_vehicle(self) -> None:
        """Stop vehicle movement using MovementController."""
        # Sync connection if needed
        if self.mav.master and not self.movement_controller.connection:
            self.movement_controller.connection = self.mav.master
        
        # Use MovementController for relay-based stop
        if self.movement_controller.connection:
            self.movement_controller.stop()
        
        self.shared.update_status(movement_state="STOPPED")
