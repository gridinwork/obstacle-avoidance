"""Entry point for v15-obstacle_mission (Raspberry Pi ready)."""

from __future__ import annotations

from avoidance import AvoidanceController
from gui import run_gui
from lidar_core import LidarCore
from mavlink_comm import MavlinkComm
from motion_control import MotionControl
from settings import Settings
from shared_data import SharedData
from utils.camera_manager import CameraManager


def main() -> None:
    settings = Settings()
    shared = SharedData(settings=settings)

    def log(msg: str) -> None:
        print(msg)
        shared.update_status(status_message=msg)

    mav = MavlinkComm(shared, settings, log_fn=log)
    lidar = LidarCore(shared, settings, log_fn=log)
    motion = MotionControl(shared, settings, mav, log_fn=log)
    camera_mgr = CameraManager(shared, log_fn=log)
    avoidance = AvoidanceController(shared, settings, mav, motion, log_fn=log)

    # GUI runs in main thread
    try:
        run_gui(shared, settings, mav, avoidance, motion, camera_mgr, lidar)
    finally:
        if not shared.get_lidar_points():
            log("[LIDAR] ERROR: No data received")
        shared.stop()
        lidar.stop_stream(keep_connection=True)
        avoidance.stop()
        mav.stop()
        camera_mgr.stop()


if __name__ == "__main__":
    main()

