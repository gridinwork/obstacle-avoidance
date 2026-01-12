"""
Lightweight device discovery helpers for the standalone debug app.

Only scans for USB LiDAR ports (/dev/ttyUSB*) and V4L2 cameras (/dev/video*),
and always exposes the Raspberry Pi CSI camera as a selectable option.
"""

from __future__ import annotations

import glob
from typing import List

RASPI_CAM_LABEL = "Raspberry Pi CSI camera (libcamera)"


def list_lidar_ports() -> List[str]:
    """Return available LiDAR serial ports."""
    return sorted(glob.glob("/dev/ttyUSB*"))


def list_cameras() -> List[str]:
    """Return camera options: Pi CSI camera + detected USB /dev/video* nodes."""
    cameras = [RASPI_CAM_LABEL]
    cameras.extend(sorted(glob.glob("/dev/video*")))
    return cameras

