# Obstacle Mission v16 (Pixhawk + LiDAR)

Python/PyQt5 desktop app that fuses PX4/ArduPilot MAVLink control with RPLIDAR
obstacle detection, avoidance, and optional camera preview. v16 adds MAVLink
GUIDED-mode obstacle bypass (yaw + velocity setpoints), configurable avoidance
angles/distances, cyan boundary-aware side selection, and on-screen maneuver
arrows. See `pixhawk_commands.md` for the exact MAVLink payloads used.

## Structure
- `main.py` — bootstraps modules and launches the GUI (fullscreen on Pi).
- `gui.py` — PyQt5 interface with LiDAR view (left) and Camera view (right).
- `lidar_core.py` — RPLIDAR discovery and scan parsing on `/dev/ttyUSB*`.
- `avoidance.py` — obstacle classification and steering/throttle decisions.
- `mavlink_comm.py` — MAVLink connection on `/dev/ttyACM*`, RC overrides, mode changes.
- `settings.py` — persistent JSON-backed configuration + device enumeration helpers.
- `shared_data.py` — thread-safe shared state.
- `utils/camera_manager.py` — Pi Camera (libcamera/Picamera2) and USB camera capture.
- `version.py` — version string displayed in the title bar.

## Hardware ports (Pi OS)
- Pixhawk: `/dev/ttyACM0`, `/dev/ttyACM1`, `/dev/ttyAMA0`
- LiDAR (USB): `/dev/ttyUSB0`, `/dev/ttyUSB1`, `/dev/serial/by-id/*`
- Cameras: Pi Camera (CSI via libcamera/Picamera2) or USB `/dev/video0`, `/dev/video1`

## Settings dialog (dynamic dropdowns)
- Pixhawk Port: auto-lists `/dev/ttyACM*` and `/dev/ttyAMA0`
- MAVLink Baud: `57600` or `115200`
- LiDAR Port: auto-lists `/dev/ttyUSB*` and `/dev/serial/by-id/*`
- Camera Selection: `Raspberry Pi Camera (libcamera)` plus detected `/dev/video*`
- All values are persisted in `settings.json`.

## Dependencies (Raspberry Pi OS 64-bit)
```bash
sudo apt update
sudo apt install -y python3-pyqt5 python3-opencv python3-serial \
    python3-picamera2 libcamera-dev v4l-utils
python3 -m pip install --upgrade pyserial pymavlink rplidar-roboticia numpy
```
Use `install_lib.py` for an automated, Pi-friendly installer that prefers apt
packages to avoid slow ARM builds.

## Running
```bash
python3 main.py
```
The GUI starts fullscreen on Raspberry Pi. Use **Settings** to pick devices,
click **Connect Devices** (statuses turn green), enable the "Show LiDAR point
cloud"/"Show Camera" checkboxes, then click **Start Visualization**. Use
**Stop Visualization** to halt streams while keeping connections alive.

## Notes
- Add your user to the `dialout` group for MAVLink/LiDAR access:
  `sudo usermod -a -G dialout $USER` then reboot.
- If the Pi Camera is selected but blank, confirm `libcamera-hello` works and
  Picamera2 is installed.

## Quick testing
- LiDAR: choose `/dev/ttyUSB*` (or `/dev/serial/by-id/*`), Connect, then Start Visualization; confirm point cloud draws.
- Camera: pick Pi Camera or `/dev/video*`, Connect, enable "Show Camera", Start Visualization; verify live feed.
- MAVLink: pick `/dev/ttyACM*/ttyAMA0` + baud, Connect, watch status turn green after heartbeat.
- Stop Visualization: point cloud/camera threads halt, connection statuses stay green.

## License

