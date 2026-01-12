"""Project-wide configuration constants."""

# Fixed baudrate for supported RPLIDAR devices.
LIDAR_BAUDRATE = 115200

# Visualization guide defaults.
# Green boundary distance (cm) aligns with the LiDAR green threshold.
GREEN_MAX_CM = 240
GREEN_MAX = GREEN_MAX_CM  # compatibility alias

# Default side offsets for boundary lines (cm).
DEFAULT_BLUE_BOUNDARY_CM = 50
DEFAULT_GREEN_BOUNDARY_CM = 100

# Default LiDAR ingestion tuning.
# Bump buffer to reduce serial stalls on first start; tune as needed.
DEFAULT_LIDAR_BUFFER_SIZE = 1024
DEFAULT_LIDAR_READ_HZ = 8.0

# Side-wall horizontal offset (mm) from the centerline.
SIDE_WALL_OFFSET_MM = DEFAULT_BLUE_BOUNDARY_CM * 10  # legacy compatibility
CART_HALF_MM = SIDE_WALL_OFFSET_MM  # compatibility alias

# GPS speed factor for time-based movement calculation when GPS is unavailable.
# Formula: time_sec = distance_m * GPS_SPEED_FACTOR
# This factor represents seconds per meter for dead-reckoning movement.
GPS_SPEED_FACTOR = 1.5  # seconds per meter (adjust based on vehicle speed characteristics)

# Time-based movement constants for obstacle avoidance (version 75)
# These values control timed turns and forward movement when GPS/compass are not used
ROTATION_TIME_SEC = 1.5  # Duration for timed turns (left/right)
FORWARD_TIME_SEC = 0.8   # Duration for timed forward movement after turns
SPEED_CAP_PERCENT = 25.0  # Speed percentage cap for obstacle avoidance maneuvers (maps to RC3 = 1250)