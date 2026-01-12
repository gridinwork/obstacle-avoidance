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

