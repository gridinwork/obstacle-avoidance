from rplidar import RPLidar

PORT = "/dev/ttyUSB0"

lidar = RPLidar(PORT, baudrate=115200)
lidar.start_motor()
lidar.set_pwm(660)

print("Reading first 5 scans...")

for i, scan in enumerate(lidar.iter_scans()):
    print("Scan", i, "points:", len(scan))
    if i >= 5:
        break

lidar.stop_motor()
lidar.disconnect()

