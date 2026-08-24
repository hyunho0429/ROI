"""Competition network defaults used by every standalone UDP runner."""


BIND_IP = "0.0.0.0"
GPS_PORT = 3001
IMU_PORT = 4001
# MORAI Host Port -> algorithm Destination Port for simulator publishers.
COMPETITION_STATUS_HOST_PORT = 9080
COMPETITION_STATUS_PORT = 9081
COLLISION_HOST_PORT = 9091
COLLISION_PORT = 9092
LIDAR_HOST_PORT = 2000
LIDAR_PORT = 2001
# Loopback UDP pose stream from the driving EKF to LiDAR deskew.  This is not
# a MORAI sensor port and must remain distinct from every simulator channel.
LIDAR_POSE_IP = "127.0.0.1"
LIDAR_POSE_PORT = 4012
# Ego Ctrl Cmd travels in the opposite direction: algorithm Destination Port
# (UDP source) -> MORAI Host Port (UDP destination).
CONTROL_IP = "192.168.0.148"
CONTROL_PORT = 9093
CONTROL_DESTINATION_PORT = 9094
TARGET_SPEED_KMH = 40.0

# Competition rulebook vehicle: 2023 Hyundai IONIQ 5.
VEHICLE_LENGTH_M = 4.635
VEHICLE_WIDTH_M = 1.892
VEHICLE_WHEELBASE_M = 3.0
VEHICLE_FRONT_OVERHANG_M = 0.845
VEHICLE_REAR_OVERHANG_M = 0.790
