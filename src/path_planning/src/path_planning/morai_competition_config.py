"""Competition network defaults used by every standalone UDP runner."""


BIND_IP = "0.0.0.0"
GPS_PORT = 3001
IMU_PORT = 4001
# MORAI Host Port -> algorithm Destination Port for simulator publishers.
COMPETITION_STATUS_HOST_PORT = 9080
COMPETITION_STATUS_PORT = 9081
COLLISION_HOST_PORT = 9091
COLLISION_PORT = 9092
# Ego Ctrl Cmd travels in the opposite direction: algorithm Destination Port
# (UDP source) -> MORAI Host Port (UDP destination).
CONTROL_IP = "192.168.56.1"
CONTROL_PORT = 9093
CONTROL_DESTINATION_PORT = 9094
TARGET_SPEED_KMH = 10.0
