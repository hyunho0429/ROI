# 센서 원본 설정

cam_set.json은 MORAI 카메라 원본 설정이다.

현재 파일에는 카메라 3개가 들어 있으며 GPS, IMU, LiDAR 목록은 비어
있다. 따라서 이 파일은 카메라 설치·화각·해상도를 확인하는 원본으로
사용한다.

ROS 네이티브 운용 기준은 다음 파일이다.

- morai_ws/config/ros_topics.yaml
- morai_ws/docs/MORAI_ROS_토픽_계약.md

ROS 네이티브 입력:

- /gps: morai_msgs/GPSMessage
- /Imu: sensor_msgs/Imu
- /lidar3D: sensor_msgs/PointCloud2
- 카메라별 topic: sensor_msgs/CompressedImage

원본 cam_set.json의 UDP 값은 127.0.0.1과 9290~9295이다. 이는 대회
알고리즘 PC의 UDP fallback 계약인 1100/1101 계열과 다르므로 그대로
실행 설정으로 사용하지 않는다.

