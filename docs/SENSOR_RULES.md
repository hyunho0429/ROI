# 대회 센서 기준과 MORAI 설정

## MORAI ROS 네이티브 센서 입력

| 센서 | ROS 토픽 | 메시지 타입 | 담당 |
|---|---|---|---|
| 카메라 4대 | 센서별 고유 topic | sensor_msgs/CompressedImage | detection |
| 3D LiDAR | /lidar3D | sensor_msgs/PointCloud2 | detection |
| GPS | /gps | morai_msgs/GPSMessage | localization |
| IMU | /Imu | sensor_msgs/Imu | localization |
| 차량 기준값 | /Ego_topic | morai_msgs/EgoVehicleStatus | 검증만 |

MORAI 공식 카메라 기본 topic은 /imag_jpeg/compressed이다. 4대의 영상을
분리하기 위해 센서별로 다음 topic을 MORAI ROS 설정에 넣는다.

- 전방: /camera/front/image/compressed
- 좌측: /camera/left/image/compressed
- 우측: /camera/right/image/compressed
- 보조: /camera/aux/image/compressed

## 카메라 장착

- 최대 4대
- Ground Truth 없음
- 2D/3D Bounding Box 시각화 끔
- 30 Hz 이하

| 카메라 | 위치 (x,y,z) m | 회전 (roll,pitch,yaw) deg | 최대 해상도 | FOV |
|---|---|---|---|---:|
| 전방 | (1.9, 0.0, 1.2) | (0, 2, 0) | 1280x720 | 90도 |
| 좌측 | (1.15, 0.65, 1.2) | (0, 10, 70) | 640x480 | 130도 |
| 우측 | (1.15, -0.65, 1.2) | (0, 10, 290) | 640x480 | 130도 |
| 네 번째 | 별도 확정 | 별도 확정 | 규정 범위 | 규정 범위 |

## LiDAR

- 모델: VLP16
- 최대 1대
- Intensity 사용
- 권장 데이터율 10 Hz
- 대회 규정상 최대 15 Hz
- ROS 네이티브 topic: /lidar3D
- 메시지 타입: sensor_msgs/PointCloud2
- frame_id: velodyne

MORAI 센서 문서 기준으로 raw Velodyne 축은 x 오른쪽, y 전방, z 위쪽이며,
ROS 좌표계에서는 x 전방, y 좌측, z 위쪽으로 사용한다. ROS topic을 바로
사용할 때는 velodyne frame을 ROS 축으로 처리한다.

## GPS·IMU·제어

| 항목 | ROS 네이티브 | 메시지 타입 | 역할 |
|---|---|---|---|
| GPS | /gps | morai_msgs/GPSMessage | EKF 위치 보정 |
| IMU | /Imu | sensor_msgs/Imu | EKF 예측 |
| Ego 기준값 | /Ego_topic | morai_msgs/EgoVehicleStatus | 정렬 검증 |
| CtrlCmd | /ctrl_cmd | morai_msgs/CtrlCmd | 차량 제어 |

GPSMessage의 실제 필드명은 Ubuntu에서 rosmsg show morai_msgs/GPSMessage로
확인한 뒤 코드에 반영한다.

## UDP fallback 포트

ROS 네이티브 연결이 불가능한 경우에만 다음 fallback을 사용한다.

| 항목 | MORAI Host | Ubuntu Destination |
|---|---:|---:|
| 전방 카메라 | 1100 | 1101 |
| 좌측 카메라 | 1110 | 1111 |
| 우측 카메라 | 1120 | 1121 |
| 보조 카메라 | 1130 | 1131 |
| VLP16 LiDAR | 2000 | 2001 |
| GPS | - | 3001 |
| IMU | - | 4001 |
| CtrlCmd | 9093 | 9094 |

## 반드시 확인할 것

- MORAI ROS 설정에서 ROS bridge IP와 topic이 정확한가
- Ubuntu에서 rostopic list로 공식 topic이 보이는가
- rostopic type으로 메시지 타입이 계약과 일치하는가
- 각 topic의 timestamp와 frame_id가 들어오는가
- 카메라 4대가 같은 기본 topic으로 섞이지 않는가
- LiDAR point cloud가 ROS 좌표축으로 들어오는가
- /Ego_topic을 EKF 입력으로 잘못 연결하지 않았는가

