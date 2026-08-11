# MORAI ROS 토픽 및 메시지 계약

## 1. 적용 범위

이 문서는 MORAI ROS 네이티브 연결을 사용할 때의 공통 입력·출력 기준이다.

기준 자료는 다음과 같다.

- MORAI-ROS_morai_msgs 저장소의 beta_drive 브랜치
- MORAI SIM ROS Network Settings 공식 문서
- MORAI SIM 센서 통신 공식 문서

MORAI ROS의 센서 입력과 기존 UDP 카메라 브릿지는 서로 다른 전송 경로이다. 둘을 동시에 같은 입력으로 연결하지 않는다.

## 2. 공식 토픽과 메시지 타입

| 방향 | 토픽 | 메시지 타입 | 담당 | 사용 목적 |
|---|---|---|---|---|
| MORAI → Ubuntu | /gps | morai_msgs/GPSMessage | localization | GPS 위치 보정 |
| MORAI → Ubuntu | /Imu | sensor_msgs/Imu | localization | EKF 예측 |
| MORAI → Ubuntu | /Ego_topic | morai_msgs/EgoVehicleStatus | localization 검증 | ENU 기준 위치·heading 대조 |
| MORAI → Ubuntu | /lidar3D | sensor_msgs/PointCloud2 | detection | VLP16 점군 |
| MORAI → Ubuntu | 카메라별 지정 topic | sensor_msgs/CompressedImage | detection | 카메라 영상 |
| MORAI → Ubuntu | /Object_topic | morai_msgs/ObjectStatusList | 검증만 | NPC·보행자·장애물 ground truth |
| MORAI → Ubuntu | /CollisionData | morai_msgs/CollisionData | diagnostics | 충돌 기록 |
| Ubuntu → MORAI | /ctrl_cmd | morai_msgs/CtrlCmd | control | 차량 제어 |

공식 카메라 기본 토픽은 /imag_jpeg/compressed이다. 카메라가 4대이면 이 기본 토픽을 그대로 공용으로 쓰지 말고, MORAI ROS 설정에서 front·left·right·aux에 각각 고유 topic을 지정한다. 현재 프로젝트의 고유 topic은 /camera/front/image/compressed, /camera/left/image/compressed, /camera/right/image/compressed, /camera/aux/image/compressed로 정한다.

공식 LiDAR 3D 토픽은 /lidar3D이며 메시지는 sensor_msgs/PointCloud2이다. 기존 프로젝트에서 사용하던 /sensors/lidar/points는 내부 remap 이름으로만 사용할 수 있고, MORAI 직접 연결의 source topic으로 기록하지 않는다.

## 3. Localization 입력의 의미

### GPS

ROS 네이티브 입력은 NMEA UDP 문자열이 아니라 morai_msgs/GPSMessage이다. 따라서 ROS 네이티브 경로에서는 기존 gps_localizer.py의 UDP NMEA parser를 거치지 않는다.

GPSMessage의 정확한 필드명과 단위는 Ubuntu에 설치한 beta_drive 메시지에서 다음 명령으로 확정한다.

    rosmsg show morai_msgs/GPSMessage

필드 확인 전까지 latitude, longitude, altitude의 이름을 코드에 추측해 고정하지 않는다.

### IMU

IMU는 sensor_msgs/Imu이다. EKF에서는 header.stamp, angular_velocity, linear_acceleration을 사용하고, orientation을 사용할 때는 covariance와 센서 설정을 함께 확인한다.

IMU 축은 imu_link에서 확인한 뒤 base_link로 변환한다. raw 센서 좌표를 map에 바로 적분하지 않는다.

### EgoVehicleStatus

EgoVehicleStatus의 position은 MORAI 문서상 ENU 기준이다.

- position: m
- velocity: m/s
- acceleration: 차량 가속도 벡터
- heading: deg
- wheel_angle: beta_drive 메시지 및 설치 버전의 실제 정의를 확인

이 토픽은 초기 GPS 정렬과 EKF 검증에서 기준값으로 사용할 수 있다. 그러나 EKF의 센서 입력에는 넣지 않는다. 그렇지 않으면 센서 기반 localization을 검증하는 과정에서 정답 pose를 다시 입력하는 데이터 누수가 발생한다.

## 4. 좌표계와 메시지 변환

MGeo와 EgoVehicleStatus의 position은 ENU 로컬 좌표계로 비교한다.

    map:       x 동쪽, y 북쪽, z 위쪽
    base_link: x 전방, y 좌측, z 위쪽

GPSMessage의 위경도·고도를 map으로 바꾼 뒤 다음과 같이 대조한다.

    GPSMessage -> UTM52 또는 local ENU -> map
    Ego_topic.position -----------------> map 기준값

처음에는 Ego_topic.position과 MGeo 경로의 좌표가 직접 같은 범위인지 확인한다. 그 다음 GPS 변환 결과와 Ego_topic.position의 잔차를 계산한다.

MGeo의 local_origin_in_global은 [302595.0, 4124145.0, 0.0]이다. UTM을 중간값으로 사용할 때의 1차 가설은 다음과 같지만, 실제 GPSMessage와 Ego_topic을 동시에 기록해 검증하기 전까지 확정하지 않는다.

    x_map = x_utm - 302595.0
    y_map = y_utm - 4124145.0

## 5. 카메라와 LiDAR 프레임

MORAI ROS에서 sensor_msgs/CompressedImage가 들어오면 카메라별 frame_id를 다음으로 고정한다.

| 센서 | frame_id | 장착 기준 |
|---|---|---|
| 전방 | front_camera | cam_set Camera-1 |
| 좌측 | left_camera | cam_set Camera-2 |
| 우측 | right_camera | cam_set Camera-3 |
| 보조 | aux_camera | Camera-4 값 확정 필요 |
| LiDAR | velodyne | VLP16 |

MORAI 공식 센서 문서에서는 Velodyne 원시 좌표를 x 오른쪽, y 전방, z 위쪽으로 설명하고, ROS 좌표계에서는 x 전방, y 왼쪽, z 위쪽으로 사용한다고 설명한다. 따라서 detection 내부의 velodyne은 ROS 좌표를 기준으로 한다. raw UDP Velodyne packet을 직접 처리하는 경우에는 별도 축 변환이 필요하다.

## 6. Control 메시지

control 팀만 /ctrl_cmd를 발행한다. beta_drive의 CtrlCmd 필드명은 longlCmdType, accel, brake, steering, velocity, acceleration이다.

공식 ROS 문서의 일반 enum 설명과 대회 세부 규정의 종방향 명령 타입이 다를 수 있으므로, 실제 대회 설정에서는 세부 규정과 MORAI 대회 환경을 우선 확인한다. 프로젝트 설정에는 현재 대회 규정 기준값을 기록하되, ROS 메시지 필드명을 longlCmdType으로 통일한다.

## 7. 개발 검증과 대회 실행 분리

개발 단계에서는 다음을 함께 기록한다.

    /gps
    /Imu
    /Ego_topic
    /lidar3D
    카메라 4개 topic

이때 /Ego_topic은 GPS 정렬의 기준값으로만 사용하고, EKF 업데이트에는 넣지 않는다.

대회 실행에서는 규정상 허용된 입력만 사용하며, /Object_topic·/GetTrafficLightStatus와 같은 ground truth 성격의 토픽은 detection 판단 입력으로 사용하지 않는다.

## 8. 원본 자료

- 메시지 정의: https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs/tree/beta_drive
- ROS 네트워크 설정: https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/ros-network-settings
- 센서 통신: https://morai-sim--drive-user-manual--en-22-r2.scrollhelp.site/msdume2/sensors
- 센서 좌표계: https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/sensor-coordinate-system
