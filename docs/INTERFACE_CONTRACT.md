# 팀 간 인터페이스 계약

내부 구현은 바꿀 수 있지만, 토픽 이름·메시지 타입·단위·좌표계·소유권은 팀 전체의 합의 없이 바꾸지 않는다.

공식 MORAI ROS 토픽의 자세한 설명은 docs/MORAI_ROS_토픽_계약.md와 config/ros_topics.yaml을 기준으로 한다.

## 좌표계와 시간

- map: ENU 지도 좌표. x=동쪽, y=북쪽, z=위쪽
- base_link: 차량 좌표. x=전방, y=왼쪽, z=위쪽
- 센서 좌표: imu_link, velodyne, front_camera, left_camera, right_camera, aux_camera
- 각도는 내부 표준에서 라디안, 거리와 고도는 미터, 속도는 m/s
- MORAI EgoVehicleStatus.heading은 deg이므로 변환 노드에서 라디안으로 바꾼다.
- 모든 센서 메시지는 header timestamp와 frame_id를 확인한다.
- map -> base_link 변환의 유일한 소유자는 localization 팀이다.
- detection 팀은 자체 지도 좌표계를 만들지 않고 TF를 사용한다.

## MORAI 네이티브 입력

| 토픽 | 메시지 타입 | 담당 | 비고 |
|---|---|---|---|
| /gps | morai_msgs/GPSMessage | localization | EKF GPS 보정 |
| /Imu | sensor_msgs/Imu | localization | EKF 예측 |
| /Ego_topic | morai_msgs/EgoVehicleStatus | localization 검증 | 정답 pose를 EKF에 넣지 않음 |
| /lidar3D | sensor_msgs/PointCloud2 | detection | VLP16 |
| /camera/front/image/compressed | sensor_msgs/CompressedImage | detection | Camera-1 |
| /camera/left/image/compressed | sensor_msgs/CompressedImage | detection | Camera-2 |
| /camera/right/image/compressed | sensor_msgs/CompressedImage | detection | Camera-3 |
| /camera/aux/image/compressed | sensor_msgs/CompressedImage | detection | Camera-4 |
| /SaveSensorData | morai_msgs/SaveSensorData | recording | 현재 센서 프레임 저장 |
| /ReplayInfo_topic | morai_msgs/ReplayInfo | replay 검증 | Rosbag Replay 통합 상태 |

카메라 공식 기본 topic은 /imag_jpeg/compressed이다. 4대의 영상을 구분하기 위해 MORAI ROS 설정에서 위의 카메라별 topic으로 지정한다.

## Localization 출력

기준 토픽:

    /localization/odometry       nav_msgs/Odometry
    /localization/pose           geometry_msgs/PoseWithCovarianceStamped
    /localization/status         진단·신뢰도 정보

기존 제어기와 연결하는 동안에는 다음 legacy 토픽을 어댑터로 유지할 수 있다.

    /localization/ego_pose
    /localization/ego_twist

Localization 팀은 GPSMessage·Imu 파싱, ENU 원점, 시간 동기화, 공분산,
GPS 끊김 처리, map -> base_link TF까지 책임진다.

EgoVehicleStatus는 정렬 검증용으로 기록하지만 EKF 측정 업데이트에는 사용하지 않는다.

## Detection 입력과 출력

원시 센서 토픽:

    /camera/front/image/compressed
    /camera/left/image/compressed
    /camera/right/image/compressed
    /camera/aux/image/compressed
    /lidar3D

계획된 인식 토픽:

    /detection/lane
    /detection/traffic_light
    /detection/obstacles
    /detection/safety_stop
    /detection/status

Detection 팀은 차선·신호·장애물·안전정지 요청을 발행하지만 직접 조향이나
가속 명령을 발행하지 않는다. /Object_topic과 /GetTrafficLightStatus는
ground truth 성격의 검증용 토픽이므로 detection 판단 입력으로 사용하지 않는다.

## Control 입력과 출력

Control은 다음 정보를 입력으로 사용한다.

    /localization/odometry
    /control/reference_path
    /detection/obstacles
    /detection/safety_stop

Control 팀만 MORAI 공식 제어 토픽 /ctrl_cmd를 발행한다.

내부 제어 노드 출력은 /control/ctrl_cmd로 두고, 마지막 remap 또는 bridge에서
/ctrl_cmd와 morai_msgs/CtrlCmd로 변환한다. beta_drive CtrlCmd의 필드명은
longlCmdType, accel, brake, steering, velocity, acceleration이다.

기록·재생 관련 service는 /Morai_SimProc이며, 실제 자동 제어를 추가하기 전에
rosservice type과 rossrv show로 beta_drive 설치 버전을 확인한다.

MORAI로 제어 명령을 보내는 노드도 control 팀만 관리한다. Localization이
유효하지 않거나 안전정지 요청이 오래된 경우에는 마지막 명령을 무한히
재사용하지 않고 정해진 안전 동작을 수행한다.

## UDP fallback

ROS 네이티브 연결을 사용할 수 없는 실험에서만 기존 UDP 브릿지를 사용한다.
카메라 UDP 포트는 config/sensor_ports.yaml의 1100/1110/1120/1130 →
1101/1111/1121/1131 계약을 따른다. UDP 브릿지가 받은 데이터는 위의
내부 sensor topic으로 변환한 뒤 detection에 전달한다.
