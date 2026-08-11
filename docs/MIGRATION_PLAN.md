# 새 구조로 이관하는 순서

## 0단계: 기준 고정

- 새 코드는 morai_ws에서만 시작한다.
- ROS 네이티브 topic과 메시지 타입은 docs/MORAI_ROS_토픽_계약.md를 따른다.
- 좌표계와 팀 간 출력은 docs/INTERFACE_CONTRACT.md를 따른다.
- 기존 카메라 포트는 ROS 네이티브가 불가능한 경우의 UDP fallback으로만 사용한다.
- 기존 코드는 필요한 개념을 확인할 때만 참고하고 다시 복사하지 않는다.

## 1단계: 지도와 경로

- 공식 MGeo 원본을 morai_ws/data/mgeo/<지도버전>/에 추가한다.
- 대회 경로와 MGeo를 겹쳐 보고 좌표계 변환을 결정한다.
- 확정된 경로와 변환을 버전으로 기록한다.

## 2단계: ROS 메시지 수신 확인

- GPS: /gps, morai_msgs/GPSMessage
- IMU: /Imu, sensor_msgs/Imu
- 차량 검증값: /Ego_topic, morai_msgs/EgoVehicleStatus
- LiDAR: /lidar3D, sensor_msgs/PointCloud2
- 카메라: 센서별 /camera/.../image/compressed, sensor_msgs/CompressedImage

각 센서는 먼저 단독으로 topic rate, timestamp age, frame_id, 해상도,
좌표축, 진단 상태를 기록한다.

Ubuntu에서 다음을 확인한다.

    rostopic list
    rostopic type /gps
    rostopic type /Imu
    rostopic type /Ego_topic
    rostopic type /lidar3D
    rosmsg show morai_msgs/GPSMessage

## 3단계: Localization

- GPSMessage의 실제 필드명과 단위를 확정한다.
- /Ego_topic의 ENU 위치와 MGeo 경로가 같은 좌표 범위인지 확인한다.
- GPSMessage를 map으로 변환하고 EgoVehicleStatus와 정렬 잔차를 계산한다.
- IMU 좌표축과 단위를 확인한다.
- GPS+IMU EKF를 구현한다.
- 정지, 직선, 회전, GPS 끊김, 복구를 시험한다.

## 4단계: Detection

- 카메라 topic을 한 대씩 보정하고 검사한다.
- LiDAR의 ROS 좌표축과 전방 ROI를 고정한다.
- 차선·신호·장애물 결과에 confidence와 freshness를 포함한다.
- 단일 센서 결과가 안정된 뒤 카메라·LiDAR 융합을 추가한다.

## 5단계: Control

- Localization과 경로만 먼저 연결한다.
- 저속에서 경로 추종을 확인한다.
- Detection의 안전정지 요청을 추가한다.
- 최종 출력은 /ctrl_cmd의 morai_msgs/CtrlCmd로 보낸다.

## 팀 간 전달 형식

패키지명, 실행 명령, source topic, 메시지 타입, frame_id, 단위, 예상 주기,
고장 시 동작, 테스트 결과를 함께 전달한다.

