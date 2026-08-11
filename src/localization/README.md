# Localization

이 폴더는 MORAI ROS의 GPS와 IMU를 이용해 차량의 map -> base_link pose를 추정한다.

## 공식 입력

- /gps: morai_msgs/GPSMessage
- /Imu: sensor_msgs/Imu
- /Ego_topic: morai_msgs/EgoVehicleStatus, 개발 검증 전용

ROS 네이티브 경로에서는 UDP NMEA GPS parser를 사용하지 않는다. UDP parser는
ROS 연결이 불가능한 실험에서만 fallback으로 사용한다.

구체적인 좌표계, GPS 정렬, EKF 초기화 순서는 다음 문서를 기준으로 한다.

- morai_ws/docs/좌표계_EKF_초기화_설계.md
- morai_ws/docs/MORAI_ROS_토픽_계약.md
- morai_ws/config/ros_topics.yaml
- morai_ws/config/localization.yaml
- morai_ws/docs/INTERFACE_CONTRACT.md

## 책임

- GPSMessage·sensor_msgs/Imu 파싱
- 센서 축·단위·timestamp 표준화
- GPS 좌표를 MGeo map으로 변환
- GPS-MGeo 정렬값 계산 및 검증
- EKF 예측·GPS 보정
- 경로 투영과 현재 링크 매칭
- 다른 팀이 사용할 표준 pose와 공분산 출력

EgoVehicleStatus는 정렬 검증 기준으로 기록하지만 EKF 측정 업데이트에 넣지 않는다.

## 구현 순서

1. /gps·/Imu·/Ego_topic을 동시에 기록한다.
2. GPSMessage의 실제 필드명과 단위를 rosmsg show로 확인한다.
3. docs/좌표정렬_실행절차.md에 따라 정지·직선 bag를 만든다.
4. GPS -> map 정적 변환을 검증하고 config/localization_alignment.yaml에 저장한다.
5. 정지 상태 초기화와 저속 직진 초기화를 각각 검증한다.
6. EKF의 IMU 예측과 GPS 보정을 연결한다.
7. GPS 블랙아웃 동안 IMU 예측이 연속되는지 확인한다.

첫 단계에서는 경로를 차량 시작점에 임의로 이동시키지 않는다. 경로와 MGeo는
map에 고정하고 센서 pose를 그 좌표계로 변환한다.
