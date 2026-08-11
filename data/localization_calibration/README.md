# Localization 정렬 기록

이 폴더는 GPS·IMU·EgoVehicleStatus 동시 기록과 정렬 결과를 관리한다.

## 필요한 파일

- alignment_stationary.bag: 정지 상태 GPS 분산 확인
- alignment_straight.bag: 저속 직선 yaw 확인
- 계산 결과: morai_ws/config/localization_alignment.yaml

## 기록 순서

1. 정지 상태를 먼저 기록한다.
2. 차량을 새로 시작하고 저속 직선을 기록한다.
3. 두 bag의 timestamp와 topic 개수를 확인한다.
4. GPSMessage 실제 field를 확인한다.
5. GPS map 결과와 /Ego_topic position을 비교한다.
6. 통과 기준을 만족할 때만 transform_validated를 true로 바꾼다.

bag 파일은 용량이 크므로 Git에 추가하지 않는다. MORAI 버전, map 이름,
scenario 이름, 센서 주기, 기록 날짜를 별도 기록으로 남긴다.

