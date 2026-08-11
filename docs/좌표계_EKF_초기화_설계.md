# 좌표계 통합 및 EKF 초기 로컬라이제이션 설계

## 1. 목적

이 문서는 MORAI에서 수신하는 GPS·IMU와 대회에서 제공한 MGeo·전역 경로를 하나의 좌표계로 통합하고, 그 결과를 이용해 초기 로컬라이제이션을 수행하기 위한 기준이다.

핵심 원칙은 다음과 같다.

1. 주행 중 사용하는 기준 좌표계는 MGeo와 대회 경로가 사용하는 로컬 ENU 좌표계로 고정한다.
2. GPS 수신 시 첫 GPS를 무조건 (0, 0)으로 삼지 않는다. GPS 좌표를 MGeo map 좌표로 변환하는 정적 변환을 먼저 검증한다.
3. IMU는 고주기 예측, GPS는 위치 보정에 사용한다. 정지 상태의 GPS course를 차량 yaw로 사용하지 않는다.
4. 경로를 차량의 첫 위치에 임의로 옮기는 방식은 최종 방식으로 사용하지 않는다.

---

## 2. 현재 파일에서 확인된 좌표계

### 2.1 MGeo 전역 정보

global_info.json의 실제 값은 다음과 같다.

| 항목 | 실제 값 | 의미 |
|---|---:|---|
| 전역 좌표계 | +proj=utm +zone=52 +datum=WGS84 +units=m +no_defs | WGS84 UTM Zone 52, 단위 m |
| 작업 좌표계 | 전역 좌표계와 동일 | MGeo 원본의 기준 |
| local_origin_in_global | [302595.0, 4124145.0, 0.0] | 로컬 좌표계의 전역 UTM 기준점 |
| 경로 변경 링크 포함 | false | 차선 변경용 링크를 별도 포함하지 않음 |

link_set.json과 node_set.json의 실제 점들은 UTM 전체 좌표가 아니라 작은 범위의 로컬 미터 좌표이다.

- 링크 점 범위: x=-404.516~112.354, y=-775.046~571.222, z=27.143~37.380
- 노드 점 범위: x=-396.378~80.523, y=-775.046~569.528, z=27.223~37.367

따라서 실제 주행 중 경로 추종에 사용할 기준은 MGeo 로컬 좌표계이고, UTM은 GPS 변환의 중간 좌표로 사용한다.

### 2.2 대회 전역 경로

2026_molit_comp_global_path.txt는 공백으로 구분된 x y z 순서의 로컬 미터 좌표이다.

| 항목 | 확인값 |
|---|---:|
| 점 개수 | 4,430개 |
| 첫 점 | (-131.689798, -428.331023, 28.543960) |
| 마지막 점 | 첫 점과 동일 |
| X 범위 | -159.242~75.414 |
| Y 범위 | -550.494~345.790 |
| XY 경로 길이 | 약 2,184.61 m |
| 평균 점 간격 | 약 0.493 m |
| MGeo 링크 점과 일치 | 4,430/4,430 |
| 사용 링크 수 | 72개 |

전역 경로는 별도의 GPS 경로가 아니라 MGeo 링크 형상에서 추출된 동일한 로컬 좌표계의 경로이다. 따라서 매 실행마다 경로를 GPS 원점에 맞춰 이동시키면 안 된다.

### 2.3 샘플 시나리오

2026_molit_comp_sample_scene.json의 mapInfo는 다음과 같다.

    mapName: R_KR_PR_K-city_2025
    eastOffset: 302595.0
    northOffset: 4124145.0
    globalCoordinateSystem: UTM52N
    scenarioCoordinateSystem: ENU

이는 global_info.json의 local_origin_in_global과 같은 오프셋을 사용한다. 샘플 ego의 초기값은 다음과 같다.

    ego 위치 = (-131.485992, -427.960999, 28.883000)
    ego yaw  = 62.515도

경로 첫 점과 비교하면 다음과 같다.

    경로 첫 점과 ego XY 거리 = 약 0.422 m
    경로 초기 접선 방향      = 약 61.298도
    ego와 경로 방향 차이     = 약 1.217도
    ego z와 경로 z 차이       = 약 0.339 m

이 값은 초기화 검증용 기준값으로 활용할 수 있다. 단, 시나리오 JSON은 실행 중 수신되는 센서값이 아니라 시뮬레이션 초기조건이다.

### 2.4 기존 GPS 로컬라이저의 문제

기존 gps_localizer.py는 UDP fallback 경로에서 GPS UDP 3001의 NMEA RMC/GGA를 받아 첫 GPS를 (0,0)으로 저장하고, 이후 GPS 변화량을 로컬 ENU로 변환한다. 이동 후 변화량으로 yaw를 구하며 IMU는 사용하지 않는다.

ROS 네이티브 경로에서는 이 parser를 사용하지 않는다. MORAI가 발행하는 /gps의 morai_msgs/GPSMessage를 직접 구독하고, beta_drive 메시지의 실제 필드명과 단위를 확인한 뒤 map 변환을 수행한다.

UDP 상대 원점 방식은 GPS 자체의 상대 좌표로는 동작하지만 MGeo map과 같은 원점을 보장하지 않는다. 따라서 GPS 위치와 경로가 평행 이동된 채로 남을 수 있다. 새 로컬라이저는 첫 GPS 원점을 최종 map 원점으로 사용하지 않는다.

### 2.5 차량 및 센서 좌표계

| 프레임 | 축 정의 | 용도 |
|---|---|---|
| map | ENU: X 동쪽, Y 북쪽, Z 위쪽 | MGeo·경로·최종 차량 pose |
| base_link | X 전방, Y 좌측, Z 위쪽 | 차량 기준 제어·센서 변환 |
| imu_link | IMU 실제 축을 확인한 뒤 base_link와 정적 변환 | EKF 예측 |
| velodyne | LiDAR 장착 좌표계 | 점군·장애물 검출 |
| front_camera 등 | 카메라 장착 좌표계 | 영상 검출 |
| optical frame | X 오른쪽, Y 아래쪽, Z 전방 | 영상 픽셀 투영 |

cam_set.json의 장착값은 다음과 같다.

| 카메라 | 위치 (x,y,z) m | 회전 (roll,pitch,yaw) 도 | 해상도 | 주기 |
|---|---|---|---|---:|
| Camera-1 | (1.90, 0.00, 1.20) | (0, 2, 0) | 1280x720 | 20 Hz |
| Camera-2 | (1.15, 0.65, 1.20) | (0, 10, 70) | 640x480 | 20 Hz |
| Camera-3 | (1.15, -0.65, 1.20) | (0, 10, 290) | 640x480 | 20 Hz |

원본 cam_set.json의 9290~9295 localhost 포트는 참고값이다. 실제 Ubuntu 브릿지 공통 포트는 Camera-1부터 각각 1100->1101, 1110->1111, 1120->1121, 1130->1131로 고정한다. 현재 cam_set에는 카메라가 3개만 정의되어 있으므로 Camera-4의 장착값과 frame 이름은 별도로 확정해야 한다.

### 2.6 MORAI ROS 메시지 입력

ROS 네이티브 연결에서는 다음 메시지를 사용한다.

| 토픽 | 메시지 타입 | 좌표·단위 | 역할 |
|---|---|---|---|
| /gps | morai_msgs/GPSMessage | beta_drive 실제 필드 확인 필요 | EKF GPS 보정 |
| /Imu | sensor_msgs/Imu | ROS 표준, 각속도 rad/s·가속도 m/s² | EKF 예측 |
| /Ego_topic | morai_msgs/EgoVehicleStatus | ENU, position m, heading deg | 개발용 정답 비교 |
| /lidar3D | sensor_msgs/PointCloud2 | ROS LiDAR 좌표 | detection |
| 카메라별 topic | sensor_msgs/CompressedImage | JPEG 압축 영상 | detection |

EgoVehicleStatus의 position은 MORAI 문서상 ENU이므로 샘플 시나리오 ego와 MGeo 경로의 직접 비교 기준으로 사용할 수 있다. 단, 이 토픽은 EKF 입력으로 사용하지 않는다.

ROS 네이티브와 UDP fallback은 입력 경로가 다르다. ROS 네이티브에서는 /gps와 /Imu를 사용하고, UDP fallback에서만 기존 GPS 3001·IMU 4001 parser를 사용한다.

---

## 3. 최종 공통 좌표계

런타임에서는 아래 관계를 유지한다.

    MGeo / global path  ───────────────┐
                                       │ 같은 map 좌표
    GPS -> UTM52 -> GPS-to-map 정렬 ───┼──> map -> base_link -> 센서 프레임
    IMU -> imu_link -> EKF 예측 ───────┘

모든 localization 출력은 map 기준의 map -> base_link pose로 한다.

### 3.1 UTM과 MGeo 로컬 좌표의 관계

현재 1차 가설은 다음과 같다.

    x_map = x_utm - 302595.0
    y_map = y_utm - 4124145.0

여기서 UTM은 WGS84 UTM Zone 52이다. 다만 global_info.json만으로 축 방향·부호·yaw 회전까지 100% 확정할 수 없으므로, 위 식은 라이브 MORAI GPS와 EgoVehicleStatus를 대조하기 전까지 검증 대기 상태로 둔다.

검증 결과가 불일치하면 다음을 확인한다.

1. GPS 위경도가 WGS84인지 확인
2. MORAI의 UTM/ENU offset 적용 방향과 축 순서 확인
3. map yaw와 MORAI yaw의 부호 및 기준축 확인

### 3.2 고도와 yaw

- 초기 경로 추종은 우선 XY를 사용한다.
- GPS 고도와 MGeo Z는 센서 기준점과 도로 중심선 기준점이 다를 수 있어 초기 EKF에서 강하게 결합하지 않는다.
- 샘플 ego와 경로 Z가 약 0.339 m 차이 나는 것이 근거다.
- 차량 yaw는 map에서 X축 기준 반시계방향 증가 라디안으로 통일한다.
- 정지 상태 GPS course는 무효로 처리한다.

---

## 4. GPS 정렬(alignment) 절차

### 4.1 정렬의 의미

정렬의 결과는 경로를 옮기는 값이 아니라 GPS 좌표계에서 MGeo map 좌표계로 가는 변환이다.

    p_map = R(dyaw) * p_gps_local + t_xy
    yaw_map = yaw_gps + dyaw

p_gps_local은 GPS를 임시 원점 기준 ENU로 변환한 값이고, t_xy와 dyaw는 MGeo map에 맞추어 보정하는 값이다.

### 4.2 오프라인 분석

1. MGeo와 전역 경로를 map으로 읽는다.
2. 경로 각 점의 진행 방향과 누적 거리를 계산한다.
3. 샘플 ego 위치를 경로에 투영한다.
4. ego 초기 위치·yaw와 경로 접선의 차이를 기록한다.
5. 경로 점에 link ID를 붙여 런타임 링크 검증에 사용한다.

오프라인 기준은 경로 첫 점과 샘플 ego의 거리 약 0.422 m, 방향 차이 약 1.217도이다.

### 4.3 MORAI 실행 중 검증

1. 샘플 시나리오 시작점에서 차량을 정지시킨다.
2. /Ego_topic의 EgoVehicleStatus 위치·heading과 /gps·/Imu를 같은 시각에 기록한다.
3. 정지 상태 GPS 10개 이상을 모은다.
4. GPSMessage의 실제 필드를 확인하고 UTM52 또는 임시 ENU로 변환한다.
5. 같은 시각의 EgoVehicleStatus.position을 map 기준값으로 삼아 t_xy와 dyaw를 계산한다.
6. 저속 직선 주행으로 GPS 이동 방향과 map 진행 방향이 일치하는지 확인한다.
7. 여러 시각의 정렬 잔차 평균과 표준편차를 기록한다.

EgoVehicleStatus는 개발 검증용 기준값으로만 사용한다. 최종 주행 로직은 규정에서 허용한 인터페이스만 사용하도록 분리한다.

### 4.4 초기 개발 통과 기준

아래는 공식 판정 기준이 아니라 좌표계 검증용 시작값이다.

| 검사 | 시작 기준 |
|---|---:|
| 정지 GPS 평균 위치 표준편차 | XY 각 축 0.5 m 이하 |
| GPS와 MORAI 기준 위치 잔차 | 1.0 m 이하 |
| 이동 후 yaw 정렬 잔차 | 3도 이하 |
| 경로 초기 횡방향 오차 | 1.0 m 이하 |
| 현재 링크 매칭 | 유효 link ID가 지속적으로 검출됨 |

통과하지 못하면 EKF 튜닝보다 먼저 좌표 변환·포트·timestamp를 점검한다.

---

## 5. EKF 기반 초기 로컬라이제이션

### 5.1 1차 상태

처음에는 평면 주행에 맞는 2차원 EKF를 구현한다.

    x = [px, py, yaw, v, gyro_bias, accel_bias]

- px, py: map 좌표의 차량 기준점 위치
- yaw: map 기준 차량 방향
- v: 차량 진행방향 속도
- gyro_bias: yaw rate bias
- accel_bias: 진행방향 가속도 bias

IMU 축과 단위를 검증한 뒤 필요하면 횡속도·3축 자세·3축 bias를 포함하는 상태로 확장한다.

### 5.2 예측과 보정

/Imu 수신 때마다 angular_velocity와 linear_acceleration, dt로 yaw와 속도를 적분하고 map XY를 예측한다. /gps 수신 때는 morai_msgs/GPSMessage의 위경도·고도 필드를 map 좌표로 변환해 px, py를 보정한다.

반드시 처리할 항목은 다음과 같다.

- yaw를 [-pi, pi]로 정규화
- IMU가 도/초인지 라디안/초인지 확인
- 가속도가 중력 보정값인지 raw 값인지 확인
- 메시지 timestamp로 dt 계산
- GPS course는 최소 속도 이상에서만 보조 측정값으로 사용
- GPS가 끊겨도 pose를 0으로 초기화하지 않고 IMU 예측을 지속

규정의 GPS 블랙아웃 미션을 고려하면 GPS health 상태와 EKF 공분산을 함께 출력해야 한다.

### 5.3 초기화 순서

    1. MGeo·경로·link 정보를 map으로 로드
    2. GPS·IMU 포트와 timestamp 상태 확인
    3. GPS 10개 이상 수집
    4. GPS 품질과 분산 확인
    5. GPS -> map 정렬 변환 적용
    6. 초기 px, py 설정
    7. yaw 결정
       - 개발 검증: MORAI 시작 자세
       - 센서 모드: 이동 후 GPS 진행 방향 + 경로 접선 + IMU
    8. 초기 공분산 설정
    9. 경로 투영과 link 매칭으로 검증
   10. 통과 후 localization 상태를 READY로 변경

정지 상태에서는 GPS course로 yaw를 결정하지 않는다. 실제 센서만 사용하는 모드에서는 차량이 짧게 직진한 뒤 경로 접선과 이동 방향을 결합한다.

---

## 6. 실제 값 대조표

| 비교 대상 | 실제 값 | 해석 |
|---|---|---|
| MGeo 로컬 원점 | UTM (302595.0, 4124145.0) | GPS 변환 후 map 좌표를 만드는 기준 후보 |
| 경로 첫 점 | (-131.6898, -428.3310, 28.5440) | map 기준 경로 시작점 |
| 샘플 ego 초기 위치 | (-131.4860, -427.9610, 28.8830) | 경로 첫 점에서 약 0.422 m |
| 샘플 ego yaw | 62.515도 | 경로 접선 약 61.298도와 약 1.217도 차이 |
| 링크 점 전체 범위 | X -404.516~112.354, Y -775.046~571.222 | map 변환 결과가 같은 규모여야 함 |
| 전역 경로 범위 | X -159.242~75.414, Y -550.494~345.790 | 경로 투영·횡오차 검사 범위 |
| ROS GPS | /gps, morai_msgs/GPSMessage | map 변환 후 EKF 보정 |
| ROS IMU | /Imu, sensor_msgs/Imu | EKF 예측 |
| 개발 기준 pose | /Ego_topic, morai_msgs/EgoVehicleStatus | GPS 정렬 잔차 비교 |
| UDP fallback GPS | Ubuntu 3001 | ROS 네이티브 불가 시에만 사용 |
| UDP fallback IMU | Ubuntu 4001 | ROS 네이티브 불가 시에만 사용 |

---

## 7. 바로 실행할 작업

1. GPS·IMU·MORAI 기준 위치를 동시에 10초 기록한다.
2. 첫 GPS 평균과 MORAI map 위치를 비교해 GPS -> map 정적 변환을 계산한다.
3. 정적 변환으로 GPS pose를 map에 올리고, 경로는 이동시키지 않는다.
4. 정지 상태 초기화와 저속 직진 초기화를 각각 검증한다.
5. EKF를 연결하고 GPS를 잠시 끊어도 IMU 예측이 연속되는지 확인한다.
6. 마지막으로 control과 detection을 공통 frame 계약에 연결한다.

첫 구현의 완료 조건은 다음 세 가지이다.

- GPS와 MGeo가 같은 map 좌표에서 겹친다.
- EKF가 공분산을 포함한 초기 pose와 yaw를 정상 출력한다.
- 경로 투영과 현재 link 검증이 반복 실행에서 일관된다.
