# MORAI UDP Stanley + EKF-INS 제어기

권장 실행기는 `morai_stanley_ins_udp.py`이다. ROS/catkin 없이 MORAI UDP만
사용하며, 경로 좌표 처리와 waypoint 전처리는
[AutoVehicle](https://github.com/shinejihun1227/AutoVehicle/tree/main/ros_ws/src)의
방식을 독립 Python 구조로 이식했다. 위치추정은 이 저장소의 15상태 error-state
EKF-INS, 횡제어는 Stanley, 종제어는 대회 규정의 `longCmdType = 1`이다.

## 전체 데이터 흐름

```text
GPS UDP ─┐
IMU UDP ─┼─> 15-state EKF-INS ─> x, y, z, yaw, speed ─> Stanley
Status ──┘                                           └─> steering

Competition speed ─> PI speed control ─> accel / brake
CollisionData ─────────────────────────> emergency brake
```

경로 CSV에 기록된 과거 IMU·가속도·각속도는 현재 차량 위치추정에 사용하지
않는다. 현재 차량은 실시간 GPS/IMU/Competition Status로 추정한다. CSV에서는
경로의 `x/y/z` 또는 GPS 열과 고정 원점만 읽는다.

## AutoVehicle에서 가져온 경로 처리

- `origin_lat/origin_lon/origin_alt`가 있는 CSV는 기록 당시 원점 기준 로컬 ENU
- 실시간 GPS도 CSV에 기록된 같은 원점으로 변환
- 0.5 m 미만으로 붙은 waypoint 제거
- 9개 waypoint 이동평균으로 경로 노이즈 완화
- 최근 target 뒤쪽으로 되돌아가지 않는 진행 방향 검색
- 한 제어 주기에서 최대 50개 segment만 검색
- 전륜축 제어점 기본 오프셋 3.0 m
- CTE 0.05 m deadband
- 조향 저역통과 필터와 최대 변화율 0.4 rad/s

로컬 ENU 변환은 AutoVehicle과 동일한 근거리 식이다.

```text
x = 6378137.0 * rad(lon - origin_lon) * cos(rad(origin_lat))
y = 6378137.0 * rad(lat - origin_lat)
z = altitude - origin_alt
```

`origin_lat/origin_lon`이 없는 공식 GPS 5열 파일은 MGeo의 UTM CRS와
`local_origin_in_global`을 사용하는 기존 맵 원점 ENU 변환으로 처리한다.

지원 예시는 다음과 같다.

```csv
x,y,z,target_speed,lat,lon,alt,origin_lat,origin_lon,origin_alt
0,0,0,1.0,,,,37.24098167,126.774355,0
0.380822,0.726466,0,1.0,37.24098833,126.774360,0,37.24098167,126.774355,0
```

```text
latitude longitude altitude eastOffset northOffset
```

`target_speed` 열이 있더라도 주행 속도에는 사용하지 않는다. 현재 런타임은
`--target-speed-kmh` 하나만 사용하며 기본값은 10 km/h이다.

## 순수 Stanley 조향

Pure Pursuit처럼 전방 목표점을 고르는 lookahead distance를 사용하지 않는다.
전륜축 제어점과 가장 가까운 경로 segment의 접선각 및 횡오차를 사용한다.

```text
front_x = x + front_axle_offset * cos(yaw)
front_y = y + front_axle_offset * sin(yaw)

heading_error = wrap(path_yaw - yaw)
cte_term = atan2(k * cte, speed + softening)
steering = heading_gain * heading_error - cte_gain * cte_term
```

여기서 `path_yaw`는 최근접 segment의 접선각이다. `target-search-window=50`은
계산량을 제한하는 인덱스 검색 범위이며 lookahead distance가 아니다.

기본 튜닝값은 AutoVehicle 설정을 따른다.

| 항목 | 기본값 |
|---|---:|
| front axle offset | 3.0 m |
| Stanley gain | 0.22 |
| softening | 3.0 m/s |
| heading error gain | 1.0 |
| cross-track error gain | 0.55 |
| controller steering limit | 21.77° |
| MORAI physical steering limit | 36.25° |
| waypoint spacing | 0.5 m |
| smoothing window | 9 |

공개 UDP 문서에는 GPS/차량 position 기준점이 앞차축인지 명시되어 있지 않다.
기본 3.0 m는 AutoVehicle 구현을 재현한 값이다. 실제 차량 기준점이 rear axle이
아니면 `--control-point-offset`을 실측값으로 바꿔야 한다.

## EKF-INS

권장 실행기는 다음 15상태 오차 벡터를 사용한다.

```text
[position error(3), velocity error(3), attitude error(3),
 gyro bias(3), accelerometer bias(3)]
```

- IMU 각속도와 specific force로 3-D strapdown 예측
- GPS 위치·고도·RMC 속도로 위치/속도 오차 보정
- IMU quaternion으로 자세 오차 보정
- Competition Status의 signed speed와 non-holonomic constraint 적용
- 정지 시 gyro/accelerometer bias 초기 보정
- GPS 음영 중 IMU + 차량 속도로 계속 예측, GPS 복구 시 재보정

상세 내용은 [README_TUNNEL_LOCALIZATION.md](README_TUNNEL_LOCALIZATION.md)를
참고한다.

## 네트워크 설정

아래 값은 코드 기본값이다. MORAI의 Destination/Host IP와 Port를 실행 옵션에
맞춰야 한다.

| UDP 항목 | 방향 | 기본 포트 |
|---|---|---:|
| GPS | SIM → 코드 | 3001 |
| IMU | SIM → 코드 | 4001 |
| Competition Vehicle Status | SIM → 코드 | 909 |
| CollisionData | SIM → 코드 | 5678 |
| Ego Ctrl Cmd | 코드 → SIM | 9090 |

Competition Status의 909처럼 1024 미만 포트는 Linux에서 권한이 필요할 수 있다.
가능하면 MORAI와 코드 양쪽 포트를 1024 이상으로 바꾸고, 대회 설정상 909를
유지해야 하면 실행 환경의 권한 설정을 확인한다.

## 실행

```bash
python3 -m pip install -r src/path_planning/requirements.txt

sudo "$(which python3)" src/path_planning/src/morai_stanley_ins_udp.py \
  --path src/path_planning/data/morai_global_path.csv \
  --gps-port 3001 \
  --imu-port 4001 \
  --competition-status-port 909 \
  --collision-port 5678 \
  --control-ip 192.168.0.170 \
  --control-port 9090 \
  --target-speed-kmh 10
```

`--control-ip`에는 MORAI가 실행되는 PC의 IPv4 주소를 넣는다. 같은 PC라면
`127.0.0.1`을 사용할 수 있다.

시작 로그에는 다음 내용이 보여야 한다.

```text
localization: GPS/IMU/status-aided 15-state error-state EKF INS
Stanley: front axle 3.00 m, no look-ahead target, fixed speed 10.0 km/h
```

주행 로그의 `cmd=(accel,steering,brake)`를 확인한다. 정지인데 accel이 0보다
큰데도 움직이지 않으면 코드의 경로 계산보다 MORAI Cmd Control의 Host IP/Port,
외부 제어 활성화, gear D 및 `longCmdType 1` 수신 설정을 먼저 점검한다.

## 주요 튜닝 옵션

- `--control-point-offset`: 위치 기준점에서 전륜축 제어점까지 전방 거리
- `--stanley-gain`, `--cross-track-error-gain`: 횡오차 복귀 강도
- `--heading-error-gain`: 경로 heading 정렬 강도
- `--max-steering-deg`: Stanley가 낼 수 있는 조향각 제한
- `--vehicle-max-steering-deg`: MORAI normalized steering ±1의 실제 조향각
- `--steering-filter-alpha`: 작을수록 조향이 부드럽지만 반응이 느림
- `--max-steering-rate-radps`: 초당 조향각 변화 제한
- `--minimum-waypoint-spacing`, `--waypoint-smoothing-window`: 경로 전처리
- `--morai-steer-sign`: 좌우가 반대일 때 `1` 또는 `-1`로 변경

GPS/IMU/Competition Status가 stale이거나 CollisionData가 충돌을 알리거나 경로
끝에 도달하면 즉시 brake 명령을 전송한다. `Ctrl+C` 종료 시에도 brake 패킷을
다섯 번 전송한다.
