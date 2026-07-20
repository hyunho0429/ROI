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

INS estimated speed ─> PI speed control ─> accel / brake
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

`origin_lat/origin_lon`이 없는 공식 GPS 5열 파일은 CSV의 `EastOffset`과
`NorthOffset`을 맵 UTM 원점으로 사용한다. 두 값은 각 샘플의 위치가 아니라
맵마다 고정된 상수이므로 모든 행에서 같아야 한다. 경로와 실시간 GPS 모두
아래처럼 동일한 원점을 사용한다.

단, `EastOffset/NorthOffset`은 ROS `GPSMessage` 필드이고 UDP GPS의 NMEA
RMC/GGA 문장에는 포함되지 않는다. UDP-only 실행에서는 MGeo `global_info.json`
또는 `--utm-origin-x/--utm-origin-y`로 이 상수를 설정한다. 제공된 GPS 기록기는
선택한 상수를 CSV 모든 행에 함께 저장하므로 이후 실행에서 자동 복원된다.

```text
x = UTM_Easting(longitude, latitude) - EastOffset
y = UTM_Northing(longitude, latitude) - NorthOffset
z = altitude - MGeo_origin_z
```

지원 예시는 다음과 같다.

```csv
x,y,z,target_speed,lat,lon,alt,origin_lat,origin_lon,origin_alt
0,0,0,1.0,,,,37.24098167,126.774355,0
0.380822,0.726466,0,1.0,37.24098833,126.774360,0,37.24098167,126.774355,0
```

```text
latitude longitude altitude eastOffset northOffset
```

쉼표 CSV와 공백/탭 TXT, 헤더 없는 ENU 2/3열 및 GPS 5열 파일을 모두
지원한다. 헤더는 공식 영문명뿐 아니라
`위도,경도,고도,동쪽 좌표,북쪽 좌표`도 인식한다.

## 2026 국토부 대회 전역 경로

대회 제공 `2026_molit_comp_global_path.txt`는 헤더 없는 `x y z` 3열이며,
단위는 meter인 K-City MGeo 로컬 ENU 좌표이다. 4,430개 원본 점은
`R_KR_PR_K-city_2025/link_set.json`의 점과 일치한다. 범위는
`x=-159.24~75.41`, `y=-550.49~345.79`, `z=28.11~28.59 m`이고 한 바퀴가
약 2,184.6 m인 폐곡선이다.

경로 파일에는 좌표 변환을 다시 적용하지 않는다. 실시간 NMEA GPS만
`global_info.json`의 UTM 52N 원점 `(302595, 4124145, 0)`을 빼서 같은 로컬
ENU로 변환한다. 이 파일이 기본 `--path`이므로 옵션을 생략해도 대회 경로로
주행한다.

대회 허용 네트워크만 사용하는 경로 기록 명령은 다음과 같다.

```bash
python3 src/path_planning/src/morai_gps_csv_recorder.py \
  --port 3001 \
  --output src/path_planning/data/morai_global_path.csv \
  --sample-distance 0.5
```

CSV에는 `latitude/longitude/altitude`, `global_enu_x/y/z`, 고정
`projection_crs/east_offset/north_offset/up_offset`을 함께 저장한다. 과거 IMU는
경로 형상이 아니며 현재 위치추정에도 재사용할 수 없으므로 저장하지 않는다.

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

아래 값은 `morai_competition_config.py`에 정의된 코드 기본값이다. MORAI의
Destination/Host IP와 Port를 이 값과 일치시켜야 한다.

| UDP 항목 | 방향 | 기본 포트 |
|---|---|---:|
| GPS | SIM → 코드 | 3001 |
| IMU | SIM → 코드 | 4001 |
| Competition Vehicle Status | SIM → 코드 | 909 |
| CollisionData | SIM → 코드 | 907 |
| Ego Ctrl Cmd | 코드 → SIM | 9090 |

MORAI에서 GPS/IMU/Competition Status/CollisionData의 Destination IP는 코드를
실행하는 PC, Destination Port는 위 수신 포트로 맞춘다. `Ego-0 > Cmd Control`은
코드가 보내는 `--control-ip/--control-port`를 수신하도록 연결한다. 차량 UI에서
`Q`를 눌러 `AV-ExternalCtrl`로 전환하고 기어가 D인지 확인해야 한다. 코드는
Competition Status가 mode 2와 gear 4를 회신하기 전까지 가속하지 않고 brake만
보낸다.

CollisionData 907과 Competition Status 909처럼 1024 미만 포트는 Linux에서
권한이 필요할 수 있다.
가능하면 MORAI와 코드 양쪽 포트를 1024 이상으로 바꾸고, 대회 설정상 909를
유지해야 하면 실행 환경의 권한 설정을 확인한다.

25.S4 기본 `Ego Ctrl Cmd`는 공개 23/24 계열과 같은 55 byte,
`data_length=23`으로 전송한다. payload는 `ctrl_mode, gear, longCmdType,
velocity, acceleration, accel, brake, front_steer`이다. 2026년 공식 예제의
59 byte 후륜 조향 형식은 `--control-protocol 26r1`로만 선택한다. 실행 직후
안전 제동 상태로 `ctrl_mode=2`,
`gear=4` 패킷을 반복 전송한다. MORAI UI에서 기존 AutoMode로 불리던 모드는
24.R2부터 `AV-ExternalCtrl`로 표시되므로, Competition Status가 mode 2와 D를
회신한 뒤에만 가속을 시작한다.

규정의 `longCmdType=1`에도 조향이 포함된다. `accel/brake`와 함께
`front_steer`를 같은 패킷으로 보낼 수 있다. 공식 UDP 예제에 맞춰 명령값은
차량 최대 조향각 대비 정규화 값 `[-1, 1]`로 보내고, Competition Status의
조향 피드백은 degree로 읽는다.

이 프로그램은 ROS 토픽을 구독하지 않고 UDP 데이터그램만 사용한다. 공식
문서의 ROS 기본 토픽은 GPS `/gps`, IMU `/Imu`지만 UDP에서는 토픽명이
아니라 센서 Network Setting의 Destination IP/Port가 중요하다. 현재 파서는
공식 GPS NMEA0183 RMC/GGA, IMU 107/115 byte, CollisionData 181 byte 구조에
맞는다. 23.R1.0 문서에서 IMU는 전체 107 byte·데이터 80 byte이며, GPS는
RMC/GGA NMEA 문장이므로 고정된 단일 패킷 크기를 제시하지 않는다. 115 byte
IMU는 초/나노초 타임스탬프가 추가된 형식으로 함께 지원한다.
`Competition Vehicle Status`는 공개 문서에 정의가 없어, 대회 환경에서
관측한 181/229 byte 패킷만 엄격히 검사한다.

IMU 115-byte 패킷의 센서 타임스탬프는 파싱하고 범위를 검사한다. 다만 GPS
NMEA, Competition Status와 동일한 시간축으로 EKF 이벤트 순서를 보장하기 위해
실시간 융합의 `dt`에는 각 UDP 데이터그램의 local monotonic 수신 시간을 쓴다.

Camera와 3D LiDAR도 대회 허용 UDP지만 이 GPS/IMU INS Stanley의 필수 입력은
아니다. 카메라는 timestamp/index/size가 붙은 분할 JPEG이고, 3D LiDAR는 선택한
Velodyne 모델의 UDP 프로토콜을 따르므로 필요 없는 포트는 열지 않는다. 이후
차선/장애물 인지를 추가할 때 별도 perception 모듈로 연결한다.

## 실행

```bash
python3 -m pip install -r src/path_planning/requirements.txt

sudo "$(which python3)" src/path_planning/src/morai_stanley_ins_udp.py \
  --path src/path_planning/data/2026_molit_comp_global_path.txt
```

위 명령에서 생략한 기본값은 코드의 `morai_competition_config.py`에 있다.
GPS `3001`, IMU `4001`, Competition Status `909`, CollisionData `907`,
제어 목적지 `192.168.0.170:9090`, 목표 속도 `10 km/h`이다. 목표 속도는
고정값이고, INS가 추정한 현재 속도와의 오차로 accel/brake를 계산한다.

`--control-ip`에는 MORAI가 실행되는 PC의 IPv4 주소를 넣는다. 같은 PC라면
`127.0.0.1`을 사용할 수 있다.

전체 주행 전에는 안전한 brake 명령과 Competition Status 피드백으로 수신을
검증한다.

```bash
sudo "$(which python3)" src/path_planning/src/morai_udp_control_check.py
```

`PASS`가 나오면 mode 2, gear 4와 brake feedback까지 확인된 것이다. 통제된
빈 공간에서만 `--drive-test`를 추가해 0.1 accel을 1초간 시험한다. 종료 시에는
항상 full brake를 다섯 번 전송한다.

시작 로그에는 다음 내용이 보여야 한다.

```text
localization: GPS/IMU/status-aided 15-state error-state EKF INS
Stanley: front axle 3.00 m, no look-ahead target, fixed speed 10.0 km/h
requesting AV-ExternalCtrl (ctrl_mode=2) and Drive (gear=4)
Competition control state: ctrl_mode=2 (AV-ExternalCtrl), gear=4 (D)
```

주행 로그의 `cmd=(accel,normalized steering,brake)`와 Competition Status가
회신한 `feedback=(accel,front steer deg,brake)`를 함께 확인한다. `cmd`의 accel이
0보다 큰데 `feedback` accel이 계속 0이면 MORAI가 제어 패킷을 받지 못한 것이므로
Cmd Control의 Host IP/Port를 점검한다. feedback에도 accel이 들어오는데 차량이
움직이지 않으면 gear, 충돌 상태 또는 차량 dynamics 설정을 확인한다.

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

## 확인한 MORAI 공식 자료

- [통신 메시지 프로토콜](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R1.0/ros-1)
- [센서 통신 프로토콜](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/-35)
- [맵 좌표계](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/-9)
- [네트워크 설정 UI](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/ui)
- [MORAI 공식 55-byte EgoCtrlCmd 정의](https://github.com/MORAI-Autonomous/MORAI-NetworkModule/blob/78e88558588451bdf9a10baf04d575c9aa3e8587/lib/define/EgoCtrlCmd.py)
- [MORAI 공식 IMU timestamp 정의](https://github.com/MORAI-Autonomous/MORAI-NetworkModule/blob/78e88558588451bdf9a10baf04d575c9aa3e8587/lib/define/IMU.py)
