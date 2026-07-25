# MORAI UDP Pure Pursuit + EKF-INS 제어기

권장 실행기는 `morai_pure_pursuit_ins_udp.py`이다. ROS/catkin 없이 MORAI UDP만
사용하며, 경로 좌표 처리와 waypoint 전처리는
[AutoVehicle](https://github.com/shinejihun1227/AutoVehicle/tree/main/ros_ws/src)의
방식을 독립 Python 구조로 이식했다. 위치추정은 이 저장소의 15상태 error-state
EKF-INS, 횡제어는 Pure Pursuit, 종제어는 대회 규정의 `longCmdType = 1`이다.

## 전체 데이터 흐름

```text
GPS UDP ─┐
IMU UDP ─┼─> 15-state EKF-INS ─> x, y, z, yaw, speed ─> Pure Pursuit
Status ──┘                                           └─> steering

INS estimated speed ─> PID speed control ─> accel / brake
CollisionData ─────────────────────────> emergency brake
```

## `beta_drive` 메시지 타입 대응

내부 필드 의미는 MORAI 공식
[`MORAI-ROS_morai_msgs`의 `beta_drive`](https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs/tree/beta_drive)
메시지 정의에 맞췄다. 단, 대회 규정에 따라 ROS transport나 ROS serialization은
사용하지 않고 MORAI UDP wire format만 사용한다.
검토 기준은 `beta_drive`의 `45c6baf148f2327f4c9fefd48262f25bdfe4b567`이다.

- `CtrlCmd`: `longlCmdType`, `accel`, `brake`, `steering`, `velocity`,
  `acceleration`과 대응한다. UDP envelope의 `ctrl_mode`, `gear`는 별도 필드다.
- `GPSMessage`: NMEA RMC/GGA에서 `latitude`, `longitude`, `altitude`, `status`를
  만들고, NMEA에 없는 `eastOffset`, `northOffset`은 MGeo 고정 원점을 붙인다.
- IMU: `sensor_msgs/Imu`와 같은 `orientation(x,y,z,w)`, `angular_velocity`,
  `linear_acceleration` 의미와 단위를 사용한다.
- `CollisionData`: UDP 각 object record의 공통 global offset을 메시지 레벨
  `global_offset_x/y/z`로 노출하고 실제 충돌 객체를 `collision_object`로 제공한다.
- `EgoVehicleStatusExtended`와 Competition Vehicle Status는 필드가 겹치지만
  동일 wire message가 아니다. 후자는 관측된 181/229-byte 전용 파서로 유지한다.
- `Competition.msg`의 `start_signal/team_name/mission_success`는 Competition
  Vehicle Status와 다른 메시지이며, 허용 UDP 규격이 제공되지 않아 수신하지 않는다.

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

`target_speed` 열이 있으면 lookahead target의 속도를 사용하고, 대회 기본
3열 경로처럼 속도 열이 없으면 `--target-speed-kmh`(기본 45 km/h)를 사용한다.

## Pure Pursuit 조향

첨부된 `purepursuit.py`의 핵심 곡률식을 사용한다. 현재 위치에서 경로 진행
방향으로 lookahead 거리만큼 떨어진 target을 보간하고, 차량 heading과 target
bearing 사이 각도 `alpha`를 계산한다. 첨부 코드는 Twist의 yaw rate를 출력하지만
MORAI Ego Ctrl Cmd는 steering을 요구하므로 kinematic bicycle 식으로 변환한다.

```text
Ld_command = clip(Ld_base + speed_gain * |speed|, Ld_min, Ld_max)
alpha = wrap(atan2(target_y - y, target_x - x) - yaw)
curvature = 2 * sin(alpha) / distance_to_target
steering = atan(wheelbase * curvature)
```

경로 진행도는 최근접 segment 투영점의 누적 거리로 계산하고 target은 누적 거리
기준으로 보간한다. 따라서 0.5 m waypoint에서 index를 한 칸씩 증가시키는 방식보다
속도와 packet 주기 변화에 안정적이다. `target-search-window=50`은 최근 진행
위치 주변의 최근접 segment 검색 범위이다.

| 항목 | 기본값 |
|---|---:|
| vehicle | 2023 Hyundai IONIQ 5 |
| length / width | 4.635 m / 1.892 m |
| wheelbase | 3.0 m (Competition Status 수신값 우선) |
| base lookahead | 2.5 m |
| speed lookahead gain | 0.5 s |
| lookahead limits | 2.5~12.0 m |
| control point offset | 0.0 m |
| steering offset | +3.0° |
| controller steering limit | 21.77° |
| MORAI physical steering limit | 36.25° |
| waypoint spacing | 0.5 m |
| smoothing window | 9 |

GPS 센서 기준점과 bicycle model 제어점이 다르면 `--control-point-offset`으로
전방(+) 또는 후방(-) 오프셋을 설정한다.

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

첨부 코드의 역할은 다음처럼 대회용 UDP/ENU 구현에 반영했다.

- `alignment.py`: 시작 후 기본 2초 동안 brake를 유지하며 gyro/가속도 bias 평균
- `mechanization.py`: IMU 각속도·specific force의 자세/속도/위치 적분
- `system_dynamics.py`: `[dp,dv,dtheta,dbg,dba]` 15상태 오차 전이
- `ekf.py`: GPS 위치·RMC 속도·차량 속도 보정과 bias feedback
- `open_run_main.py`: Alignment → EKF-INS → Pure Pursuit 실행 순서

MORAI IMU가 absolute quaternion을 제공하므로 실제 로봇용 코드의 초기 직진 yaw
alignment는 사용하지 않는다. 초기 직진 명령 없이 brake 상태에서 quaternion과
bias를 초기화하는 편이 대회 출발 절차에 안전하다.

상세 내용은 [README_TUNNEL_LOCALIZATION.md](README_TUNNEL_LOCALIZATION.md)를
참고한다.

## 네트워크 설정

아래 값은 `morai_competition_config.py`에 정의된 코드 기본값이다. MORAI의
Destination/Host IP와 Port를 이 값과 일치시켜야 한다.

| UDP 항목 | 방향 | MORAI Host Port | 알고리즘 Destination Port |
|---|---|---:|---:|
| GPS | SIM → 코드 | 센서 설정값 | 3001 |
| IMU | SIM → 코드 | 센서 설정값 | 4001 |
| Competition Vehicle Status | SIM → 코드 | 9080 | 9081 |
| CollisionData | SIM → 코드 | 9091 | 9092 |
| Ego Ctrl Cmd | 코드 → SIM | 9093 | 9094 |

현재 네트워크 기준 MORAI Host IP는 `192.168.56.1`, 알고리즘 PC Destination IP는
`192.168.56.101`이다. MORAI에서 GPS/IMU/Competition Status/CollisionData의
Destination IP는 `192.168.56.101`, Destination Port는 위 수신 포트로 맞춘다.
Status와 Collision의 Host Port도 각각 9080, 9091로 맞춘다.
`Ego-0 > Cmd Control`은 Host Port 9093, Destination Port 9094로 설정한다.
코드는 로컬 9094에 bind한 뒤 `--control-ip`의 9093으로 명령을 보낸다. 기본
`--control-ip`는 MORAI Host IP인 `192.168.56.1`이다. 차량 UI에서
`Q`를 눌러 `AV-ExternalCtrl`로 전환하고 기어가 D인지 확인해야 한다. 코드는
Competition Status가 mode 2와 gear 4를 회신하기 전까지 가속하지 않고 brake만
보낸다.

현재 Status/Collision/Ctrl Cmd 포트는 모두 1024 이상이므로 Linux 일반 사용자로
bind할 수 있다.

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

### Competition Vehicle Status 확인

현재 파서가 지원하는 공통 payload는 다음과 같다.

| byte offset | 크기 | 데이터 |
|---:|---:|---|
| 0 | 11 | header `#MoraiInfo$` |
| 11 | 4 | data length: 152 또는 200 |
| 15 | 12 | aux data |
| 27 | 8 | sec, nsec |
| 35 | 2 | ctrl mode, gear |
| 37 | 4 | signed velocity (km/h) |
| 41 | 4 | map data ID |
| 45 | 8 | accel/brake pedal |
| 53 | 12 | vehicle size x/y/z (m) |
| 65 | 12 | overhang/wheelbase/rear overhang (m) |
| 77 | 12 | position x/y/z (m) |
| 89 | 12 | roll/pitch/yaw (degree) |
| 101 | 12 | velocity x/y/z (km/h) |
| 113 | 12 | angular velocity x/y/z (degree/s) |
| 125 | 12 | acceleration x/y/z (m/s²) |
| 137 | 4 | front steer (degree) |
| 141 | 38 | link ID |
| 179 | 48 | 229-byte형만: tire force/slip/stiffness |
| 끝 | 2 | `0x0D 0x0A` |

전체 UDP payload는 기본형 181 byte 또는 확장형 229 byte이다. `tcpdump`나
Wireshark에서 보이는 UDP Length는 8-byte UDP header를 포함하므로 각각 189,
237로 표시될 수 있다.

실제 대회 패킷을 읽고 길이, 송신 포트, header, hex와 모든 해석 필드를 출력하려면
주행 노드를 먼저 종료하고 다음 명령을 실행한다. 두 프로세스가 Destination Port
9081을 동시에 bind하지 않도록 주의한다.

```bash
python3 src/path_planning/src/morai_competition_status_inspect.py \
  --host-port 9080 --destination-port 9081 --count 10 --hex-bytes 0
```

주행기를 실행한 상태에서 패킷을 수동 캡처하려면 `tcpdump`를 사용한다.

```bash
# 화면에 payload 길이와 전체 hex 출력
sudo tcpdump -ni any -s 0 -c 10 -XX \
  'udp src port 9080 and dst port 9081'

# Wireshark에서 열 수 있는 pcap 저장
sudo tcpdump -ni any -s 0 -c 100 \
  -w competition_status.pcap \
  'udp src port 9080 and dst port 9081'

# 현재 UDP bind 상태 확인
sudo ss -lunp | grep -E ':(9081|9092|9094)\\b'
```

`parser: INCOMPATIBLE`가 표시되면 출력된 `payload`, `header`, `data_length`, hex를
기준으로 대회 전용 구조를 확정해야 한다. 원본 pcap에는 시뮬레이터/PC IP 정보가
포함될 수 있으므로 외부 공유 전 확인한다.

IMU 115-byte 패킷의 센서 타임스탬프는 파싱하고 범위를 검사한다. 다만 GPS
NMEA, Competition Status와 동일한 시간축으로 EKF 이벤트 순서를 보장하기 위해
실시간 융합의 `dt`에는 각 UDP 데이터그램의 local monotonic 수신 시간을 쓴다.

Camera와 3D LiDAR도 대회 허용 UDP지만 이 GPS/IMU INS Pure Pursuit의 필수 입력은
아니다. 카메라는 timestamp/index/size가 붙은 분할 JPEG이고, 3D LiDAR는 선택한
Velodyne 모델의 UDP 프로토콜을 따르므로 필요 없는 포트는 열지 않는다. 이후
차선/장애물 인지를 추가할 때 별도 perception 모듈로 연결한다.

## 실행

권장 실행 방법은 `roslaunch`이다. ROS는 프로세스 실행과 인자 전달에만 사용하고,
센서와 제어 통신은 모두 UDP로 유지한다.

```bash
cd ~/catkin_ws
python3 -m pip install -r src/path_planning/requirements.txt
catkin_make
source devel/setup.bash
roslaunch path_planning morai_pure_pursuit_udp.launch
```

기본 종방향 PID는 `main` 브랜치와 같은 30 Hz, `Kp=0.075`, `Ki=0.0001`,
`Kd=0.025`이다. 양수 출력은 accel, 음수 출력은 brake로 분리되고 Pure Pursuit
steering과 함께 `longCmdType=1` 패킷으로 전송된다. launch 인자는 다음처럼
덮어쓸 수 있다.

```bash
roslaunch path_planning morai_pure_pursuit_udp.launch \
  target_speed_kmh:=45.0 \
  speed_kp:=0.075 speed_ki:=0.0001 speed_kd:=0.025
```

Competition Status Destination Port `9081`, CollisionData Destination Port
`9092`, Ego Ctrl Cmd Destination/source Port `9094`는 모두 일반 사용자로 bind할
수 있는 1024 이상 포트이다.

Python 단독 실행도 계속 지원한다.

```bash
python3 -m pip install -r src/path_planning/requirements.txt

python3 src/path_planning/src/morai_pure_pursuit_ins_udp.py \
  --path src/path_planning/data/2026_molit_comp_global_path.txt
```

위 명령에서 생략한 기본값은 코드의 `morai_competition_config.py`에 있다.
GPS `3001`, IMU `4001`, Competition Status `9080 -> 9081`, CollisionData
`9091 -> 9092`, Ego Ctrl Cmd `9094 -> 192.168.56.1:9093`, 목표 속도
`45 km/h`이다. 목표 속도는
고정값이고, INS가 추정한 현재 속도와의 오차로 accel/brake를 계산한다.

`--control-ip`에는 MORAI가 실행되는 PC의 IPv4 주소를 넣는다. 같은 PC라면
`127.0.0.1`을 사용할 수 있다.

전체 주행 전에는 안전한 brake 명령과 Competition Status 피드백으로 수신을
검증한다.

```bash
python3 src/path_planning/src/morai_udp_control_check.py
```

`PASS`가 나오면 mode 2, gear 4와 brake feedback까지 확인된 것이다. 통제된
빈 공간에서만 `--drive-test`를 추가해 0.1 accel을 1초간 시험한다. 종료 시에는
항상 full brake를 다섯 번 전송한다.

시작 로그에는 다음 내용이 보여야 한다.

```text
localization: GPS/IMU/status-aided 15-state error-state EKF INS
alignment: hold brake for 2.0s (at least 20 IMU samples)
Pure Pursuit: Ld=clip(2.50+0.50*speed, 2.50, 12.00)m, wheelbase=3.00m, steering_offset=+3.00deg, fixed speed 45.0 km/h
requesting AV-ExternalCtrl (ctrl_mode=2) and Drive (gear=4)
Competition control state: ctrl_mode=2 (AV-ExternalCtrl), gear=4 (D)
```

주행 로그의 `cmd=(accel,normalized steering,brake)`와 Competition Status가
회신한 `feedback=(accel,front steer deg,brake)`를 함께 확인한다. `cmd`의 accel이
0보다 큰데 `feedback` accel이 계속 0이면 MORAI가 제어 패킷을 받지 못한 것이므로
Cmd Control의 Host IP/Port를 점검한다. feedback에도 accel이 들어오는데 차량이
움직이지 않으면 gear, 충돌 상태 또는 차량 dynamics 설정을 확인한다.

## 주요 튜닝 옵션

- `--lookahead-distance`, `--lookahead-speed-gain`: 기본/속도 비례 lookahead
- `--minimum-lookahead`, `--maximum-lookahead`: lookahead 제한
- `--wheelbase`: 상태 패킷 수신 전 사용할 기본 wheelbase
- `--control-point-offset`: GPS/INS 위치에서 제어점까지 전방(+)·후방(-) 거리
- `--steering-offset-deg`: Pure Pursuit 조향 방향 반대로 적용해 조향각 크기를 줄이는 offset
- `--max-steering-deg`: Pure Pursuit이 낼 수 있는 조향각 제한
- `--vehicle-max-steering-deg`: MORAI normalized steering ±1의 실제 조향각
- `--alignment-seconds`, `--alignment-min-samples`: 정지 bias 초기화 조건
- `--steering-filter-alpha`: 작을수록 조향이 부드럽지만 반응이 느림
- `--max-steering-rate-radps`: 초당 조향각 변화 제한
- `--minimum-waypoint-spacing`, `--waypoint-smoothing-window`: 경로 전처리
- `--morai-steer-sign`: 좌우가 반대일 때 `1` 또는 `-1`로 변경
- `--speed-kp`, `--speed-ki`, `--speed-kd`: 종방향 PID 게인
- `--control-rate-hz`: PID 및 제어 루프 주기(기본 30 Hz)

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
