# MORAI 대회용 UDP Stanley 제어기

GPS 터널 음영 주행용 INS 및 dead-reckoning 실행 방법은
[`README_TUNNEL_LOCALIZATION.md`](README_TUNNEL_LOCALIZATION.md)를 참고한다.
기존 `morai_stanley_udp.py`는 GPS가 오래 끊기면 제동하는 평면 EKF 기준
실행기이며, 터널 시험에는 두 전용 실행기 중 하나를 사용한다.

`morai_stanley_udp.py`는 ROS 없이 다음 대회 허용 인터페이스만 사용한다.

- `[UDP] Ego Ctrl Cmd`: 조향 및 accel/brake 송신
- `[UDP] CollisionData`: 충돌 시 3초간 제동
- `[UDP] Competition Vehicle Status`: 실제 속도 피드백
- `[UDP] GPS`: NMEA RMC/GGA 위치·속도
- `[UDP] IMU`: quaternion yaw와 yaw rate

Camera와 3D LiDAR도 허용 센서지만, 미리 기록한 경로를 Stanley로 추종하는
기본 기능에는 필요하지 않아 이 프로그램에서는 열지 않는다. 향후 차선·장애물
인식 기능을 붙일 때 별도 perception 모듈로 연결하는 편이 안전하다.

## 대회 규정과 26.R1 기준 핵심 사항

대회 규정에 따라 `Ego Ctrl Cmd`는 항상 `longCmdType = 1`을 전송한다. 목표
속도와 현재 속도의 차이를 PI 제어기로 accel 또는 brake 값으로 바꾸며, 두
페달을 동시에 명령하지 않는다. 속도 제어 모드인 `longCmdType = 2`를 지정하면
인코더가 오류를 발생시키므로 실수로 규정을 어길 수 없다.

위치추정은 Competition Status의 위치·yaw를 사용하지 않는다. 노이즈가 적용된
GPS와 IMU를 `[x, y, vx, vy, yaw, gyro bias]` EKF로 융합한다. Competition
Status는 종방향 페달 제어를 위한 signed velocity만 사용하며, 상태 패킷이
잠시 끊기면 GPS RMC 속도를 대신 사용한다.

기존 OpenIMU용 `alignment`, strapdown `mechanization`, Earth-frame
`system_dynamics`는 사용하지 않는다. MORAI IMU가 절대 quaternion을 제공하고
Stanley에는 수평 위치·yaw·속도만 필요하므로, 이 전체 INS를 그대로 쓰면 중복
적분과 좌표계 오류가 생긴다. Z는 GPS 고도를 지도 원점 기준으로 바꾼 뒤 저역
통과 필터링한다. Z가 매번 변해야 경로를 저장하거나 추종하는 조건은 아니다.

## 패킷과 좌표

공식 [26.R1 센서 통신 프로토콜](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/26.R1/-35)은
GPS UDP가 UTM x/y가 아닌 NMEA0183 RMC/GGA 위도·경도·고도라고 명시한다.
따라서 `pyproj`로 WGS84를 MGeo의 `global_coordinate_system`으로 투영하고,
`global_info.json`의 `local_origin_in_global`을 빼서 경로 CSV와 같은 맵 원점
ENU 좌표로 만든다. 기본 K-City 데이터는 EPSG:32652이다.

공식 IMU UDP는 107 byte이며 quaternion 순서는 wire 기준 **w, x, y, z**이다.
각속도는 rad/s, 선가속도는 m/s²이다. 수신기는 길이, `#IMUData$`, data size
80, tail을 모두 검사하고 내부에서는 x,y,z,w 순서로 변환한다.

공식 [26.R1 UDP 프로토콜](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/26.R1/udp-1)을
기준으로 `Ego Ctrl Cmd`는 59 byte, `CollisionData`는 181 byte로 처리한다.
제어 패킷은 앞·뒤 조향 필드를 모두 포함하며 뒤 조향은 0으로 둔다.

`Competition Vehicle Status`는 `Ego Vehicle Status`와 별개다. 전용 공개 규격을
확인할 수 없어 별도 파서로 분리했다. 실제 25.01 환경에서 확인된 181-byte
형식(`data_length=152`)과 타이어 정보가 추가된 229-byte 확장 형식
(`data_length=200`)을 모두 지원한다. 그 외 길이·헤더·data length는 임의로
해석하지 않고 오류 로그에 실제 값을 출력한다.

## 설치

Python 3.8 이상에서 저장소 루트 기준으로 실행한다.

```bash
python3 -m pip install -r src/path_planning/requirements.txt
```

## 네트워크 설정

아래 포트는 실행 예시일 뿐이다. MORAI Network/Sensor Settings의 Destination
Port와 명령행 옵션을 반드시 동일하게 맞춘다.

| 항목 | 방향 | 기본 포트 |
|---|---|---:|
| GPS | SIM → 프로그램 | 9100 |
| IMU | SIM → 프로그램 | 9101 |
| Competition Vehicle Status | SIM → 프로그램 | 3315 |
| CollisionData | SIM → 프로그램 | 5678 |
| Ego Ctrl Cmd | 프로그램 → SIM | 9090 |

한 PC에서 실행하면 센서와 Publisher의 Destination IP, Ctrl Cmd의 Host IP를
보통 `127.0.0.1`로 둔다. 다른 PC/VM/WSL이면 프로그램이 실행되는 PC의 실제
IPv4 주소를 Destination IP로 사용한다. 네 수신 포트는 서로 달라야 한다.

## 실행

```bash
python3 src/path_planning/src/morai_stanley_udp.py \
  --path src/path_planning/data/morai_global_path.csv \
  --gps-port 9100 \
  --imu-port 9101 \
  --competition-status-port 3315 \
  --collision-port 5678 \
  --control-ip 127.0.0.1 \
  --control-port 9090 \
  --target-speed-kmh 20
```

다른 맵이면 그 맵의 MGeo 설정을 넘긴다.

```bash
python3 src/path_planning/src/morai_stanley_udp.py \
  --path /absolute/path/run.csv \
  --global-info /absolute/path/mgeo/global_info.json
```

GPS 또는 IMU가 timeout을 넘겨 끊기거나, 위치가 예측치에서 15 m 이상 튀거나,
충돌이 감지되거나, 목적지에 도달하거나, `Ctrl+C`가 입력되면 brake 명령을
보낸다.

## 현장에서 먼저 확인할 값

- `--max-steering-deg`: 선택 차량의 최대 앞바퀴 조향각. 기본값은 36.25°이다.
- `--morai-steer-sign`: 매우 낮은 속도에서 좌/우가 맞는지 확인한다. 반대면 1을 쓴다.
- `--control-point-offset`: 기본 0 m. GPS 센서 장착 위치에서 앞차축 중심까지의
  차량 전방 오프셋을 실측했을 때만 설정한다.
- `--imu-yaw-offset-deg`: 센서가 차량 정면과 돌아가 장착된 경우에만 보정한다.
- `--speed-kp`, `--speed-ki`, `--max-accel-pedal`, `--max-brake-pedal`: 먼저
  5–10 km/h에서 튜닝한다.
- `--stanley-gain`: 커지면 횡오차를 빠르게 줄이지만 GPS 노이즈에서 진동할 수 있다.

차량 위치 또는 GPS가 앞차축 중심인지 MORAI 공개 UDP 문서에는 명시되어 있지
않다. 따라서 wheelbase만으로 임의 보정하지 않는다.
