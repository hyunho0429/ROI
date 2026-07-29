# MORAI 대회 자율주행

`dev/stanley` 브랜치는 MORAI 25.S4 대회 환경에서 전역 경로를 추종하기 위한
UDP 기반 자율주행 코드이다.

- 횡방향 제어: Pure Pursuit
- 종방향 제어: PID
- 위치 추정: GPS/IMU 기반 15상태 오차 상태 EKF-INS
- 속도 보조: Competition Vehicle Status의 signed velocity
- 제어 통신: MORAI Ego Ctrl Cmd UDP
- 안전 기능: 센서 stale, 충돌, 잘못된 제어 모드 및 기어 감지 시 제동

ROS는 프로세스 실행과 인자 전달에만 사용한다. GPS, IMU, Competition Vehicle
Status, CollisionData 및 Ego Ctrl Cmd는 모두 UDP로 통신한다.

## 기본 제어 설정

메인 코드는 다음 형식의 제어 패킷을 전송한다.

| 항목 | 기본값 | 설명 |
|---|---:|---|
| `ctrl_mode` | 2 | 외부 자율주행 제어 모드 |
| `gear` | 4 | Drive |
| `longCmdType` | 1 | accel/brake 직접 제어 |
| 제어 주기 | 30 Hz | PID 및 Pure Pursuit 계산 주기 |
| 목표 속도 | 30 km/h | launch 인자로 변경 가능 |
| 차량 모델 | 2023 Hyundai IONIQ 5 | 대회 규정집 기준 |
| wheelbase | 3.0 m | IONIQ 5 축간거리 |
| 차폭 | 1.892 m | IONIQ 5 차폭 |
| PID Kp | 0.075 | 종방향 비례 이득 |
| PID Ki | 0.0001 | 종방향 적분 이득 |
| PID Kd | 0.025 | 종방향 미분 이득 |

PID의 양수 출력은 `accel`, 음수 출력은 `brake`로 분리한다. Pure Pursuit가
계산한 조향값은 같은 `longCmdType=1` 패킷의 `steering` 필드로 전송한다.

## 네트워크 설정

기본값은
`src/path_planning/src/path_planning/morai_competition_config.py`에 정의되어
있다.

| 네트워크 | 방향 | MORAI Host/Source Port | MORAI Destination Port |
|---|---|---:|---:|
| GPS | MORAI → 알고리즘 | 센서 설정값 | 3001 |
| IMU | MORAI → 알고리즘 | 센서 설정값 | 4001 |
| Competition Vehicle Status | MORAI → 알고리즘 | 9080 | 9081 |
| CollisionData | MORAI → 알고리즘 | 9091 | 9092 |
| 3D LiDAR Intensity | MORAI → 알고리즘 | 2000 | 2001 |
| Ego Ctrl Cmd | 알고리즘 → MORAI | 9093 | 9094 |

현재 네트워크 기준 MORAI Host IP는 `192.168.56.1`, 알고리즘 PC Destination IP는
`192.168.56.101`이다. MORAI에서 GPS, IMU, Competition Vehicle Status 및
CollisionData의 Destination IP는 `192.168.56.101`로 설정한다. Ego Ctrl Cmd의
`control_ip`에는 MORAI 시뮬레이터가 실행되는 `192.168.56.1`을 사용한다.

알고리즘 PC의 IP는 다음 명령으로 확인할 수 있다.

```bash
hostname -I
```

## 1. 저장소 받기

다른 컴퓨터에서 처음 받는 경우:

```bash
cd ~
git clone -b dev/stanley --single-branch https://github.com/hyunho0429/ROI.git
cd ROI
```

이미 저장소가 있는 경우:

```bash
cd ~/ROI
git fetch origin
git switch dev/stanley
git pull origin dev/stanley
```

구버전 Git에서 `git switch`가 지원되지 않으면 `git checkout dev/stanley`를
사용한다.

## 2. 최초 빌드

저장소 루트가 catkin workspace이다.

```bash
cd ~/ROI
source /opt/ros/noetic/setup.bash
python3 -m pip install -r src/path_planning/requirements.txt
catkin_make
source devel/setup.bash
```

새 터미널을 열 때마다 다음 환경 설정을 다시 실행한다.

```bash
cd ~/ROI
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## 3. Competition Vehicle Status 정보 확인

다음 프로그램은 제어 명령을 보내지 않고 Competition Vehicle Status만 수신한다.

```bash
cd ~/ROI
source devel/setup.bash

python3 src/path_planning/src/morai_competition_status_inspect.py \
  --host-port 9080 \
  --destination-port 9081 \
  --count 10 \
  --hex-bytes 0
```

정상 수신 시 다음 정보가 출력된다.

- 송신 IP와 source port
- UDP payload 크기, header 및 data length
- `ctrl_mode`, gear 및 signed velocity
- accel/brake 페달 피드백
- 차량 크기, wheelbase 및 overhang
- 위치, 자세, 속도, 각속도 및 가속도
- 조향각과 link ID
- 원본 패킷 hexadecimal 값

현재 파서는 181-byte 기본 패킷과 229-byte 확장 패킷을 지원한다.
`--hex-bytes 96`을 사용하면 원본 패킷 앞부분만 출력한다.

이 프로그램과 메인 주행 코드는 모두 Destination Port `9081`을 사용하므로
동시에 실행하지 않는다.

패킷이 수신되지 않으면 다음 명령으로 포트와 원본 패킷을 확인한다.

```bash
sudo ss -lunp | grep 9081

sudo tcpdump -ni any -s 0 -c 10 -XX \
  'udp src port 9080 and dst port 9081'
```

## 4. 3D LiDAR Intensity UDP 확인

MORAI 3D LiDAR UDP는 Velodyne 프로토콜을 따른다. Python `socket.recvfrom()`
기준으로는 Ethernet/IP/UDP 42 byte header가 제외된 `1206 bytes` payload가
수신된다. 구조는 `12개 data block x 100 bytes = 1200 bytes`와
`timestamp 4 bytes + factory/status 2 bytes = 6 bytes`이다.

각 data block은 `flag 2 bytes`, `azimuth 2 bytes`, 그리고 `32개 channel data`
로 구성된다. 각 channel data는 `distance uint16 2 bytes + reflectivity/intensity
uint8 1 byte`이다.

```bash
cd ~/ROI
source devel/setup.bash

python3 src/path_planning/src/morai_lidar_intensity_inspect.py \
  --host-port 2000 \
  --destination-port 2001 \
  --count 10 \
  --sample-points 5
```

수신이 정상이라면 각 UDP packet마다 sender, payload 크기, Velodyne block layout,
timestamp/factory, point 수, 거리/intensity 범위, sample point가 출력된다.
일반 UDP 수신은 `--header-bytes auto` 또는 `--header-bytes 0`을 쓰면 되고,
pcap 등에서 42 byte network header까지 포함된 데이터를 직접 넣을 때만
`--header-bytes 42`를 사용한다.

CSV로 일부 point를 저장해서 확인하려면:

```bash
python3 src/path_planning/src/morai_lidar_intensity_inspect.py \
  --host-port 2000 \
  --destination-port 2001 \
  --count 3 \
  --dump-csv /tmp/morai_lidar_points.csv
```

## 5. Ego Ctrl Cmd 통신 점검

Competition Status 확인 프로그램을 종료한 뒤 실행한다. `MORAI_PC_IP`에는
시뮬레이터가 실행되는 PC의 실제 IPv4 주소를 입력한다.

```bash
cd ~/ROI
source devel/setup.bash
export MORAI_PC_IP=192.168.56.1

python3 src/path_planning/src/morai_udp_control_check.py \
  --control-ip "$MORAI_PC_IP"
```

이 프로그램은 안전한 brake 명령을 전송하고 Competition Vehicle Status의
피드백을 확인한다. 정상이라면 다음 메시지가 출력된다.

```text
PASS: MORAI reflected the longCmdType-1 brake command
```

점검 패킷은 `ctrl_mode=2`, `gear=4`, `longCmdType=1`, `accel=0`으로
전송된다. 빈 공간에서 실제 가속 통신까지 확인해야 할 때만 `--drive-test`를
추가한다.

## 6. 메인 주행 코드 실행

Competition Status 확인 프로그램과 제어 점검 프로그램을 모두 종료한 뒤 실행한다.

```bash
cd ~/ROI
source devel/setup.bash
export MORAI_PC_IP=192.168.56.1

roslaunch path_planning morai_stanley_udp.launch \
  control_ip:="$MORAI_PC_IP"
```

MORAI와 알고리즘을 같은 PC에서 실행하는 경우:

```bash
roslaunch path_planning morai_stanley_udp.launch \
  control_ip:=127.0.0.1
```

기본 전역 경로는
`src/path_planning/data/2026_molit_comp_global_path.txt`이고 목표 속도는
`30 km/h`이다.

목표 속도와 PID 이득을 변경하는 예:

```bash
roslaunch path_planning morai_stanley_udp.launch \
  control_ip:=192.168.56.1 \
  target_speed_kmh:=8.0 \
  speed_kp:=0.075 \
  speed_ki:=0.0001 \
  speed_kd:=0.025
```

다른 경로 파일을 사용하는 예:

```bash
roslaunch path_planning morai_stanley_udp.launch \
  control_ip:=192.168.56.1 \
  path:=/home/ubuntu/path/reference_path.csv
```

## 정상 실행 로그

정상 시작 시 다음과 비슷한 로그가 출력된다.

```text
localization: GPS/IMU/status-aided 15-state error-state EKF INS
alignment: hold brake for 2.0s (at least 20 IMU samples)
Pure Pursuit: Ld=clip(2.00+0.15*speed, 2.00, 4.00)m, wheelbase=3.00m, fixed speed 30.0 km/h
steering smoothing: alpha=0.15, max_rate=0.35 rad/s
requesting AV-ExternalCtrl (ctrl_mode=2) and Drive (gear=4)
Competition control state: ctrl_mode=2 (AV-ExternalCtrl), gear=4 (D)
```

안전을 위해 다음 상황에서는 가속하지 않고 brake 명령을 유지한다.

- GPS, IMU 또는 Competition Status가 아직 수신되지 않은 경우
- 필요한 센서 데이터가 stale 상태인 경우
- Competition Status가 `ctrl_mode=2`, `gear=4`를 회신하지 않는 경우
- CollisionData에서 충돌이 검출된 경우
- 전역 경로의 마지막 지점에 도달한 경우

종료할 때는 `Ctrl+C`를 누른다. 종료 과정에서도 안전 정지 패킷을 전송한다.

## 위치 추정 방식

1. GPS UDP에서 NMEA0183 RMC/GGA 문장을 수신한다.
2. 위도, 경도, 고도를 경로와 동일한 map-local ENU 좌표로 변환한다.
3. 초기 정렬 동안 brake를 유지하며 IMU 자세와 바이어스를 초기화한다.
4. IMU 각속도와 선형가속도로 INS mechanization을 수행한다.
5. GPS 위치와 Competition Status 속도를 이용해 15상태 오차 상태 EKF를 보정한다.
6. 추정된 ENU 위치, yaw 및 속도를 Pure Pursuit와 PID에 전달한다.

GPS가 일시적으로 끊겨도 설정된 `max_gps_outage` 동안 IMU와 Competition Status
속도를 사용해 추측 항법을 계속한다. 자세한 내용은
`src/path_planning/README_TUNNEL_LOCALIZATION.md`를 참고한다.

## 기준 경로 기록

MORAI 차량을 수동으로 움직이며 GPS 기준점을 CSV로 기록할 수 있다.

```bash
cd ~/ROI
source devel/setup.bash

python3 src/path_planning/src/morai_gps_csv_recorder.py \
  --bind-ip 0.0.0.0 \
  --port 3001 \
  --output src/path_planning/data/morai_global_path.csv
```

CSV에는 원본 위도, 경도, 고도와 변환된 map-local ENU, CRS 및 고정 offset 정보가
저장된다. 기본적으로 차량이 0.5 m 이상 이동할 때 새로운 점을 저장한다.

## Python으로 직접 실행

`roslaunch` 사용을 권장하지만 다음과 같이 직접 실행할 수도 있다.

```bash
cd ~/ROI
python3 -m pip install -r src/path_planning/requirements.txt

python3 src/path_planning/src/morai_stanley_ins_udp.py \
  --path src/path_planning/data/2026_molit_comp_global_path.txt \
  --control-ip 192.168.56.1
```

Competition 속도 보조 추측 항법 실행기는 다음과 같다.

```bash
python3 src/path_planning/src/morai_stanley_dead_reckoning_udp.py \
  --path src/path_planning/data/2026_molit_comp_global_path.txt \
  --control-ip 192.168.56.1
```

## 문제 확인

### Competition Status TIMEOUT

- MORAI Destination IP가 알고리즘 PC IP인지 확인한다.
- Host/Source Port가 `9080`, Destination Port가 `9081`인지 확인한다.
- 메인 주행 코드나 다른 프로세스가 `9081`을 사용 중인지 확인한다.
- 방화벽에서 UDP 수신을 허용했는지 확인한다.

### 제어 명령이 반영되지 않음

- `control_ip`가 MORAI PC IP인지 확인한다.
- Ego Ctrl Cmd Host Port가 `9093`, Destination/Source Port가 `9094`인지
  확인한다.
- 주행 로그의 명령값과 Competition Status의 accel/brake/steer 피드백을 비교한다.
- 차량이 `ctrl_mode=2`, `gear=4` 상태인지 확인한다.

### 센서 대기 상태에서 주행하지 않음

- GPS `3001`, IMU `4001`, Competition Status `9081` 수신 여부를 확인한다.
- 초기 정렬에는 기본 2초와 최소 20개의 IMU 샘플이 필요하다.
- 경로 파일과 `global_info.json`의 좌표계 및 offset이 일치하는지 확인한다.

## 보조 경로 계획 및 RViz 실행

MGeo 다익스트라 경로 계획과 RViz 시각화가 필요한 경우 다음 명령을 사용한다.

```bash
roslaunch path_planning kcity_2025_dijkstra.launch use_odom_start:=false
```

고정 시작 노드와 도착 노드를 지정하는 예:

```bash
roslaunch path_planning kcity_2025_dijkstra.launch \
  interactive:=false \
  use_odom_start:=false \
  start_node:=A1256W000437 \
  goal_node:=A1256W000531
```

## 상세 문서

- `src/path_planning/README_PURE_PURSUIT_UDP.md`: UDP 패킷, 좌표 변환, 제어 및
  튜닝 상세
- `src/path_planning/README_TUNNEL_LOCALIZATION.md`: GPS 음영 구간 위치 추정
- `src/path_planning/README_GPS_CSV_RECORDER.md`: GPS 기준 경로 기록
