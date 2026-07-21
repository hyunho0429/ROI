# K-City 2025 Dijkstra Path Planning

This branch contains the K-City 2025 MGeo planner and the competition UDP
Pure Pursuit/PID controller with GPS/IMU EKF-INS localization.

## Build

```bash
cd ~/catkin_ws
sb
catkin_make
source devel/setup.bash
```

## Run

```bash
roslaunch path_planning kcity_2025_dijkstra.launch use_odom_start:=false
```

The launch shows a default example route from `A1256W000437` to `A1256W000531`.
In RViz, select `2D Nav Goal` and click another destination on the map to update the route. The planner chooses a reachable MGeo node near the clicked point, computes the shortest Dijkstra path from the start node, and publishes `/global_path` and `/global_path_marker`.

Useful topics:

```bash
rostopic echo /node
rostopic echo /link
rostopic echo /global_path
rostopic echo /global_path_marker
```

## MORAI keyboard path CSV recording

Record the reference path with the competition-allowed GPS UDP sensor while
driving manually. The recorder saves a point every 0.5 m by default:

```bash
python3 src/path_planning/src/morai_gps_csv_recorder.py \
  --bind-ip 0.0.0.0 --port 3001 \
  --output src/path_planning/data/morai_global_path.csv
```

The CSV contains raw latitude/longitude/altitude, derived map-local ENU, and the
fixed CRS/EastOffset/NorthOffset/UpOffset used for conversion. It deliberately
does not use Ego Vehicle Status or store historical IMU samples. See
`src/path_planning/README_GPS_CSV_RECORDER.md`.

## MORAI UDP Pure Pursuit control

The `dev/stanley` branch now runs a standalone Pure Pursuit controller with a
15-state GPS/IMU/Competition-speed-aided EKF-INS. It ports AutoVehicle's local
ENU CSV conversion and waypoint preprocessing without its ROS dependencies.
Install `src/path_planning/requirements.txt`, then run the recommended INS
runner:

The competition UDP values are defined in
`src/path_planning/src/path_planning/morai_competition_config.py`: GPS `3001`,
IMU `4001`, Competition Status `9080 -> 9081`, CollisionData `9091 -> 9092`,
and Ego Ctrl Cmd `9094 -> 192.168.0.170:9093`. The target speed is `10 km/h`.
These values do not need to be
repeated on the command line.

```bash
python3 src/path_planning/src/morai_pure_pursuit_ins_udp.py \
  --path src/path_planning/data/2026_molit_comp_global_path.txt
```

The same controller can be started through `roslaunch`. ROS is used only to
start the process and supply arguments; GPS, IMU, Competition Status,
CollisionData, and Ego Ctrl Cmd still use UDP only.

```bash
cd ~/catkin_ws
python3 -m pip install -r src/path_planning/requirements.txt
catkin_make
source devel/setup.bash
roslaunch path_planning morai_pure_pursuit_udp.launch
```

The launch defaults use the `main` branch longitudinal PID settings at 30 Hz:
`Kp=0.075`, `Ki=0.0001`, and `Kd=0.025`. Positive PID output is sent as
`accel`, negative output as `brake`, and Pure Pursuit supplies `steering` in
the same MORAI `longCmdType=1` packet. Network and controller values can be
overridden without editing code, for example:

```bash
roslaunch path_planning morai_pure_pursuit_udp.launch \
  control_ip:=192.168.0.170 target_speed_kmh:=10.0 \
  speed_kp:=0.075 speed_ki:=0.0001 speed_kd:=0.025
```

See `src/path_planning/README_PURE_PURSUIT_UDP.md` for the MORAI 25.S4 protocol basis,
coordinate conversion, network settings, safety behavior, and tuning values.

## 대회 UDP 주행 코드 실행 순서

아래 명령은 저장소를 `~/ROI`에 clone한 Ubuntu/ROS Noetic 환경을 기준으로 한다.
다른 위치에 clone했다면 `~/ROI`를 실제 저장소 경로로 바꾼다.

### 1. 브랜치 업데이트 및 최초 빌드

```bash
cd ~/ROI
git switch dev/stanley
git pull origin dev/stanley

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

### 2. MORAI 네트워크 설정

MORAI의 각 Destination IP에는 알고리즘을 실행하는 PC의 IPv4 주소를 입력한다.
Ego Ctrl Cmd의 전송 대상 IP만 MORAI 시뮬레이터 PC의 IPv4 주소이다.

| 네트워크 | 방향 | MORAI Host/Source Port | 알고리즘 Destination/Source Port |
|---|---|---:|---:|
| GPS | MORAI -> 알고리즘 | 센서 설정값 | 3001 |
| IMU | MORAI -> 알고리즘 | 센서 설정값 | 4001 |
| Competition Vehicle Status | MORAI -> 알고리즘 | 9080 | 9081 |
| CollisionData | MORAI -> 알고리즘 | 9091 | 9092 |
| Ego Ctrl Cmd | 알고리즘 -> MORAI | 9093 | 9094 |

알고리즘 PC의 IP는 다음 명령으로 확인할 수 있다.

```bash
hostname -I
```

### 3. Competition Vehicle Status 단독 확인

이 프로그램은 제어 명령을 보내지 않고 Competition Status만 수신한다. 메인 주행
프로그램도 Destination Port `9081`을 사용하므로 두 프로그램을 동시에 실행하지
않는다.

```bash
cd ~/ROI
source devel/setup.bash

python3 src/path_planning/src/morai_competition_status_inspect.py \
  --host-port 9080 \
  --destination-port 9081 \
  --count 10 \
  --hex-bytes 0
```

정상 수신 시 payload 크기, header, `ctrl_mode`, gear, 속도, accel/brake,
조향각, wheelbase, 위치, 자세, 각속도, 가속도 및 link ID가 출력된다. 현재 파서는
181-byte 기본 패킷과 229-byte 확장 패킷을 지원한다. `--hex-bytes 96`을 사용하면
패킷 앞부분만 출력할 수 있다.

`TIMEOUT`이 발생하면 MORAI Destination IP/Port와 로컬 bind 상태를 확인한다.

```bash
sudo ss -lunp | grep 9081
sudo tcpdump -ni any -s 0 -c 10 -XX \
  'udp src port 9080 and dst port 9081'
```

### 4. Ego Ctrl Cmd 안전 점검

Competition Status 확인 프로그램을 종료한 뒤, 안전한 brake 패킷을 보내 MORAI가
제어 명령을 수신하고 피드백하는지 확인한다. 먼저 `MORAI_PC_IP`에 시뮬레이터가
실행되는 PC의 실제 IPv4 주소를 넣는다.

```bash
cd ~/ROI
source devel/setup.bash
export MORAI_PC_IP=192.168.0.170

python3 src/path_planning/src/morai_udp_control_check.py \
  --control-ip "$MORAI_PC_IP"
```

정상이면 다음 메시지가 출력된다.

```text
PASS: MORAI reflected the longCmdType-1 brake command
```

점검 및 메인 코드는 `ctrl_mode=2`, `gear=4`, `longCmdType=1`을 전송한다.
`longCmdType=1`에서 PID 출력은 accel/brake로, Pure Pursuit 출력은 steering으로
전송된다.

### 5. 메인 Pure Pursuit + PID + EKF-INS 주행

Competition Status 확인 프로그램과 제어 점검 프로그램을 모두 종료한 뒤 실행한다.

```bash
cd ~/ROI
source devel/setup.bash
export MORAI_PC_IP=192.168.0.170

roslaunch path_planning morai_pure_pursuit_udp.launch \
  control_ip:="$MORAI_PC_IP"
```

예를 들어 MORAI PC IP가 `192.168.0.170`이면 다음과 같다.

```bash
roslaunch path_planning morai_pure_pursuit_udp.launch \
  control_ip:=192.168.0.170
```

MORAI와 알고리즘을 같은 PC에서 실행할 때는 `control_ip:=127.0.0.1`을 사용할 수
있다. 기본 경로는 `2026_molit_comp_global_path.txt`, 목표 속도는 `10 km/h`이다.
목표 속도를 변경하려면 다음과 같이 실행한다.

```bash
roslaunch path_planning morai_pure_pursuit_udp.launch \
  control_ip:=192.168.0.170 \
  target_speed_kmh:=8.0
```

정상 시작 시 다음 로그를 확인한다.

```text
localization: GPS/IMU/status-aided 15-state error-state EKF INS
requesting AV-ExternalCtrl (ctrl_mode=2) and Drive (gear=4)
Competition control state: ctrl_mode=2 (AV-ExternalCtrl), gear=4 (D)
```

안전을 위해 GPS, IMU, Competition Status 중 필요한 입력이 준비되지 않았거나
Competition Status가 `ctrl_mode=2`, `gear=4`를 회신하지 않으면 가속하지 않고
brake 명령을 유지한다. 이 상태가 계속되면 Cmd Control IP/Port, 차량 제어 모드,
기어 및 UDP 방화벽을 확인한다.

For comparison, the speed-aided dead-reckoning alternative remains available:

```bash
# Competition-speed-aided dead reckoning
python3 src/path_planning/src/morai_pure_pursuit_dead_reckoning_udp.py \
  --path src/path_planning/data/2026_molit_comp_global_path.txt
```

Both continue through a configurable GPS outage while IMU and Competition
Vehicle Status remain fresh. See `src/path_planning/README_TUNNEL_LOCALIZATION.md`.

For a fixed start and goal node:

```bash
roslaunch path_planning kcity_2025_dijkstra.launch \
  interactive:=false \
  use_odom_start:=false \
  start_node:=A1256W000437 \
  goal_node:=A1256W000531
```
