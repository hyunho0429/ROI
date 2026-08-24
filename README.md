# MORAI 대회 자율주행

`dev/merged_sensor` 브랜치는 MORAI 25.S4 대회 환경에서 전역 경로를 추종하는
UDP 기반 자율주행 코드에 LiDAR와 카메라 인식을 통합한 브랜치이다.

- 횡방향 제어: Pure Pursuit
- 종방향 제어: PID
- 위치 추정: GPS/IMU 기반 15상태 오차 상태 EKF-INS
- 속도 보조: Competition Vehicle Status의 signed velocity
- 제어 통신: MORAI Ego Ctrl Cmd UDP
- 안전 기능: 센서 stale, 충돌, 잘못된 제어 모드 및 기어 감지 시 제동

ROS는 프로세스 실행과 인자 전달에만 사용한다. GPS, IMU, Competition Vehicle
Status, CollisionData 및 Ego Ctrl Cmd는 모두 UDP로 통신한다.

Pure Pursuit UDP·EKF 주행 스택과 LiDAR의 Euclidean clustering, 3D bounding
box, Kalman+Hungarian tracking 및 끼어들기 공간 판단 통합 실행법은
[`docs/LIDAR_PURE_PURSUIT_INTEGRATION.md`](docs/LIDAR_PURE_PURSUIT_INTEGRATION.md)를
참고한다.

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

아래의 `192.168.56.x` 값은 기존 단독 Python 실행 예제의 네트워크 구성이다.
통합 launch는 `feat/lidar` 설정을 이어받아 로컬 센서 수신에는
`bind_ip=0.0.0.0`, 기본 제어 목적지에는 `morai_host_ip=192.168.0.148`을 사용한다.
MORAI 센서의 Destination IP는 기존 `feat/lidar`에서 사용하던 Ubuntu PC의 실제
IP를 그대로 유지하면 되며 통합 launch 명령에 로컬 IP를 따로 넣지 않는다.

알고리즘 PC의 IP는 다음 명령으로 확인할 수 있다.

```bash
hostname -I
```

## 1. 저장소 받기

다른 컴퓨터에서 처음 받는 경우:

```bash
cd ~
git clone -b dev/merged_sensor --single-branch https://github.com/hyunho0429/ROI.git
cd ROI
```

이미 저장소가 있는 경우:

```bash
cd ~/ROI
git fetch origin
git switch dev/merged_sensor
git pull origin dev/merged_sensor
```

구버전 Git에서 `git switch`가 지원되지 않으면 `git checkout dev/merged_sensor`를
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

모든 해석 필드를 한 번에 확인하려면 전용 UDP 로거를 실행한다. 기본적으로 최신
상태를 1초마다 한 블록으로 계속 출력하며 차량 제어 명령은 전송하지 않는다.

```bash
cd ~/ROI
source devel/setup.bash

python3 src/path_planning/src/morai_competition_vehicle_status_logger.py \
  --host-port 9080 \
  --destination-port 9081 \
  --interval 1.0
```

한 번만 출력하고 종료하려면 `--count 1`, 모든 수신 패킷을 출력하려면
`--interval 0`을 사용한다. 출력에는 control mode/gear, 페달과 조향각, 위치와
자세, 속도·각속도·가속도, 차량 크기와 wheelbase/overhang, map/link ID 및 확장
패킷의 타이어 lateral force/side slip/cornering stiffness가 포함된다.

Destination Port `9081`은 한 프로세스만 사용해야 하므로 Stanley 주행 코드나 기존
상태 점검기를 종료한 뒤 실행한다.

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

이 브랜치의 LiDAR parser는 VLP-16 수직각 테이블을 사용한다. 공식 문서의 원시
Velodyne LiDAR 센서 좌표계는 `x=right`, `y=forward`, `z=up`이고, 전방 장애물
판정과 로그의 기본 좌표는 차량 로컬 좌표계 `x_forward=forward`, `y_left=left`,
`z_up=up`으로 변환해서 사용한다. 변환식은 다음과 같다.

```text
vehicle_x_forward = raw_y_forward
vehicle_y_left    = -raw_x_right
vehicle_z_up      = raw_z_up
```

시뮬레이터 차량 위치 `(x, y, z)`는 맵 원점 기준 ENU 전역 좌표다. 이 값은 LiDAR
로컬 point를 맵 전역 좌표로 변환할 때 차량 위치/yaw와 함께 사용해야 하며,
이 inspector의 전방 장애물 판정은 차량 로컬 좌표만 사용한다.

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
timestamp/factory, point 수, 거리/intensity 범위, 전방 장애물 후보, sample point가 출력된다.
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

전방 장애물 감지는 여러 UDP packet을 누적해서 확인하는 `lidar_monitor.py`를
사용한다. 큰 박스를 설치했는데 `none`이 나오면 먼저 `--yaw-scan`으로 LiDAR
장착 yaw 방향이 맞는지 확인한다.

```bash
PYTHONPATH=src/path_planning/src python3 src/path_planning/src/lidar_monitor.py \
  --bind-ip 0.0.0.0 \
  --host-port 2000 \
  --destination-port 2001 \
  --packets-per-window 80 \
  --fov-left-deg 90 \
  --fov-right-deg 90 \
  --yaw-scan
```

정상적으로 장애물 후보가 잡히면 `front_points`, `object_like`, `front nearest`,
`object-like nearest`가 출력된다. 특정 yaw offset에서만 후보가 많이 잡히면
그 값을 다음 실행에 적용한다.

```bash
PYTHONPATH=src/path_planning/src python3 src/path_planning/src/lidar_monitor.py \
  --bind-ip 0.0.0.0 \
  --host-port 2000 \
  --destination-port 2001 \
  --packets-per-window 80 \
  --fov-left-deg 90 \
  --fov-right-deg 90 \
  --lidar-yaw-offset-deg 90
```

RViz에서 LiDAR PointCloud를 바로 확인하려면 다음 launch를 사용한다. 이 launch는
UDP LiDAR 수신 node와 RViz를 함께 실행하고, 기본값으로 MORAI 공식 문서의 RViz
scan 화면처럼 거의 360도 영역을 publish하되 차량 바로 뒤쪽 blind sector만 제외한다.

```bash
roslaunch path_planning morai_lidar_rviz.launch
```

위 launch의 기본값은 `bind_ip=0.0.0.0`, `host_port=2000`,
`destination_port=2001`, `rear_blind_deg=60`, `rolling_clouds=1`,
`display_rolling_clouds=1`, `display_history_s=0.0`으로 설정되어 있다. 방위각의
360도 경계뿐 아니라 기본 15개 패킷 누적 또는 0.05초 경과 시에도 새 PointCloud를
발행하므로 패킷 유실로 경계를 건너뛰더라도 RViz 화면이 첫 스캔에 멈추지 않는다.
LiDAR node와 RViz는 함께 실행된다.

현재 LiDAR 단독 개발용 기본값은 `deskew_pose_source:=sensor_udp`이다. 따라서
LiDAR node가 MORAI GPS `3001`과 IMU `4001`을 직접 수신하고 자세를 계산하므로
Stanley 주행 코드를 함께 실행할 필요가 없다.

```bash
# LiDAR 2001 + GPS 3001 + IMU 4001 직접 수신, ground removal, deskew, RViz
roslaunch path_planning morai_lidar_rviz.launch
```

나중에 Stanley와 LiDAR를 동시에 실행할 때는 두 프로세스가 `3001/4001`을 함께
bind하지 않도록 Stanley만 센서를 수신하고 LiDAR는 융합 자세 UDP `4012`를 받는
모드로 변경한다.

```bash
# terminal 1
roslaunch path_planning morai_stanley_udp.launch \
  control_ip:=192.168.0.148

# terminal 2
roslaunch path_planning morai_lidar_rviz.launch \
  deskew_pose_source:=fused_pose_udp
```

LiDAR만 정지 상태에서 확인할 때는 `deskew_enabled:=false`를 지정할 수 있다. 필요한
GPS/IMU 또는 융합 자세 UDP가 아직 준비되지 않았으면 LiDAR node는 중단되지 않고
원본 점군으로 자동 fallback한다.

차량 뒤쪽도 포함한 완전 360도에 가깝게 보고 싶으면 `rear_blind_deg:=0`으로 둔다.

```bash
roslaunch path_planning morai_lidar_rviz.launch \
  fov_left_deg:=180 \
  fov_right_deg:=180 \
  rear_blind_deg:=0
```

전방 특정 영역만 보고 싶으면 좌우 각도를 줄인다.

```bash
roslaunch path_planning morai_lidar_rviz.launch \
  fov_left_deg:=45 \
  fov_right_deg:=45 \
  rear_blind_deg:=0
```

주행 알고리즘에는 `/morai/lidar/live_points`를 사용한다. 이 토픽은 기본
`rolling_clouds:=1`이라 최신 cloud만 publish한다. RViz는
`/morai/lidar/display_points`를 사용하며, 주행 중 과거 scan의 잔상이 장애물처럼
보이지 않도록 기본 `display_rolling_clouds:=1`, `display_history_s:=0.0`, RViz
`Decay Time:=0`을 사용한다.
PointCloud2에는 `x`, `y`, `z`, `distance_m`, `intensity`, `ring`,
`bearing_deg` 필드가 들어간다. RViz 색상은 기본적으로 `distance_m` 기준이다.
현재 샘플링 영역에서 가장 가까운 거리값은 다음 토픽으로도 확인할 수 있다.

```bash
rostopic echo /morai/lidar/nearest_distance_m
```

`nearest_distance_m`는 전체 point 중 최솟값이 아니라 전방 장애물 후보 영역
`nearest_x_min_m=1.0`, `nearest_x_max_m=40.0`, `nearest_y_abs_m=3.0`,
`nearest_z_min_m=-1.4`, `nearest_z_max_m=2.5` 안의 최단거리다. 전체 최솟값을
그대로 쓰면 지면/차량 근처 return 때문에 약 `0.5 m` 근처 값이 계속 나올 수 있다.
가까운 박스까지 거리를 더 민감하게 보고 싶으면 `nearest_x_min_m`을 낮추고,
바닥 point가 섞이면 `nearest_z_min_m`을 높인다.

장애물 군집은 grid 기반 Euclidean clustering으로 계산한다. 결과는 JSON 문자열로
`/morai/lidar/obstacles`에 publish되고, RViz에는 `/morai/lidar/obstacle_markers`
MarkerArray로 박스와 라벨이 표시된다. 각 군집에는 차량 기준 중심 거리, 가장 가까운
점 거리, 각도, 중심점, bounding box, point 수가 포함된다.

```bash
rostopic echo /morai/lidar/obstacles
```

주요 조정 파라미터:

```bash
roslaunch path_planning morai_lidar_rviz.launch \
  cluster_tolerance_m:=0.8 \
  cluster_min_points:=3 \
  cluster_min_height_m:=0.15 \
  cluster_x_min_m:=1.0 \
  cluster_x_max_m:=40.0 \
  cluster_y_abs_m:=5.0 \
  cluster_z_min_m:=-1.4 \
  cluster_z_max_m:=2.5
```

장애물이 여러 개로 쪼개져 보이면 `cluster_tolerance_m`을 키우고, 바닥/노이즈가
군집으로 잡히면 `cluster_min_points`를 키우거나 `cluster_z_min_m`을 높인다.

바닥을 맞힌 단일 LiDAR ring이 원호 또는 동심원처럼 보이는 점은 기본
`vertical_support_enabled:=true` 필터에서 제거한다. 같은 수평 위치 반경 `0.65 m`
안에 다른 ring의 점이 있으면서 높이 차이가 `0.05 m` 이상이어야 실제 장애물
후보로 유지된다. 이 필터는 nearest distance, clustering 및 RViz 표시 점군에 모두
적용된다.

```bash
roslaunch path_planning morai_lidar_rviz.launch \
  vertical_support_radius_m:=0.65 \
  vertical_support_min_height_m:=0.05
```

먼 장애물이 너무 많이 사라지면 `vertical_support_radius_m`을 `0.8` 정도로
늘린다. 원본 점군은 유지하고 장애물 판정에서만 원호를 제외하려면
`vertical_support_filter_cloud:=false`를 사용한다.

### 차량 크기 기반 인접 차선 공간 확인

Competition Vehicle Status에서 확인한 ego 크기 `xyz=(4.635, 1.892, 2.434)m`를
각각 길이, 폭, 높이로 사용한다. 기본 차선 폭 `3.5m`에서 좌우 인접 차선의 LiDAR
클러스터를 찾고 현재 ego 위치를 포함하는 전후 빈 구간을 계산한다.

기본 종방향 여유는 앞뒤 각각 `1.0m`이므로 필요한 물리적 빈 공간은
`4.635 + 1.0 + 1.0 = 6.635m`이다. 차선 한쪽의 횡방향 여유는
`(3.5 - 1.892) / 2 = 0.804m`이고 기본 최소 조건은 `0.2m`이다. 앞이나 뒤에
장애물 클러스터가 없으면 해당 방향의 `40m` LiDAR 확인 경계를 빈 구간 경계로
사용한다.

```bash
roslaunch path_planning morai_lidar_rviz.launch \
  merge_gap_enabled:=true \
  ego_size_x_m:=4.635 \
  ego_size_y_m:=1.892 \
  ego_size_z_m:=2.434 \
  adjacent_lane_width_m:=3.5
```

좌우 상태는 조건 충족 여부와 관계없이 1초마다 항상 출력된다.

```text
MERGE_GAP GEOMETRY_ONLY | LEFT=AVAILABLE reason=available gap=... | RIGHT=BLOCKED reason=obstacle_alongside gap=...
```

- `CHECKING(1/3)`: 공간 조건을 만족했지만 연속 scan 확인 중
- `AVAILABLE`: 기본 3 scan 연속으로 물리적 공간 조건 충족
- `BLOCKED`: 차량이 옆에 있거나 앞뒤/차선 폭 여유가 부족함
- `range_limit`: 해당 방향 `40m` 안에서 장애물이 검출되지 않아 확인 경계를 사용함
- `obstacle`: 실제 장애물 cluster 표면을 공간 경계로 사용함

이 결과는 차체가 들어갈 수 있는 **기하학적 공간 확인**일 뿐이다. 고속도로 차선
변경 제어에는 앞뒤 차량의 상대속도, TTC, 차선 곡률 및 실제 차선 변경 궤적 검증이
추가로 필요하다.

LiDAR 회전 속도는 30 Hz를 그대로 권장한다. 20 Hz로 낮추면 한 회전에 걸리는 시간이
길어져 동일한 차량 속도와 yaw rate에서 보상해야 할 왜곡량이 오히려 커진다. node는
azimuth가 360도에서 0도로 넘어가는 시점에 한 회전을 완성하고, 각 UDP 패킷 수신
시각의 EKF 위치/yaw를 보간한 뒤 마지막 패킷의 차량 좌표계로 변환한다.

`display_history_s`나 RViz PointCloud2의 `Decay Time`을 키우면 deskew 여부와
무관하게 서로 다른 시각의 ego-frame scan이 겹쳐 긴 무지개 궤적처럼 보일 수 있다.

RViz의 `Ego Axes`에서 빨간 X축은 차량 전방, 초록 Y축은 차량 좌측, 파란 Z축은
위쪽이다. 차량이 전진할 때 화면상 좌우로 움직이는 듯 보이면 먼저 RViz 카메라
시점 문제인지 이 축 기준으로 확인한다.

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

## LiDAR + 카메라 통합 실행

### 기능

`dev/merged_sensor`는 `feat/lidar`의 주행 및 LiDAR 코드를 그대로 유지하면서
`feature-camera`의 카메라 기능을 ROS launch로 함께 실행한다.

- GPS/IMU 기반 위치 추정과 Pure Pursuit 경로 추종
- PID 종방향 제어 및 MORAI Ego Ctrl Cmd UDP 송신
- 3D LiDAR UDP 수신, clustering, bounding box, Kalman+Hungarian tracking
- 고속도로 카메라 조건이 활성화될 때만 LiDAR 왼쪽 끼어들기 공간을 RViz에 표시
- 왼쪽 끼어들기 가능 여부와 map 장애물 목록을 ROS 메시지로 발행
- 전방 카메라 영상의 ONNX 차선 segmentation 및 차선 후보 추적
- YOLOv8 기본 객체 탐지
- 커스텀 모델이 있을 때 신호등과 MORAI 장애물 탐지 결과 중첩
- YOLO person 단독 인식 기반 보행자 즉시 정지 및 재출발
- LiDAR/RViz, 차선 인식, YOLO 화면을 하나의 `roslaunch`로 실행

차선과 YOLO는 각각 독립 프로세스로 실행된다. 한 카메라 프로세스에 문제가 생겨도
기존 주행 및 LiDAR 코드를 직접 변경하거나 같은 프로세스에서 종료시키지 않는다.
기존 LiDAR 전용 launch도 그대로 남아 있으므로 종전 실행법을 계속 사용할 수 있다.

### 코드 구조

```text
ROI/
├── Sensor/                         # feature-camera 원본 카메라 기능
│   ├── LaneCandidates.py           # 차선 segmentation, 후보 추출 및 화면 표시
│   ├── YoloCamera_v2.py            # 기본/커스텀 YOLO 탐지 및 화면 표시
│   ├── CameraUDP.py                 # 최신 완성 프레임만 유지하는 저지연 수신기
│   └── lane_segmentation.onnx      # 차선 segmentation 모델
├── lib/
│   ├── define/Camera.py            # MORAI 카메라 UDP 데이터 구조/파싱
│   └── network/UDP.py              # 카메라 UDP Receiver
├── src/
│   ├── bringup/morai_bringup/
│   │   └── launch/
│   │       ├── morai_udp_ekf_purepursuit_lidar_tracking.launch
│   │       │                       # 기존 feat/lidar 통합 launch
│   │       └── morai_udp_ekf_purepursuit_lidar_camera.launch
│   │                               # 주행+LiDAR+카메라 최상위 launch
│   ├── perception/lidar_perception/
│   │                               # 기존 LiDAR 인식 노드와 RViz 설정
│   ├── perception/camera_perception/
│   │   ├── launch/camera_perception.launch
│   │   └── scripts/
│   │       ├── camera_feature_runner.py
│   │       ├── highway_environment_gate_node.py
│   │       └── pedestrian_crossing_fusion_node.py
│   │                               # 고속도로 게이트와 보행자 카메라/LiDAR 융합
│   ├── path_planning/              # UDP 주행, 경로 추종 및 LiDAR 연동
│   ├── localization/               # GPS/IMU/EKF 위치 추정
│   └── control/                    # Pure Pursuit 관련 패키지
└── docs/LIDAR_CAMERA_INTEGRATION.md
```

최상위 launch의 실행 관계는 다음과 같다.

```text
morai_udp_ekf_purepursuit_lidar_camera.launch
├── morai_udp_ekf_purepursuit_lidar_tracking.launch
│   ├── 기존 GPS/IMU/EKF/Pure Pursuit 주행 스택
│   └── 기존 LiDAR tracking + merge-gap + RViz
└── camera_perception.launch
    ├── lane_camera  → Sensor/LaneCandidates.py
    ├── yolo_camera  → Sensor/YoloCamera_v2.py
    ├── highway_environment_gate
    │    car_detected AND (점선 조건, 현재 비활성) → merge-gap 활성화
    └── pedestrian_crossing_fusion
         person_detected AND 가까운 동적 LiDAR 객체 → 정지/재출발
```

### 센서 및 모델 준비

MORAI의 센서 목적지 IP는 기존 `feat/lidar`에서 사용하던 ROS Ubuntu PC의 실제
IP를 그대로 사용한다. 프로그램은 LiDAR와 카메라 모두 동일한 로컬 수신 주소
`bind_ip=0.0.0.0`에 bind하므로 실행 명령에서 IP를 따로 지정할 필요가 없다.

| 기능 | 기본 수신 포트 | 비고 |
|---|---:|---|
| GPS | `3001` | 기존 주행 스택 |
| IMU | `4001` | 기존 주행 스택 |
| Competition Vehicle Status | Host `9080` → Destination `9081` | 대회용 차량 상태, `/Ego_topic`으로 호환 변환 |
| LiDAR | `2001` | RViz/tracking 입력 |
| 차선 카메라 | `1101` | `LaneCandidates.py` |
| YOLO 카메라 | `1131` | `YoloCamera_v2.py` |

MORAI 센서 설정의 포트가 위 값과 일치해야 하며 Ubuntu 방화벽에서도 해당 UDP
수신을 허용해야 한다.

- `Sensor/lane_segmentation.onnx`: 저장소에 포함되어 있다.
- `yolov8n.pt`: 저장소에 없으며 Ultralytics가 최초 실행 시 내려받을 수 있다.
- `Sensor/null.pt`: 커스텀 신호등/장애물 모델이며 저장소에 포함되어 있지 않다.

커스텀 모델이 없으면 경고를 출력하고 기본 YOLO 객체 탐지만 계속한다. 커스텀
탐지가 필요하면 모델을 `Sensor/null.pt`에 복사하거나 launch 인자로 절대 경로를
지정한다.

### 최초 설치와 빌드

저장소 루트가 catkin workspace이다.

```bash
cd ~/ROI
git fetch origin
git switch dev/merged_sensor
git pull origin dev/merged_sensor

source /opt/ros/noetic/setup.bash
python3 -m pip install -r src/path_planning/requirements.txt
python3 -m pip install -r requirements.txt
catkin_make
source devel/setup.bash
```

새 터미널에서는 다음 두 환경 설정을 다시 적용한다.

```bash
cd ~/ROI
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 통합 실행

다음 한 줄로 LiDAR/RViz, YOLO, 보행자 정지·재출발 및 차량 제어를 실행한다.
차선 인식은 기본적으로 비활성화되어 있다. `enable_control` 기본값이 `true`이므로
네트워크와 위치 정보가 정상 수신되면 차량이 바로 움직일 수 있다.

```bash
cd ~/ROI
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch
```

정상 실행 시 다음 화면이 나타난다.

1. LiDAR tracking 및 merge-gap 상태를 표시하는 RViz
2. 카메라 실시간 화면
3. 입력 프레임과 bounding box가 일치하는 YOLO 검출 화면

종료할 때는 launch 터미널에서 `Ctrl+C`를 누른다. OpenCV 창에서 누르는
`q` 또는 `Esc`는 해당 카메라 프로세스만 종료할 수 있다.

### 센서만 안전하게 확인

차량 제어 없이 센서 수신과 화면만 확인하려면 제어 송신을 끈다.

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch enable_control:=false
```

목표 속도를 함께 지정하는 예:

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch \
  target_speed_mps:=6.0
```

### 주요 launch 인자

| 인자 | 기본값 | 설명 |
|---|---:|---|
| `bind_ip` | `0.0.0.0` | LiDAR와 카메라의 공통 로컬 UDP bind 주소 |
| `competition_status_host_port` | `9080` | MORAI Competition Status 송신(Host/Source) 포트 |
| `competition_status_port` | `9081` | ROS PC Competition Status 수신(Destination) 포트 |
| `morai_host_ip` | `192.168.0.148` | Ego Ctrl Cmd를 보낼 MORAI PC 주소 |
| `enable_control` | `true` | 차량 제어 UDP 송신 여부 |
| `target_speed_mps` | `6.0` | Pure Pursuit 목표 속도 (`21.6 km/h`) |
| `rviz` | `true` | LiDAR RViz 실행 여부 |
| `enable_lane` | `false` | 차선 인식 프로세스 실행 여부 |
| `lane_port` | `1101` | 차선 카메라 UDP 수신 포트 |
| `enable_yolo` | `true` | YOLO 프로세스 실행 여부 |
| `yolo_port` | `1131` | YOLO 카메라 UDP 수신 포트 |
| `base_model_path` | `yolov8n.pt` | 기본 YOLO 모델 경로/이름 |
| `custom_model_path` | `null.pt` | 커스텀 YOLO 모델 경로/이름 |
| `yolo_confidence` | `0.4` | YOLO confidence 임계값 |
| `yolo_inference_size` | `320` | YOLO 추론 입력 크기 |
| `camera_display_fps` | `0.0` | `0`은 MORAI 카메라 수신 속도를 그대로 사용 |
| `yolo_cpu_threads` | `1` | YOLO에 사용하는 PyTorch CPU 스레드 수 |
| `enable_highway_gate` | `true` | 카메라 기반 고속도로 환경 게이트 실행 |
| `require_dashed_lane` | `false` | `true`이면 car와 점선이 모두 탐지되어야 활성화 |
| `car_detection_hold_s` | `2.0` | 일시적인 YOLO 누락 시 car 조건 유지시간 |
| `enable_pedestrian_crossing` | `true` | YOLO person 기반 정지·재출발 제어 |
| `person_clear_confirmation_s` | `0.5` | person 미검출 후 재출발까지 연속 확인 시간 |
| `merge_available_topic` | `/perception/merge_gap/available` | 왼쪽 차선 끼어들기 가능 토픽 |
| `merge_unavailable_topic` | `/perception/merge_gap/unavailable` | 왼쪽 차선 끼어들기 불가능 토픽 |

통합 주행 브리지는 기존 일반 EgoVehicleStatus 포트 `909` 대신 Competition
Vehicle Status를 `9081`에서 받는다. 두 상태의 `#MoraiInfo$` 패킷 구조는 같지만
Competition 네트워크는 그중 일부 정보만 제공한다. ROS 주행 코드가 사용하는 차량
위치·heading·속도·가속도와 accel/brake/steer 입력을 기존
`/Ego_topic`(`morai_msgs/EgoVehicleStatus`) 형식으로 변환한다. 따라서 하위 ROS
노드의 토픽 이름은 바뀌지 않는다. 181바이트 기본 패킷과 229바이트 확장 패킷을
모두 지원한다.

커스텀 모델의 절대 경로를 지정하는 예:

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch \
  custom_model_path:=/home/user/models/morai_signal.pt
```

카메라만 개별 실행하는 예:

```bash
roslaunch camera_perception camera_perception.launch
```

YOLO를 끄고 차선 인식만 실행하는 예:

```bash
roslaunch camera_perception camera_perception.launch enable_yolo:=false
```

### 문제 해결

#### RViz는 뜨지만 LiDAR 점군이 보이지 않음

- MORAI LiDAR Destination Port가 `2001`인지 확인한다.
- 기존 `feat/lidar`와 동일한 Destination IP가 설정되어 있는지 확인한다.
- 다른 LiDAR 수신 프로세스가 같은 UDP 포트를 사용 중인지 확인한다.

#### 차선 또는 YOLO 창이 뜨지 않음

- MORAI 카메라 Destination Port가 각각 `1101`, `1131`인지 확인한다.
- Ubuntu의 `DISPLAY` 및 X11 GUI 환경을 확인한다.
- `python3 -m pip install -r requirements.txt`가 완료됐는지 확인한다.
- 같은 카메라 포트를 사용하는 기존 Python 프로그램을 종료한다.

#### 기본 YOLO 모델을 내려받지 못함

- 최초 실행 시 Ubuntu PC의 인터넷 연결을 확인한다.
- 미리 받은 `yolov8n.pt`의 절대 경로를 `base_model_path`로 지정한다.

#### 커스텀 탐지가 표시되지 않음

- `Sensor/null.pt` 또는 `custom_model_path`가 실제 파일을 가리키는지 확인한다.
- 모델의 class 이름과 학습 데이터가 현재 후처리 조건과 맞는지 확인한다.

#### YOLO 화면이 느리거나 끊겨 보임

- YOLO 수신/화면과 추론은 비동기로 실행되며 오래된 프레임을 큐에 쌓지 않는다.
- `yolo_cpu_threads=0`은 VirtualBox의 모든 vCPU를 YOLO가 점유하지 않도록 추론
  스레드 수를 자동 제한한다. 필요하면 `yolo_cpu_threads:=1`로 고정한다.
- 화면 상단의 `LIVE FPS`는 실제 화면 갱신률, `YOLO FPS`는 객체 검출 갱신률이다.
- `latency`는 해당 검출 프레임이 수신된 뒤 화면 결과가 준비될 때까지의 시간이다.
  커스텀 모델이 있으면 기본 `BASE` 결과를 먼저 표시하고 같은 프레임의
  `BASE+CUSTOM` 결과를 후속 반영하여 car/person 판단이 두 번째 모델을 기다리지 않는다.
- CPU가 느리면 `yolo_inference_size:=320`으로 줄여서 실행한다. 보행자
  검출 정확도가 유지되는지 반드시 MORAI에서 확인한다.

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch \
  enable_lane:=false yolo_inference_size:=320 camera_display_fps:=0 \
  yolo_cpu_threads:=1
```

수신 프레임이 0.5초 이상 끊기면 화면에
`NO NEW CAMERA FRAME - check MORAI UDP`가 표시된다. 이 경우는 YOLO 추론이
아니라 MORAI 카메라 Destination IP/Port 또는 UDP 패킷 수신을 확인한다.

#### YOLO bounding box가 객체에서 밀려 보임

- `Live Preview`는 실시간 수신 화면이며 box를 그리지 않는다.
- `YOLO Detection (Frame Matched)`는 YOLO가 실제로 추론한 동일 프레임에만
  box를 그린다. box 위치는 이 창에서 확인한다.
- 두 창의 시각이 다른 것은 YOLO 추론 지연을 숨기지 않고 표시하는 정상
  동작이다. 검출 토픽도 frame-matched 추론 결과로 갱신된다.

#### 화면은 정상인데 차량이 움직이지 않음

- 기본값은 `enable_control=true`이다. 센서만 확인할 때는 `enable_control:=false`로 지정한다.
- `morai_host_ip`, 제어 포트, `ctrl_mode=2`, `gear=4`를 확인한다.

### 끼어들기 상태 토픽 사용

RViz의 왼쪽 차선 확정 상태와 동일한 이진 판정을 확인한다.

```bash
rostopic echo /perception/merge_gap/available
rostopic echo /perception/merge_gap/unavailable
```

두 토픽은 모두 `std_msgs/Bool`이며 왼쪽 차선만 판단한다. 통합 카메라 launch에서는
YOLO가 COCO `car`를 탐지해 `/perception/camera/highway_environment=true`가 된
동안에만 끼어들기 판단과 RViz 왼쪽 선이 활성화된다. 정상 활성 상태에서는 두 값이
항상 반대다. 게이트가 꺼질 때는 이전 가능 상태를 남기지 않도록 한 번
`available=false`, `unavailable=true`를 발행한 뒤 판단 출력을 중단한다. 점선 인식은
아직 미구현이므로 `require_dashed_lane=false`가 기본이며, 구현 후 이 값을 `true`로
바꾸면 `car AND dashed_lane` 조건으로 전환된다. 기존 주행 제어에는 연결하지 않는다.

### 보행자 정지 및 재출발

```bash
rostopic echo /perception/camera/person_detected
rostopic echo /perception/pedestrian_crossing/stop_required
rostopic echo /perception/pedestrian_crossing/resume_allowed
rostopic echo /perception/pedestrian_crossing/status
```

YOLO에서 `person=true`가 발행되면 LiDAR 거리 조건 없이 즉시
`stop_required=true`가 된다. 실제 제어가 활성화된 경우 Pure Pursuit가
`velocity=0`, `brake=1`을 발행한다. 카메라에서 person이 0.5초간 연속
미검출되면 `resume_allowed=true`, `stop_required=false`가 되고 기존 전역
경로 추종으로 복귀한다. 자세한 조건은
[`docs/PEDESTRIAN_CROSSING.md`](docs/PEDESTRIAN_CROSSING.md)를 참고한다.

더 자세한 센서별 설명과 개별 실행법은
[`docs/LIDAR_CAMERA_INTEGRATION.md`](docs/LIDAR_CAMERA_INTEGRATION.md)를 참고한다.
끼어들기 메시지의 전체 필드와 좌표계 계약은
[`docs/MERGE_GAP_TOPIC_CONTRACT.md`](docs/MERGE_GAP_TOPIC_CONTRACT.md)를 참고한다.
