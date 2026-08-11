# Pure Pursuit + UDP LiDAR 인지 실행법

## 구성

MORAI 외부 통신은 UDP를 사용한다.

- GPS: Ubuntu UDP `3001`
- IMU: Ubuntu UDP `4001`
- EgoVehicleStatus: Ubuntu UDP `909`
- LiDAR: MORAI source `2000` → Ubuntu destination `2001`
- EgoCtrlCmd: Ubuntu source `9094` → MORAI destination `9093`

ROS 토픽은 같은 Ubuntu catkin workspace 안에서 노드를 연결하고 RViz로
시각화하는 용도로만 사용한다. LiDAR는 MORAI ROS 센서 토픽을 구독하지 않고
UDP `2001`을 직접 수신한다. GPS/IMU 포트 중복 바인딩을 피하기 위해 LiDAR
deskew는 UDP EKF가 발행한 `/localization/odometry`를 로컬에서 사용한다.

LiDAR UDP 포트는 한 프로세스만 열 수 있으므로 아래 통합 launch 중 하나만
실행한다.

## 빌드

```bash
cd ~/morai_ws/src/ROI
git fetch origin
git switch dev/pure_pursuit
git pull --ff-only origin dev/pure_pursuit

cd ~/morai_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
rospack profile
```

`morai_msgs`는 공식 `beta_drive` 버전이 workspace `src` 안에 있어야 한다.

## 단계별 통합 실행

아래 예시는 MORAI PC가 `192.168.56.1`, Ubuntu 센서 목적지가
`192.168.56.101`인 구성이다. 먼저 `enable_control:=false`로 센서와 인지
결과를 확인한다.

### 1. Euclidean clustering + 주행 스택

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_euclidean.launch \
  morai_host_ip:=192.168.56.1 \
  control_remote_port:=9093 \
  enable_control:=false \
  rviz:=true
```

- 결과: `/morai/lidar/euclidean/results`
- 시각화: 군집별 색상 PointCloud와 중심점/ID

### 2. 3D bounding box + 주행 스택

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_bbox.launch \
  morai_host_ip:=192.168.56.1 \
  control_remote_port:=9093 \
  enable_control:=false \
  rviz:=true
```

- 결과: `/morai/lidar/bbox/results`
- 시각화: Euclidean 군집과 3D axis-aligned bounding box

### 3. Kalman + Hungarian tracking + 주행 스택

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_tracking.launch \
  morai_host_ip:=192.168.56.1 \
  control_remote_port:=9093 \
  enable_control:=false \
  rviz:=true
```

- 결과: `/morai/lidar/tracking/results`
- 시각화: Track ID, box, 상대속도 벡터
- Kalman filter: 위치와 속도를 예측·보정한다.
- Hungarian assignment: 현재 box와 기존 track의 전체 거리 비용이 최소가
  되도록 1:1 대응시킨다.

### 4. 끼어들기 공간 판단 + 주행 스택

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_merge_gap.launch \
  morai_host_ip:=192.168.56.1 \
  control_remote_port:=9093 \
  enable_control:=false \
  rviz:=true
```

필요하면 같은 명령에 `lane_width_m:=3.5 time_headway_s:=1.5
minimum_ttc_s:=3.0 confirmation_scans:=3`을 추가해 판단 기준을 조정한다.

- 결과: `/morai/lidar/merge_gap/results`
- 시각화: 왼쪽·오른쪽 목표 차선 공간을 빨강/노랑/초록으로 표시
- 로그: `MERGE_GAP TRACKED`, `MERGE_GAP AVAILABLE`, `MERGE_GAP LOST`

기본 차량 크기는 `4.635 × 1.892 × 2.434 m`, 차선 폭은 `3.5 m`다.
각 목표 차선에서 ego 옆과 겹치는 box가 없어야 하고, 앞·뒤 차량까지의
여유거리가 다음 동적 기준 이상이어야 한다.

```text
앞 필요거리 = 1.0 m + max(0, -앞차 상대 x속도) × 1.5 s
뒤 필요거리 = 1.0 m + max(0,  뒤차 상대 x속도) × 1.5 s
필요 공간    = ego 길이 + 앞 필요거리 + 뒤 필요거리
```

앞·뒤 TTC는 기본 `3.0 s` 이상이어야 하고, 모든 조건이 3회 연속 만족해야
`AVAILABLE`로 확정한다. Tracking 메시지가 0.5초 이상 끊기면 결과는
`valid=false`로 바뀌며 확보 상태를 해제한다.

상태 확인:

```bash
rostopic echo /morai/lidar/merge_gap/results
rostopic hz /morai/lidar/live_points
rostopic hz /morai/lidar/tracking/results
```

로그 예시:

```text
MERGE_GAP TRACKED | LEFT=CHECKING(2/3) ... | RIGHT=BLOCKED ...
MERGE_GAP AVAILABLE: LEFT=AVAILABLE reason=available ...
MERGE_GAP LOST: LEFT=BLOCKED reason=rear_ttc_short ...
```

## 실제 제어 활성화

GPS, IMU, EKF, LiDAR와 인지 결과를 먼저 확인한 뒤 동일 명령의
`enable_control`만 `true`로 바꾼다.

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_merge_gap.launch \
  morai_host_ip:=192.168.56.1 \
  control_remote_port:=9093 \
  enable_control:=true \
  rviz:=true
```

현재 끼어들기 결과는 인지 출력이다. `AVAILABLE`이 되어도 자동 차선 변경
명령은 내리지 않으며 기존 Pure Pursuit는 주어진 global path를 계속 추종한다.
