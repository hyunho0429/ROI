# LiDAR perception demos (UDP input + RViz)

세 예제는 MORAI LiDAR/GPS/IMU를 기존 UDP 수신 노드로 직접 받고, 알고리즘
결과를 로컬 ROS 토픽으로 RViz에만 표시한다. MORAI ROS 센서 토픽은 사용하지
않는다. 예제 하나가 UDP 포트를 사용하므로 세 launch는 한 번에 하나씩 실행한다.

## 빌드

```bash
cd ~/ROI
git switch feat/lidar
git pull --ff-only origin feat/lidar
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

MORAI Network Settings 기본값은 LiDAR `2000 -> 2001`, GPS 목적지 `3001`,
IMU 목적지 `4001`이다. 알고리즘 PC의 IP가 `192.168.56.101`이면 MORAI의 각
Destination IP를 `192.168.56.101`로 지정한다. launch의 `bind_ip` 기본값은
`0.0.0.0`이므로 보통 별도 변경이 필요 없다.

## 1. Euclidean clustering

```bash
roslaunch path_planning morai_lidar_euclidean.launch
```

- 군집별 PointCloud 색상과 `C0`, `C1` ID/점 개수를 표시한다.
- 결과 토픽: `/morai/lidar/euclidean/results`
- 주요 튜닝: `cluster_tolerance_m`, `cluster_min_points`,
  `cluster_min_height_m`

## 2. 3D bounding box

```bash
roslaunch path_planning morai_lidar_bbox.launch
```

- Euclidean 군집마다 ego 축 기준 AABB(Axis-Aligned Bounding Box)를 생성한다.
- 반투명 3D 박스, 외곽선, 거리와 `가로x세로x높이`를 실시간 표시한다.
- 결과 토픽: `/morai/lidar/bbox/results`

## 3. Kalman + Hungarian tracking

```bash
roslaunch path_planning morai_lidar_tracking.launch
```

- AABB 중심을 constant-velocity Kalman filter로 예측한다.
- Hungarian 전역 할당과 `match_distance_m` gating으로 검출-트랙을 연결한다.
- `TENT`는 생성 직후의 tentative track, `CONF`는 기본 3회 이상 연결된
  confirmed track이다. 화살표는 1초 동안의 상대속도 방향/크기를 나타낸다.
- 결과 토픽: `/morai/lidar/tracking/results`
- 주요 튜닝: `match_distance_m`, `min_hits`, `max_missed`

예:

```bash
roslaunch path_planning morai_lidar_tracking.launch \
  cluster_tolerance_m:=0.6 cluster_min_points:=3 \
  match_distance_m:=2.5 min_hits:=3 max_missed:=8
```

Tracking 좌표는 `morai_lidar` ego-local frame이므로 표시되는 속도는 Ego 대비
상대속도다. UDP 소스의 GPS/IMU deskew는 한 회전 내부 왜곡을 보상하지만,
이 학습용 standalone tracker는 프레임 사이 좌표를 world frame으로 바꾸지는
않는다.

Stanley와 동시에 실행하여 GPS/IMU 포트를 Stanley가 사용 중이라면 기존
fused-pose UDP `4012`를 사용한다.

```bash
roslaunch path_planning morai_lidar_tracking.launch \
  deskew_pose_source:=fused_pose_udp pose_bind_ip:=127.0.0.1 pose_port:=4012
```
