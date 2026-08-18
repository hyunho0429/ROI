# 끼어들기 가능 여부 및 map 장애물 토픽

## 현재 판정 범위

현재 끼어들기 가능 여부는 LiDAR tracking의 장애물 위치, bounding box, 상대 속도,
앞뒤 여유 거리와 TTC만 사용한다. RViz에서 확정된 초록색 공간과 동일한 상태만
`available=true`로 발행한다. 빨간색 `BLOCKED`와 노란색 `CHECKING`은 모두
주행 측에서 불가로 취급한다.

YOLO 차량 검출과 점선 차선 구간 조건은 아직 판정에 포함하지 않는다. 향후 두
기능을 추가할 때 perception 노드에서 최종 availability 조건에 결합하면 메시지와
주행 구독 인터페이스는 그대로 유지할 수 있다. 현재 `decision_source`는
`lidar_gap_only`이다.

## 토픽

### `/perception/merge_gap/status`

타입: `lidar_perception/MergeGapStatus`

```text
std_msgs/Header header
time timestamp
bool valid
bool any_available
bool left_available
bool right_available
string left_reason
string right_reason
string decision_source
uint32 obstacle_count
lidar_perception/LidarObstacle[] obstacles
```

- `header.frame_id`: 항상 `map`
- `header.stamp`, `timestamp`: map 장애물 목록의 측정 시각
- `valid`: LiDAR gap 판정과 최신 map 장애물 데이터가 모두 유효한지 여부
- `any_available`: 좌·우 중 하나 이상이 확정적으로 가능한지 여부
- `left_available`, `right_available`: RViz의 좌·우 초록색 확정 상태
- `left_reason`, `right_reason`: 가능/불가 또는 invalid 원인
- `decision_source`: 현재는 `lidar_gap_only`
- `obstacle_count`: `obstacles` 배열 길이
- `obstacles`: 같은 시각의 map 좌표 추적 장애물 목록

`valid=false`일 때 모든 availability는 안전을 위해 `false`이다.

### `/perception/lidar/tracked_obstacles_map`

타입: `lidar_perception/LidarObstacleArray`

끼어들기 상태와 무관하게 map 장애물 목록만 필요한 모듈을 위한 기존 토픽이다.
`MergeGapStatus.obstacles`와 동일한 필드 계약을 사용한다.

### 기존 호환 토픽

- `/morai/lidar/merge_gap/results`: 기존 JSON 문자열 결과
- `/morai/lidar/merge_gap/markers`: 기존 RViz marker

두 토픽은 기존 기능 보존을 위해 그대로 유지한다.

## 장애물 필드

`lidar_perception/LidarObstacle`:

```text
uint32 id
float64 center_x_map
float64 center_y_map
float64 yaw
float64 length
float64 width
float64 velocity_x_map
float64 velocity_y_map
```

모든 위치·방향·속도는 `map` 좌표계 기준이다.

- `id`: map-frame Kalman/Hungarian track ID
- `center_x_map`, `center_y_map`: 장애물 bounding box 중심점 `[m]`
- `yaw`: 장애물이 바라보는 map-frame 방향 `[rad]`
- `length`, `width`: oriented bounding box 길이와 폭 `[m]`
- `velocity_x_map`, `velocity_y_map`: map-frame 속도 성분 `[m/s]`

## 확인 명령

빌드 후 메시지 정의를 확인한다.

```bash
rosmsg show lidar_perception/MergeGapStatus
rosmsg show lidar_perception/LidarObstacle
```

실시간 상태와 map 장애물을 확인한다.

```bash
rostopic echo /perception/merge_gap/status
rostopic echo /perception/lidar/tracked_obstacles_map
```

발행 주기와 연결 상태를 확인한다.

```bash
rostopic hz /perception/merge_gap/status
rostopic info /perception/merge_gap/status
```

## 주행 게이트

`purepursuit_mgeo`는 선택적으로 `MergeGapStatus`를 구독한다.

- `merge_gate_enabled=false`(기본): 기존 주행을 완전히 보존하고 메시지는 모니터링만 한다.
- `merge_gate_enabled=true`: 선택한 방향이 확정 가능할 때만 기존 경로 주행을 허용한다.
- invalid, timeout, 불가 상태: brake 명령을 발행한다.
- 가능 상태: 기존 Pure Pursuit 경로 추종을 계속한다.

좌측 공간을 기준으로 게이트를 활성화하는 예:

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch \
  enable_control:=true \
  merge_gate_enabled:=true \
  merge_target_side:=left
```

`merge_target_side`는 `left`, `right`, `either` 중 하나이다. 이 게이트는 차선 변경
경로 또는 조향을 생성하지 않는다. 실제 끼어들기 수행에는 별도의 차선 변경 경로
생성 상태 머신이 추가로 필요하다.
