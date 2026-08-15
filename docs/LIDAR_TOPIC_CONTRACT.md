# LiDAR ROS 토픽 및 map 장애물 메시지

이 문서는 `feat/lidar`의 권장 실행 파일인
`morai_udp_ekf_purepursuit_lidar_tracking.launch`를 기준으로 한다. MORAI 센서와
제어의 외부 연결은 UDP이고, 아래 ROS 토픽은 Ubuntu 내부 노드 간 통신과
RViz 시각화에 사용한다.

## 현재 LiDAR 토픽

| 기능 | 토픽 | 메시지 |
|---|---|---|
| Ego 차량 위치·자세 | `/localization/odometry` | `nav_msgs/Odometry` |
| 필터링된 LiDAR 점군 | `/morai/lidar/live_points` | `sensor_msgs/PointCloud2` |
| RViz 표시용 LiDAR 점군 | `/morai/lidar/display_points` | `sensor_msgs/PointCloud2` |
| Euclidean 군집 점군 | `/morai/lidar/euclidean/clustered_points` | `sensor_msgs/PointCloud2` |
| Euclidean 군집 결과 | `/morai/lidar/euclidean/results` | `std_msgs/String` (JSON 배열) |
| Euclidean 군집 시각화 | `/morai/lidar/euclidean/markers` | `visualization_msgs/MarkerArray` |
| Bounding Box 군집 점군 | `/morai/lidar/bbox/clustered_points` | `sensor_msgs/PointCloud2` |
| Bounding Box 결과 | `/morai/lidar/bbox/results` | `std_msgs/String` (JSON 배열) |
| Bounding Box 시각화 | `/morai/lidar/bbox/markers` | `visualization_msgs/MarkerArray` |
| Kalman+Hungarian 군집 점군 | `/morai/lidar/tracking/clustered_points` | `sensor_msgs/PointCloud2` |
| LiDAR 상대좌표 Tracking 결과 | `/morai/lidar/tracking/results` | `std_msgs/String` (JSON 배열) |
| Tracking 시각화 | `/morai/lidar/tracking/markers` | `visualization_msgs/MarkerArray` |
| map 좌표 Tracking 결과 | `/perception/lidar/tracked_obstacles_map` | `lidar_perception/LidarObstacleArray` |
| 끼어들기 공간 판단 | `/morai/lidar/merge_gap/results` | `std_msgs/String` (JSON) |
| 끼어들기 공간 시각화 | `/morai/lidar/merge_gap/markers` | `visualization_msgs/MarkerArray` |

`/perception/lidar/obstacle_stop_required`는 현재 코드에 없다. 정지 판단이
필요하면 map 장애물 결과를 사용하는 판단 노드에서 별도로 발행한다.

## map 좌표 장애물 메시지

토픽:

```text
/perception/lidar/tracked_obstacles_map
```

메시지:

```text
lidar_perception/LidarObstacleArray
```

`LidarObstacleArray.msg`:

```text
std_msgs/Header header
uint32 obstacle_count
LidarObstacle[] obstacles
```

- `header.stamp`: 장애물을 계산한 LiDAR PointCloud timestamp
- `header.frame_id`: `map`
- `obstacle_count`: `obstacles` 배열의 원소 수
- 확정 전 tentative track은 제외하며 기본적으로 3회 검출된 track부터 발행

`LidarObstacle.msg`:

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

| 필드 | 의미 | 단위 |
|---|---|---|
| `id` | Kalman+Hungarian Track ID | - |
| `center_x_map` | 장애물 회전 Bounding Box 중심의 map x | m |
| `center_y_map` | 장애물 회전 Bounding Box 중심의 map y | m |
| `yaw` | map +x축 기준 반시계방향 장애물 장축 방향 | rad |
| `length` | 회전 Bounding Box 장축 길이 | m |
| `width` | 회전 Bounding Box 단축 길이 | m |
| `velocity_x_map` | map x방향 장애물 속도 | m/s |
| `velocity_y_map` | map y방향 장애물 속도 | m/s |

`yaw`, `length`, `width`는 LiDAR 군집의 PCA 기반 2D 회전 Bounding Box에서
계산한다. 움직이는 장애물은 속도 방향으로 yaw의 앞뒤를 결정한다. 정지하거나
매우 느린 장애물은 LiDAR 형상만으로 앞뒤를 구분할 수 없어 yaw에 180도 모호성이
있지만, 충돌 검사에 사용하는 사각형의 점유 영역은 동일하다.

map Tracking은 LiDAR 군집을 PointCloud timestamp의
`/localization/odometry`로 map에 변환한 다음 별도 Kalman+Hungarian filter에
입력한다. 따라서 `/morai/lidar/tracking/results`의 Ego 상대속도를 단순히 이름만
바꾼 값이 아니다.

LiDAR 원점과 odometry의 `base_link` 원점이 다르면 launch에 실제 장착 변환을
넣어야 한다. 기본값은 세 값 모두 `0`이다.

```bash
lidar_x_m:=0.0 lidar_y_m:=0.0 lidar_yaw_deg:=0.0
```

- `lidar_x_m`: `base_link` 원점에서 LiDAR까지 전방 거리
- `lidar_y_m`: `base_link` 원점에서 LiDAR까지 좌측 거리
- `lidar_yaw_deg`: `base_link` 대비 LiDAR 수평 장착각

## 확인 명령

```bash
rosmsg show lidar_perception/LidarObstacle
rosmsg show lidar_perception/LidarObstacleArray
rostopic echo /perception/lidar/tracked_obstacles_map
rostopic hz /perception/lidar/tracked_obstacles_map
```

Frenet 회피 모듈은 `center_x_map`, `center_y_map`을 기준 경로에 투영해 `s`,
`d`로 바꾸고, `velocity_x_map`, `velocity_y_map`을 경로 접선·법선 방향으로
투영해 `velocity_s`, `velocity_d`로 사용한다.
