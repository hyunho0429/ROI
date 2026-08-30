# 왼쪽 차선 끼어들기 판단 토픽

공간 판단은 LiDAR tracking의 장애물 위치, bounding box, 상대 속도, 앞뒤 여유
거리와 TTC를 사용한다. 통합 카메라 launch에서는 YOLO `car`, `bus`, `truck` 중
하나의 탐지와 HD MAP에서 투영한 자차 왼쪽 점선 경계가 함께 확인되면 고속도로
환경을 활성화한다. 이 최초 활성화 조건에서는 기존 LiDAR 평행 객체 탐지를 사용하지
않으며, 활성화 뒤의 실제 끼어들기 공간 판단에는 기존 LiDAR tracking을 그대로 쓴다.

| 기능 | 토픽 | 메시지 | 값 |
|---|---|---|---|
| 왼쪽 차선 끼어들기 가능 | `/perception/merge_gap/available` | `std_msgs/Bool` | 가능하면 `data: true` |
| 왼쪽 차선 끼어들기 불가능 | `/perception/merge_gap/unavailable` | `std_msgs/Bool` | 불가능하면 `data: true` |
| YOLO 도로 차량 탐지 | `/perception/camera/car_detected` | `std_msgs/Bool` | 현재 프레임에서 COCO `car`, `bus`, `truck` 중 하나 탐지(기존 토픽명 유지) |
| HD MAP 왼쪽 점선 | `/perception/camera/dashed_lane_detected` | `std_msgs/Bool` | 카메라 영상에 투영된 자차 왼쪽 MGeo 경계가 유효한 백색 점선이면 `true` |
| 고속도로 환경 게이트 | `/perception/camera/highway_environment` | `std_msgs/Bool` | YOLO car/bus/truck AND HD MAP 왼쪽 점선, 최초 ON 후 유지 |

정상 입력에서는 두 값이 항상 반대다. RViz에서 왼쪽 공간이 확정된 초록색이면
`available=true`이고, 빨간색 또는 확인 중인 노란색이면 `unavailable=true`다.
LiDAR tracking 입력이 끊기거나 유효하지 않으면 안전하게 `available=false`,
`unavailable=true`를 발행한다.

고속도로 게이트가 `false`이면 RViz 끼어들기 선과 주기적인 판단 발행을 중단한다.
게이트가 `true`에서 `false`로 바뀔 때는 이전 `available=true`가 남지 않도록 한 번
`available=false`, `unavailable=true`를 발행한다.

두 메시지에는 timestamp, 장애물 개수, 위치, 속도 등 추가 데이터가 없다. 이
정보가 따로 필요하면 기존 `/perception/lidar/tracked_obstacles_map`
(`lidar_perception/LidarObstacleArray`) 토픽을 구독한다.

```bash
rostopic echo /perception/merge_gap/available
rostopic echo /perception/merge_gap/unavailable
rostopic type /perception/merge_gap/available
rostopic type /perception/merge_gap/unavailable
```

이 토픽은 판단 결과만 발행하며 차량을 직접 제어하지 않는다.
