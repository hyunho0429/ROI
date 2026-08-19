# 왼쪽 차선 끼어들기 판단 토픽

현재 판단은 LiDAR tracking의 장애물 위치, bounding box, 상대 속도, 앞뒤 여유
거리와 TTC를 사용한다. YOLO 차량 검출과 점선 차선 조건은 아직 포함하지 않는다.

| 기능 | 토픽 | 메시지 | 값 |
|---|---|---|---|
| 왼쪽 차선 끼어들기 가능 | `/perception/merge_gap/available` | `std_msgs/Bool` | 가능하면 `data: true` |
| 왼쪽 차선 끼어들기 불가능 | `/perception/merge_gap/unavailable` | `std_msgs/Bool` | 불가능하면 `data: true` |

정상 입력에서는 두 값이 항상 반대다. RViz에서 왼쪽 공간이 확정된 초록색이면
`available=true`이고, 빨간색 또는 확인 중인 노란색이면 `unavailable=true`다.
LiDAR tracking 입력이 끊기거나 유효하지 않으면 안전하게 `available=false`,
`unavailable=true`를 발행한다.

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
