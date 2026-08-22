# 보행자 횡단 정지·재출발

통합 카메라 launch는 YOLO COCO `person`과 기존 LiDAR 동적 객체 추적 결과를
결합해 횡단보도 보행자 정지 요청을 만든다. 카메라와 LiDAR 객체를 calibration으로
직접 투영해 1:1 매칭하는 방식은 아니며, 카메라 person 조건과 차량 전방·측면의
사람 크기 LiDAR cluster 조건을 동시에 확인하는 조건 수준 fusion이다.

## 동작 조건

정지 조건은 다음 두 조건이 0.2초 동안 함께 유지되는 것이다.

1. `/perception/camera/person_detected=true`
2. `/detection/dynamic_obstacles`에서 차량 후방 0.5m 이내를 제외한 전방·측면에
   길이와 폭이 각각 1.5m 이하인 객체의 bounding box가 LiDAR 원점 기준 약 1.5m
   이내에 존재

정지가 활성화되면 Pure Pursuit는 전역 경로 계산을 유지하면서 `velocity=0`,
`brake=1`인 `/ctrl_cmd`를 발행한다.

재출발은 다음 조건이 1초 동안 모두 유지될 때만 허용한다.

1. 카메라에서 person이 사라짐
2. `/detection/obstacle_states`의 전방·측면 1.5m 영역에서 사람 크기 LiDAR
   cluster가 모두 사라짐
3. 카메라, LiDAR 객체 상태, odometry 입력이 모두 최신 상태

센서 입력이 끊기면 이미 활성화된 정지는 해제하지 않는다. 안전 해제 후 Pure
Pursuit는 현재 위치에서 기존 전역 경로와 `target_speed_mps`를 다시 사용한다.

## 토픽

| 기능 | 토픽 | 메시지 |
|---|---|---|
| YOLO person | `/perception/camera/person_detected` | `std_msgs/Bool` |
| 보행자 정지 요청 | `/perception/pedestrian_crossing/stop_required` | `std_msgs/Bool` |
| 재출발 허용 | `/perception/pedestrian_crossing/resume_allowed` | `std_msgs/Bool` |
| 판단 상태 | `/perception/pedestrian_crossing/status` | `std_msgs/String` JSON |
| 전체 LiDAR 객체 입력 | `/detection/obstacle_states` | `std_msgs/String` JSON |
| 동적 LiDAR 객체 입력 | `/detection/dynamic_obstacles` | `std_msgs/String` JSON |

상태 JSON에는 `state`, `inputs_ready`, `person_detected`,
`dynamic_candidate_count`, `nearby_lidar_count`, `nearest_distance_m`,
`transition`이 포함된다.

## 확인

```bash
rostopic echo /perception/camera/person_detected
rostopic echo /perception/pedestrian_crossing/status
rostopic echo /perception/pedestrian_crossing/stop_required
rostopic echo /perception/pedestrian_crossing/resume_allowed
rostopic echo /ctrl_cmd
```

1.5m 기준은 launch 인자로 조정할 수 있다.

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch \
  pedestrian_detection_distance_m:=1.5
```
