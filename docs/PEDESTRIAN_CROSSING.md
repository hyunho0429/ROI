# 보행자 카메라 정지·재출발

통합 launch는 YOLO COCO `person` 판단만으로 보행자 정지 요청을 만든다.
LiDAR 객체, 거리, 바운딩 박스, odometry는 보행자 정지·재출발 판단에
사용하지 않는다. LiDAR 정적·동적 객체 토픽은 기존대로 독립 동작한다.

## 동작 조건

- YOLO가 `/perception/camera/person_detected=true`를 발행하면 즉시
  `/perception/pedestrian_crossing/stop_required=true`를 발행한다.
- Pure Pursuit는 `velocity=0`, `brake=1`, `steering=0`을 발행한다.
- YOLO `person=false`가 기본 0.5초 연속 유지되면
  `stop_required=false`, `resume_allowed=true`를 발행한다.
- Pure Pursuit는 현재 위치에서 기존 전역 경로와 목표 속도를 다시 사용한다.
- 정지 중 person 카메라 토픽이 끊기면 안전을 위해 정지를 해제하지 않는다.

0.5초 확인 시간은 한 프레임의 YOLO 미검출로 차량이 즉시 재출발하는
것을 막는 디바운스이다. `person_clear_confirmation_s`로 조정할 수 있다.

## 토픽

| 기능 | 토픽 | 메시지 |
|---|---|---|
| YOLO person | `/perception/camera/person_detected` | `std_msgs/Bool` |
| 보행자 정지 요청 | `/perception/pedestrian_crossing/stop_required` | `std_msgs/Bool` |
| 재출발 허용 | `/perception/pedestrian_crossing/resume_allowed` | `std_msgs/Bool` |
| 판단 상태 | `/perception/pedestrian_crossing/status` | `std_msgs/String` JSON |

상태 JSON에는 `mode`, `state`, `inputs_ready`, `person_detected`,
`person_input_age_s`, `transition`이 포함된다.

## 실행

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch \
  enable_control:=true \
  enable_yolo:=true \
  enable_pedestrian_crossing:=true \
  person_clear_confirmation_s:=0.5
```

## 확인

```bash
rostopic echo /perception/camera/person_detected
rostopic echo /perception/pedestrian_crossing/status
rostopic echo /perception/pedestrian_crossing/stop_required
rostopic echo /perception/pedestrian_crossing/resume_allowed
rostopic echo /ctrl_cmd
```
