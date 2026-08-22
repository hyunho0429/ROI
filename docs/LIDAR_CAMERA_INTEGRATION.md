# LiDAR + Camera 통합 실행

이 구성은 `feat/lidar`의 주행, LiDAR tracking 및 RViz launch를 그대로 포함하고
`feature-camera`의 차선 후보 추출과 YOLO 객체 탐지를 독립 프로세스로 추가한다.
카메라 프로세스가 종료되어도 주행/LiDAR 노드를 직접 변경하거나 종료시키지 않는다.

## 준비

Ubuntu ROS1 환경에서 저장소를 catkin workspace로 사용한다.

```bash
cd ~/morai_ws
python3 -m pip install -r requirements.txt
catkin_make
source devel/setup.bash
```

MORAI 센서 설정의 수신 IP/포트를 다음 기본값과 맞춘다.

| 기능 | 기본 주소/포트 |
|---|---|
| LiDAR | `0.0.0.0:2001` |
| 차선 카메라 | `0.0.0.0:1101` |
| YOLO 카메라 | `0.0.0.0:1131` |

통합 launch의 `camera_ip`은 기본적으로 LiDAR와 동일한 `bind_ip`를 사용한다.
기본 `0.0.0.0`은 Ubuntu PC의 모든 네트워크 인터페이스에서 UDP를 수신한다.
MORAI 센서의 목적지 IP는 별도로 Ubuntu PC의 실제 IP로 설정해야 한다.

## 전체 실행

먼저 제어 송신을 끈 상태로 화면과 센서 수신을 검증한다.

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch \
  enable_control:=false
```

정상 실행 시 다음 세 화면이 함께 나타난다.

1. 기존 LiDAR tracking RViz
2. `Lane Candidate Extraction` 차선 인식 화면
3. `MORAI Cam 4 Traffic & Object Monitor` YOLO 화면

경로·센서·조향 방향과 비상 정지를 확인한 뒤에만 실제 제어를 켠다.

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch \
  enable_control:=true \
  target_speed_mps:=6.0
```

## 주요 launch 인자

| 인자 | 기본값 | 설명 |
|---|---:|---|
| `rviz` | `true` | LiDAR RViz 표시 |
| `enable_lane` | `true` | 차선 인식 프로세스 실행 |
| `lane_port` | `1101` | 차선 카메라 UDP 포트 |
| `enable_yolo` | `true` | YOLO 프로세스 실행 |
| `yolo_port` | `1131` | YOLO 카메라 UDP 포트 |
| `base_model_path` | `yolov8n.pt` | 기본 YOLO 모델 |
| `custom_model_path` | `null.pt` | 커스텀 신호등/장애물 모델 |
| `yolo_confidence` | `0.4` | YOLO confidence 임계값 |
| `yolo_inference_size` | `416` | YOLO 추론 입력 크기(작을수록 빠르지만 소형 객체 정확도 감소) |
| `camera_display_fps` | `0.0` | `0`은 MORAI 수신 프레임율로 즉시 표시 |
| `yolo_cpu_threads` | `0` | VM GUI/UDP용 CPU를 남기는 PyTorch 자동 스레드 설정 |
| `enable_highway_gate` | `true` | YOLO 기반 고속도로 환경 게이트 |
| `require_dashed_lane` | `false` | 점선 인식 구현 후 `true`로 전환 |
| `car_detection_hold_s` | `2.0` | YOLO car 조건 유지시간 |
| `enable_pedestrian_crossing` | `true` | YOLO person 기반 정지·재출발 |
| `person_clear_confirmation_s` | `0.5` | person 미검출 후 재출발 확정 시간 |
| `enable_control` | `false` | 차량 제어 UDP 송신 |

커스텀 모델은 저장소에 포함되지 않는다. `Sensor/null.pt`에 복사하거나 절대 경로로
지정한다.

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch \
  custom_model_path:=/home/user/models/morai_signal.pt
```

커스텀 모델이 없으면 경고를 출력하고 COCO 기본 YOLO 탐지만 계속한다.

YOLO 수신/표시와 모델 추론은 서로 다른 스레드에서 동작한다. 화면은 추론을
기다리지 않고 최신 UDP 프레임을 보여 주며, 검출 박스와 Bool 토픽은 가장
최근에 완료된 추론 결과로 갱신된다. 화면 상단에 `LIVE FPS`, `YOLO FPS`,
`infer ms`, `result age` 가 표시된다.
새 UDP 프레임이 0.5초 이상 없으면 창의 이벤트 처리는 계속하면서
`NO NEW CAMERA FRAME` watchdog 문구와 수신 상태를 로그로 표시한다.

## 고속도로 환경 기반 끼어들기 활성화

통합 launch에서는 기본 YOLO의 COCO `car` 탐지 결과를
`/perception/camera/car_detected`(`std_msgs/Bool`)로 발행한다. 별도 게이트 노드가
이를 2초간 유지해 `/perception/camera/highway_environment`를 10 Hz로 발행한다.
이 값이 `true`인 동안에만 왼쪽 끼어들기 판단, 관련 Bool 결과와 RViz 선이
활성화된다.

점선 인식 토픽 `/perception/camera/dashed_lane_detected`는 향후 연결을 위해 미리
정의되어 있지만 현재 발행 노드는 없다. 점선 인식 구현 후 다음 인자를 사용하면
고속도로 게이트가 `car AND dashed_lane`으로 동작한다.

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch \
  require_dashed_lane:=true
```

## 보행자 횡단 정지

YOLO `person=true`가 확인되면 LiDAR 조건 없이
`/perception/pedestrian_crossing/stop_required=true`가 즉시 발행된다. Pure Pursuit는
이를 최우선 정지 조건으로 사용한다. 카메라에서 person이 0.5초간
연속 미검출되면 `/perception/pedestrian_crossing/resume_allowed=true`가 되고
기존 전역 경로를 다시 추종한다. 세부 토픽과 시간 조건은
[`PEDESTRIAN_CROSSING.md`](PEDESTRIAN_CROSSING.md)를 참고한다.

## 개별 확인

카메라 화면만 확인하려면 다음을 사용한다.

```bash
roslaunch camera_perception camera_perception.launch \
  camera_ip:=0.0.0.0
```

한 기능씩 끌 수도 있다.

```bash
roslaunch camera_perception camera_perception.launch \
  camera_ip:=0.0.0.0 enable_yolo:=false
```

GUI가 보이지 않으면 X11/`DISPLAY` 설정과 OpenCV GUI 지원 여부를 확인한다. 포트가
이미 사용 중이면 같은 카메라 포트를 수신하는 기존 Python 프로세스를 종료하거나
launch 인자를 MORAI 센서 설정과 함께 변경한다.
