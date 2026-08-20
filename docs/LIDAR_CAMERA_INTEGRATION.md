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
| `enable_control` | `false` | 차량 제어 UDP 송신 |

커스텀 모델은 저장소에 포함되지 않는다. `Sensor/null.pt`에 복사하거나 절대 경로로
지정한다.

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit_lidar_camera.launch \
  custom_model_path:=/home/user/models/morai_signal.pt
```

커스텀 모델이 없으면 경고를 출력하고 COCO 기본 YOLO 탐지만 계속한다.

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
