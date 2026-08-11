# MORAI UDP·GPS·IMU·EKF·Pure Pursuit 실제 주행 운용

이 문서는 MORAI SIM에서 GPS와 IMU를 UDP로 수신하고, GPS를 MGeo local ENU로 변환한 뒤,
IMU와 함께 EKF로 차량 기준점 pose를 추정하여 Pure Pursuit와 MORAI UDP 제어까지 연결하는
실제 운용 절차를 정리한다.

## 1. 전체 데이터 흐름

```text
MORAI GPS UDP 3001 ─┐
                    ├─ morai_udp_bridge ─> /gps, /Imu
MORAI IMU UDP 4001 ─┘          │
                               v
                     gps_mgeo_converter
                               │ /localization/gps
                               v
                     ekf_local_enu
                               │ /localization/odometry
                               v
                     purepursuit_mgeo
                               │ /ctrl_cmd (ROS)
                               v
                     morai_udp_drive_bridge
                               │ EgoCtrlCmd UDP
                               v
MORAI Cmd Control Destination Port

MORAI EgoVehicleStatus UDP 908 -> Ubuntu 909 -> /Ego_topic
```

`/Ego_topic`은 MORAI 차량의 위치·속도·heading을 받는 검증용 기준 토픽이다.
GPS·IMU 기반 EKF에 `/Ego_topic`을 넣지 않는다. 대회 입력이 아닌 시뮬레이터 ground truth를
EKF에 넣으면 localization 검증이 무효가 되기 때문이다.

## 2. MORAI Network Settings

현재 프로젝트 기본값은 다음과 같다.

| 통신 | MORAI Host Port | Ubuntu Destination/수신 Port | 방향 |
|---|---:|---:|---|
| GPS | 센서 설정값 | 3001 | MORAI -> Ubuntu |
| IMU | 센서 설정값 | 4001 | MORAI -> Ubuntu |
| EgoVehicleStatus | 908 | 909 | MORAI -> Ubuntu |
| EgoCtrlCmd | 9094 | 9093 | Ubuntu -> MORAI |

MORAI 버전이나 Network Settings에 다른 포트가 표시되면 launch 인자로 덮어쓴다.
특히 Cmd Control destination port는 `9093` 또는 설치된 예제의 `9096`일 수 있으므로
MORAI 설정 화면과 일치시켜야 한다.

MORAI Host IP는 현재 기본값 `192.168.0.151`이다. Ubuntu와 MORAI PC가 다른 네트워크라면
실제 MORAI PC 주소로 바꾼다.

## 3. Ubuntu 빌드

공식 `morai_msgs`의 `beta_drive` 패키지가 `src/morai_msgs`에 있어야 한다.

```bash
cd ~/morai_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
rospack profile
rospack find morai_msgs
```

## 4. 실제 주행 전 UDP·EKF만 검증

먼저 제어 출력을 막은 상태로 실행한다.

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit.launch \
  morai_host_ip:=192.168.0.151 \
  control_remote_port:=9093 \
  enable_control:=false
```

확인한다.

```bash
rostopic hz /gps
rostopic hz /Imu
rostopic hz /Ego_topic
rostopic hz /localization/gps
rostopic hz /localization/odometry
rostopic echo -n 1 /gps
rostopic echo -n 1 /Imu
rostopic echo -n 1 /Ego_topic
rostopic echo -n 1 /localization/odometry
rosrun tf tf_echo map base_link
```

정지 상태에서 다음을 확인한다.

1. GPS `status`가 1 이상이고 위도·경도·고도가 유효하다.
2. IMU packet layout과 쿼터니언이 정상이다.
3. `/localization/gps`가 MGeo 경로와 같은 좌표 범위에 있다.
4. `/localization/odometry`의 위치가 GPS보다 부드럽고 yaw가 차량 방향을 나타낸다.
5. `/Ego_topic`의 위치와 EKF 위치 차이를 기록한다. 이 차이는 검증용이지 EKF 입력이 아니다.

## 5. 실제 MORAI 제어 활성화

센서와 EKF를 먼저 확인한 뒤에만 실행한다.

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit.launch \
  morai_host_ip:=192.168.0.151 \
  control_remote_port:=9093 \
  wheelbase_m:=3.0 \
  enable_control:=true
```

이때 `/ctrl_cmd`의 ROS 값은 다음처럼 UDP EgoCtrlCmd로 변환된다.

| ROS CtrlCmd | UDP EgoCtrlCmd |
|---|---|
| velocity m/s | velocity km/h (`×3.6`) |
| steering rad | steer -1~1 (`steering / 40°`) |
| longlCmdType | cmd_type |
| accel/brake 0~1 | accel/brake 0~1 |

시작 시 명령이 없거나 0.5초 이상 끊기면 브릿지는 정지 명령을 보낸다. 종료 시에도
브레이크 정지 패킷을 한 번 보낸다.

## 6. 반드시 분리할 테스트

다음 두 launch를 동시에 실행하지 않는다.

- `mgeo_purepursuit_closed_loop.launch`: 가상 차량이 `/localization/odometry`를 발행
- `morai_udp_ekf_purepursuit.launch`: 실제 GPS·IMU EKF가 `/localization/odometry`를 발행

둘을 동시에 실행하면 localization topic이 충돌한다.

## 7. 공식 프로토콜 단위

- EgoVehicleStatus: 181 bytes
- EgoCtrlCmd: 55 bytes
- EgoVehicleStatus 위치: ENU m
- EgoVehicleStatus heading/steer: deg
- EgoVehicleStatus velocity: UDP 원본 km/h
- EgoCtrlCmd velocity: km/h
- EgoCtrlCmd steering: 실제 앞바퀴각/최대 앞바퀴각, -1~1

이 변환은 MORAI 공식 UDP 프로토콜 정의에 맞춘다.
