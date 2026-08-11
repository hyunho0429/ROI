# MORAI ROS 기록·센서 저장·Rosbag Replay 사용법

## 1. 기능을 어디에 사용할 것인가

기록과 재생은 localization·detection 알고리즘 내부에 섞지 않고 공통
recording/replay 계층으로 관리한다.

| 기능 | 실행 주체 | 결과 | 주 용도 |
|---|---|---|---|
| SaveSensorData | Ubuntu가 MORAI에 ROS 메시지 발행 | MORAI SensorData 폴더 | 현재 프레임의 센서 파일 저장 |
| rosbag record | Ubuntu ROS | .bag 파일 | ROS 토픽 전체 기록 |
| MORAI Rosbag Replay | MORAI가 bag 직접 읽음 | MORAI 환경 재현 | 시뮬레이터에서 주행·센서 재생 |
| Network Replay | Ubuntu rosbag play + MORAI Bridge | MORAI가 ReplayInfo 수신 | ROS 네트워크 기반 재생 |

## 2. SaveSensorData

MORAI 공식 ROS 메시지는 다음과 같다.

- 토픽: /SaveSensorData
- 메시지: morai_msgs/SaveSensorData
- 방향: Ubuntu -> MORAI

필드:

| 필드 | 의미 |
|---|---|
| is_custom_file_name | false이면 MORAI 기본 저장 경로, true이면 사용자 경로 사용 |
| custom_file_name | 사용자 지정 파일 이름 |
| file_dir | 사용자 지정 저장 디렉터리 |

기본 저장 위치는 MORAI Launcher의 MoraiLauncher_{os}_Data/SaveFile/SensorData이다.
센서 종류별 폴더로 저장된다.

### 기본 경로에 한 프레임 저장

Ubuntu에서 morai_msgs가 설치되어 있다는 전제하에 다음처럼 발행한다.

    rostopic pub -1 /SaveSensorData morai_msgs/SaveSensorData "{is_custom_file_name: false, custom_file_name: '', file_dir: ''}"

### 사용자 경로에 저장

file_dir는 MORAI PC에서 접근 가능한 디렉터리로 지정한다. Ubuntu의 경로를
입력한다고 MORAI PC의 파일 시스템에 저장되는 것은 아니다.

    rostopic pub -1 /SaveSensorData morai_msgs/SaveSensorData "{is_custom_file_name: true, custom_file_name: 'kcity_align_001', file_dir: 'D:/MORAI_capture'}"

경로 형식과 실제 저장 성공 여부는 MORAI PC 운영체제·Launcher 버전에 따라
확인한다. 먼저 false로 기본 경로 저장을 시험한 뒤 custom 경로를 사용한다.

SaveSensorData는 한 번의 현재 프레임 저장 명령이다. 연속적인 시간 시퀀스를
저장하려면 rosbag record를 사용한다.

## 3. Ubuntu rosbag record

### localization 정렬용 기록

GPS와 IMU, MORAI 기준값을 같은 시간축으로 기록한다.

    cd ~/morai_ws/data/rosbags
    rosbag record -O localization_alignment.bag /gps /Imu /Ego_topic /clock

이 bag는 다음을 검증하는 데 사용한다.

- GPSMessage를 MGeo map으로 변환한 위치
- EgoVehicleStatus.position과 GPS 위치의 잔차
- IMU angular_velocity·linear_acceleration의 단위와 축
- GPS 끊김 후 EKF 예측 연속성

### perception 기록

    rosbag record -O perception_run.bag /camera/front/image/compressed /camera/left/image/compressed /camera/right/image/compressed /camera/aux/image/compressed /lidar3D /clock

### 전체 검증 기록

    rosbag record -O full_validation.bag /gps /Imu /Ego_topic /camera/front/image/compressed /camera/left/image/compressed /camera/right/image/compressed /camera/aux/image/compressed /lidar3D /ReplayInfo_topic /clock

기록 전 반드시 rostopic type으로 topic과 message type을 확인한다.

## 4. MORAI Rosbag Replay

이 방식은 MORAI Simulator가 rosbag 파일을 직접 읽는 방식이다.

1. rosbag 파일을 MORAI Launcher의 Data/SaveFile/Rosbag 폴더에 복사한다.
2. MORAI SIM에서 PlayMode -> Replay -> Rosbag Replay를 연다.
3. File List에서 bag를 선택한다.
4. Load 완료 후 Start를 누른다.
5. Replayer Control Panel에서 Play를 누른다.
6. 필요하면 재생 위치·속도·Heading Offset·Height Offset을 조정한다.
7. Bridge Setting에서 /ReplayInfo_topic 발행을 켜고 ROS에서 확인한다.

이 방식은 센서가 차량에 다시 부착된 상태로 시나리오를 재현하고 센서
데이터셋을 만드는 데 적합하다. 재생되는 sensor frame rate와 Ego/NPC/
Pedestrian 재생 옵션을 확인한다.

## 5. Network Replay

이 방식은 Ubuntu에서 bag를 재생하고 MORAI가 ROS bridge를 통해
ReplayInfo를 받아 시뮬레이션을 재현한다.

### Ubuntu

    source /opt/ros/noetic/setup.bash
    roslaunch rosbridge_server rosbridge_websocket.launch
    cd ~/morai_ws/data/rosbags
    rosbag play localization_alignment.bag --clock

### MORAI

1. Bridge Setting에서 rosbridge IP와 port를 입력한다.
2. Connect를 누른다.
3. Replayer Control Panel이 표시되는지 확인한다.
4. Dataset과 Draw Mode를 선택한다.
5. Ubuntu의 rosbag play를 시작한다.
6. /ReplayInfo_topic이 발행되고 MORAI가 재생되는지 확인한다.

Network Replay에서는 bag에 포함된 토픽만 재생된다. localization을 다시
시험하려면 /gps, /Imu, /Ego_topic을 bag에 포함해야 하며, /ReplayInfo_topic
만 기록한 bag로는 raw GPS/IMU EKF를 재생할 수 없다.

## 6. ReplayInfo의 역할

ReplayInfo는 MORAI Rosbag Replay에서 발행되는 통합 정보이다.

- 토픽: /ReplayInfo_topic
- 메시지: morai_msgs/ReplayInfo
- 포함 정보: ego pedal·steering, orientation, acceleration, angular velocity,
  NPC·pedestrian·obstacle 목록

ReplayInfo는 localization의 GPS/IMU 입력으로 사용하지 않는다. 정렬 결과,
차량 상태, 주변 객체 재현 여부를 확인하는 검증·디버깅 입력으로만 사용한다.

## 7. ROSbag 재생 제어

MORAI 문서에는 다음 service가 정의되어 있다.

- service: /Morai_SimProc
- type: morai_msgs/MoraiSimProcSrv
- request: morai_msgs/MoraiSimProcHandle
- response: morai_msgs/MoraiSrvResponse

기본 sim_process_status는 Play 0x01, Pause 0x10, Stop 0x20이다.
Replay 모드에서 파일 로드·target·시작 시각·배속 조절 옵션을 조합할 수
있지만, 먼저 UI 기반 replay가 정상 동작한 뒤 service 자동화를 추가한다.

Ubuntu에서 실제 필드를 확인한다.

    rossrv show morai_msgs/MoraiSimProcSrv
    rosmsg show morai_msgs/MoraiSimProcHandle
    rosmsg show morai_msgs/SaveSensorData
    rosmsg show morai_msgs/ReplayInfo

## 8. 우리 팀의 권장 사용 순서

1. MORAI ROS 네이티브 topic 연결을 확인한다.
2. /gps, /Imu, /Ego_topic만 10초 기록한다.
3. GPS-MGeo alignment와 EKF를 offline bag로 검증한다.
4. SaveSensorData로 특정 시점의 카메라·LiDAR 원본을 저장한다.
5. perception bag로 카메라·LiDAR detection을 반복 검증한다.
6. 전체 bag로 localization·detection·control 통합을 확인한다.
7. 최종적으로 MORAI Rosbag Replay 또는 Network Replay에서 동일 결과를 확인한다.

실제 주행 중에는 ReplayInfo나 ObjectStatus를 센서 입력으로 사용하지 않고,
GPS·IMU·카메라·LiDAR 계약만 사용한다.

## 9. 원본 자료

- Replay - Rosbag: https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/replay-rosbag
- ROS 통신 메시지: https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/ros-2
- 센서 데이터 저장: https://morai-sim--drive-user-manual--en-22-r2.scrollhelp.site/msdume2/capture-sensor-data

