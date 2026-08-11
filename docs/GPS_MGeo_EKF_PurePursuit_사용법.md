# GPS·IMU·MGeo·Pure Pursuit 통합 사용법

이 문서는 현재 MORAI 환경에서 뒤 차축 중앙을 차량 기준점(`base_link`)으로 사용하고,
MORAI UDP 센서값을 MGeo local ENU 좌표계로 바꾼 뒤 EKF와 Pure Pursuit를 연결하는 방법을 정리한다.

## 1. 현재 확정한 기준

| 항목 | 설정 |
|---|---|
| `base_link` 원점 | 차량 뒤 차축 중앙 |
| `base_link` 축 | x 전방, y 좌측, z 위쪽 |
| GPS 위치 | `(0, 0, 1.2)` m, RPY `(0, 0, 0)` |
| IMU 위치 | `(0, 0, 0)` m, RPY `(0, 0, 0)` |
| 지도 좌표 | MGeo local ENU: x=동쪽, y=북쪽, z=위쪽 |
| MGeo 원점 | UTM52N `(302595.0, 4124145.0, 0.0)` |
| Pure Pursuit 기준점 | `base_link`, 즉 뒤 차축 중앙 |
| 차량 모델 | `2025_Hyundai_ioniq5` |
| 휠베이스 | `3.000 m` |
| 최대 휠각 | `40° = 0.6981317008 rad` |
| 최소 회전반경 | `5.87 m` |
| 카메라 | 이번 통합 런치에는 포함하지 않음 |

센서 외부 파라미터의 실제 파일은 `config/sensor_extrinsics.yaml`이다. GPS의 x·y가
기준점과 같으므로 평면 위치는 그대로 사용하고, 높이를 기준점으로 표현할 때만 GPS z에서
1.2 m를 뺀다. IMU는 현재 기준점과 같은 위치·자세로 설정되어 있다.

## 2. 데이터 흐름

```text
MORAI UDP
  ├─ GPS  -> morai_udp_bridge -> /gps (morai_msgs/GPSMessage)
  └─ IMU  -> morai_udp_bridge -> /Imu (sensor_msgs/Imu)

/gps + MGeo UTM 원점
  -> gps_mgeo_converter
  -> /localization/gps (nav_msgs/Odometry, map -> base_link 위치)

/localization/gps + /Imu
  -> ekf_local_enu
  -> /localization/odometry, /localization/pose, map -> base_link TF

/localization/odometry + data/routes/2026_molit_comp_global_path.txt
  -> purepursuit_mgeo
  -> /control/lookahead_point, /control/steering_preview
  -> /ctrl_cmd (enable_control=true일 때만 발행)
```

## 3. Ubuntu에서 빌드

저장소를 `/home/<사용자>/morai_ws`에 직접 받았다는 기준이다.

```bash
cd ~/morai_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

새 터미널을 열 때마다 `source devel/setup.bash`를 실행한다.

## 4. 먼저 센서 토픽만 검증

MORAI에서 GPS와 IMU UDP를 송신한 뒤, 아래 통합 런치를 실행한다. 기본값은 Pure Pursuit의
`/ctrl_cmd` 제어 출력을 막아 두었으므로 센서·좌표·추정만 검증할 수 있다.

```bash
roslaunch morai_bringup localization_purepursuit.launch enable_control:=false
```

다른 터미널에서 확인한다.

```bash
rostopic list | egrep 'gps|Imu|localization|lookahead|steering'
rostopic hz /gps
rostopic hz /Imu
rostopic echo -n 1 /gps
rostopic echo -n 1 /localization/gps
rostopic echo -n 1 /localization/odometry
rosrun tf tf_echo map base_link
```

확인할 내용은 다음과 같다.

1. `/gps`의 위도·경도·고도가 MORAI 화면의 차량 위치와 변하는 방향이 같은가.
2. `/localization/gps`의 x·y가 MGeo 경로와 같은 영역에 있는가.
3. 정지 상태에서 `/localization/odometry`의 x·y가 심하게 튀지 않는가.
4. 차량을 전진시킬 때 x·y가 경로 방향으로 변하고, yaw가 차량 진행 방향을 나타내는가.
5. `/control/lookahead_point`가 경로 위에 있고 `/control/steering_preview`가 좌회전·우회전에
   맞춰 부호를 바꾸는가.

## 5. GPS·IMU rosbag 기록

UDP를 ROS 토픽으로 변환하는 노드가 실행 중일 때 기록한다.

```bash
mkdir -p ~/morai_ws/data/rosbag
rosbag record -O ~/morai_ws/data/rosbag/gps_imu_$(date +%Y%m%d_%H%M%S).bag \
  /gps /Imu /localization/gps /localization/odometry /tf /tf_static
```

기록이 끝나면 `Ctrl+C`로 종료한다. 재생은 다음처럼 한다.

```bash
roscore
rosbag play --clock ~/morai_ws/data/rosbag/<기록파일>.bag
```

현재 EKF는 센서 메시지의 헤더 시간으로 계산한다. UDP IMU 패킷의 장비 시간과 GPS 시간축이
서로 다른 경우를 피하기 위해 통합 런치는 `imu_use_packet_time=false`로 두었고, 두 센서 모두
Ubuntu에서 수신한 ROS 시각을 사용한다. 시간 동기 검증이 끝난 뒤에만 패킷 시각 사용을 다시
검토한다.

## 6. Pure Pursuit를 제어 출력까지 켜는 순서

다음 항목을 먼저 확인한다.

1. 차량 모델이 `2025_Hyundai_ioniq5`인지 확인한다.
2. `wheelbase_m:=3.0`이 적용되었는지 확인한다.
3. 최대 휠각 `40°`가 `0.6981317008 rad`로 제한되는지 확인한다.
4. MGeo 경로와 초기 `/localization/odometry` 위치가 같은 좌표계인지 확인한다.
5. 정지·저속에서 `/control/steering_preview` 부호가 MORAI 조향 방향과 같은지 확인한다.
6. `CtrlCmd`의 종방향 명령 모드와 필드 이름을 Ubuntu에서 확인한다.

```bash
rosmsg show morai_msgs/CtrlCmd
```

그 다음에만 제어 출력을 켠다.

```bash
roslaunch morai_bringup localization_purepursuit.launch \
  wheelbase_m:=3.0 enable_control:=true
```

`purepursuit_mgeo_node.py`는 MGeo x·y를 `map`에서 `base_link`로 변환하고,
뒤 차축 중앙을 기준으로 곡률을 계산한 뒤 MORAI `CtrlCmd.steering`에 라디안 값을 넣는다.
경로 파일은 위도·경도가 아니라 이미 MGeo local ENU x y z 형식이어야 한다.

## 7. MORAI 없이 대회 경로만 먼저 검증

GPS·IMU가 아직 준비되지 않았을 때는 경로의 각 점을 차량 pose처럼 재생해서 Pure Pursuit의
추종점과 조향 계산만 확인할 수 있다. 이 테스트는 실제 localization이 아니며, `/ctrl_cmd`를
발행하지 않는다.

```bash
roslaunch purepursuit_mgeo mgeo_purepursuit_dryrun.launch
```

확인할 토픽:

```bash
rostopic hz /localization/odometry
rostopic echo -n 1 /control/lookahead_point
rostopic echo -n 1 /control/steering_preview
rostopic info /ctrl_cmd
```

터미널에는 경로 점 수와 Pure Pursuit의 nearest index, lookahead 거리, steering preview가
출력된다. 경로 재생이 끝나면 `MGeo 경로 pose 재생을 완료했다.`가 출력된다.

## 8. CtrlCmd 폐루프 제어 검증

Pure Pursuit가 실제로 발행한 `/ctrl_cmd`를 가상 차량 모델이 받아서 차량 pose를 적분하는
폐루프 테스트를 실행할 수 있다. 이 단계에서는 `/ctrl_cmd`가 실제 MORAI로 전달되지 않으며,
MORAI 차량의 물리 엔진·타이어 모델을 검증하는 테스트도 아니다.

```bash
roslaunch purepursuit_mgeo mgeo_purepursuit_closed_loop.launch
```

확인할 토픽:

```bash
rostopic hz /localization/odometry
rostopic echo /control/steering_preview
rostopic echo /ctrl_cmd
```

가상 차량이 `/ctrl_cmd`의 velocity와 steering을 사용해 움직이고, Pure Pursuit가 갱신된
`/localization/odometry`를 다시 받아 경로를 추종한다. 이 테스트를 통과한 후에 실제 MORAI
차량의 `/localization/odometry`로 교체한다.

## 9. 실제 센서 연결 전환

경로 replay를 종료한 뒤에만 실제 통합 런치를 실행한다.

```bash
roslaunch morai_bringup localization_purepursuit.launch enable_control:=false
```

이때 `/localization/odometry`는 replay 노드가 아니라 GPS-MGeo 변환기와 ENU EKF가 발행해야
한다. 두 노드가 동시에 같은 `/localization/odometry`를 발행하면 안 된다.

## 10. 자주 발생하는 문제

### 위치가 수백만 m로 나오거나 경로에서 멀다

UTM 절대좌표와 MGeo local 좌표를 섞은 경우다. `global_info.json`의 UTM 원점
`(302595.0, 4124145.0)`을 뺀 좌표가 현재 경로의 좌표다.

### yaw가 90도 또는 반대 방향이다

MORAI IMU quaternion의 축 정의와 차량 축 정의를 확인하고 `ekf_local_enu.launch`의
`yaw_offset_deg`를 조정한다. 움직이는 동안 GPS 속도 방향으로도 비교해야 한다.

### Pure Pursuit가 경로를 찾지 못한다

`/localization/odometry`의 `header.frame_id`가 `map`인지, 경로 파일이 실제로 존재하는지,
경로의 첫 점과 추정 위치의 거리가 합리적인지 확인한다.

### 조향 방향이 반대다

`purepursuit_mgeo.launch`의 `steering_sign`을 `-1.0`으로 바꾸기 전에,
MORAI 차량 모델의 조향 필드 단위와 좌우 부호를 저속에서 확인한다.

## 11. 이번 구현의 한계

- EKF는 현재 2D local ENU 상태 `[x, y, yaw, 전진속도, gyro bias, accel bias]`를 사용한다.
- GPS 품질별 공분산이 MORAI 메시지에 없으므로 기본 GPS 분산을 사용한다.
- 차량 제원은 반영했지만 MORAI의 `CtrlCmd.steering` 부호·단위는 저속에서 추가 검증해야 한다.
- 폐루프 가상 차량은 실제 MORAI 차량의 동역학을 대신하지 않는다.
- 카메라 4개와 객체 검출·추적은 이번 단계에서 연결하지 않았다.
- Windows에서는 ROS 노드를 실행할 수 없으므로 `catkin_make`와 실제 토픽 검증은 Ubuntu에서 한다.
