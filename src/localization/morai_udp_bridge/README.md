# MORAI GPS·IMU UDP 브릿지

이 패키지는 MORAI SIM: Drive에서 Ubuntu로 전달되는 GPS·IMU UDP 패킷을
파싱하고 ROS 토픽으로 발행한다.

## 출력 토픽

| 센서 | 토픽 | 메시지 |
|---|---|---|
| GPS | `/gps` | `morai_msgs/GPSMessage` |
| IMU | `/Imu` | `sensor_msgs/Imu` |

## 기본 수신 포트

- GPS: `3001`, NMEA `$GPRMC`·`$GPGGA`, 전체 1028바이트
- IMU: `4001`, 현재 NetworkModule 구조체 115바이트
- IMU 레거시 형식: 107바이트도 패킷 길이로 자동 검출

포트와 프로토콜 값은 [udp_localization.yaml](../../../config/udp_localization.yaml)과
[sensor_ports.yaml](../../../config/sensor_ports.yaml)을 기준으로 관리한다.

## 빌드

```bash
cd ~/morai_ws
catkin_make
source devel/setup.bash
```

## 실행

```bash
roslaunch morai_udp_bridge morai_udp_bridge.launch
```

ROS 설치나 `morai_msgs` 빌드 전에 UDP 패킷 자체만 확인하려면 다음 명령을
사용한다.

```bash
cd ~/morai_ws
python3 src/localization/morai_udp_bridge/scripts/morai_udp_dump.py \
  --bind-ip 0.0.0.0 \
  --gps-port 3001 \
  --imu-port 4001
```

이 독립 수신기에서 GPS의 `len=1028`, IMU의 `len=115` 또는 명시적으로 지원한
`len=107`이 확인된 뒤 ROS 브릿지를 실행한다.

## 확인

```bash
rostopic echo /gps
rostopic echo /Imu
rostopic hz /gps
rostopic hz /Imu
```

노드가 출력하는 `packet length`, `layout`, `data_length`를 먼저 확인한다.
IMU 값이 튀거나 패킷이 폐기되면 ROS EKF를 실행하기 전에 길이·timestamp·쿼터니언을
먼저 해결해야 한다.
