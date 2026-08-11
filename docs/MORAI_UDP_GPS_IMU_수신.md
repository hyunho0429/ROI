# MORAI GPS·IMU UDP 수신 절차

## 1. 이번 구현의 기준

제공된 MORAI 예제의 처리 흐름은 다음과 같다.

1. UDP 소켓을 Ubuntu 수신 주소와 포트에 bind한다.
2. GPS는 `$GPRMC` 또는 `$GPGGA` header를 확인한다.
3. IMU는 고정 길이 binary packet을 구조체로 해석한다.
4. 파싱한 값을 ROS 메시지로 변환한다.

이번 구현은 공식 예제처럼 `ctypes` native alignment에 의존하지 않고,
Ubuntu x86에서 명시적 little-endian으로 읽는다. 따라서 구조체 padding 때문에
필드가 밀리는 문제를 줄일 수 있다.

## 2. GPS 패킷

GPS는 MORAI 예제의 구조체 기준으로 받되, 시뮬레이터 버전에 따라 패킷 전체가
1028바이트로 패딩되거나 NMEA 문장 길이 그대로(현재 확인값 77바이트) 들어오는
두 경우를 모두 허용한다.

```text
offset 0       : header 6 bytes, $GPRMC 또는 $GPGGA
offset 6       : NMEA data 1022 bytes
전체           : 1028 bytes(NUL padding 포함) 또는 NMEA 문장 실제 길이
```

NMEA의 위도·경도는 `ddmm.mmmm` 형식이므로 decimal degree로 변환한다.
`S`, `W` 방향은 음수로 바꾼다. checksum이 있으면 XOR 검증 후 오류 패킷을
폐기한다.

`GPGGA`는 고도와 fix quality를 포함하므로 고도는 최신 GGA 값을 사용한다.
`GPRMC`가 먼저 도착하면 고도는 직전 GGA 값이 없을 때만 0.0으로 발행된다.

UDP GPS 패킷에는 MGeo map offset이 포함되지 않는다. 따라서 `eastOffset`,
`northOffset`는 [udp_localization.yaml](../config/udp_localization.yaml)에
실험으로 입력하며, 실제 GPS → map 정렬이 끝나기 전에는 임의의 값으로 EKF를
초기화하지 않는다.

## 3. IMU 패킷

MORAI NetworkModule의 `lib/define/IMU.py` 구조체는 다음 순서이다.

```text
header[9]
data_length: int
aux_data[3]: int
sec: int
nsec: int
ori_w, ori_x, ori_y, ori_z: double
ang_vel_x, ang_vel_y, ang_vel_z: double
lin_acc_x, lin_acc_y, lin_acc_z: double
tail[2]
```

위 구조체는 little-endian 기준 전체 115바이트이고, 데이터부 double 10개는
80바이트이다. MORAI 버전에 따라 `data_length` 필드가 80 또는 88로 들어올 수
있지만, 실제 double 데이터 배치는 동일하다. 일부 23.R1 문서에는 전체 107바이트로 적혀 있으므로 구현은
보조 정수 1개만 있는 107바이트 레거시 형식도 길이로 구분해 지원한다.

ROS `sensor_msgs/Imu`로 변환할 때 MORAI의 `(w, x, y, z)`를 ROS 필드
`orientation.x/y/z/w`에 맞게 재배치한다. 각속도는 rad/s, 선가속도는 m/s²로
그대로 발행한다.

## 4. MORAI와 Ubuntu 설정

MORAI PC의 GPS·IMU Sensor Network 설정에서 Ubuntu PC의 IP와 아래 포트를
목적지로 지정한다.

| 센서 | Ubuntu 수신 포트 | ROS 토픽 |
|---|---:|---|
| GPS | 3001 | `/gps` |
| IMU | 4001 | `/Imu` |

카메라 포트 `1100/1110/1120/1130 → 1101/1111/1121/1131`와는 별개이다.
GPS·IMU 포트가 실제 대회 PC 설정과 다르면 launch 인자로 덮어쓴다.

```bash
roslaunch morai_udp_bridge morai_udp_bridge.launch \
  bind_ip:=0.0.0.0 \
  gps_port:=3001 \
  imu_port:=4001
```

## 5. 패킷 수신 확인 순서

```bash
rostopic list | grep -E '^/(gps|Imu)$'
rostopic hz /gps
rostopic hz /Imu
rostopic echo -n 1 /gps
rostopic echo -n 1 /Imu
```

확인할 항목은 다음과 같다.

- GPS 위도·경도가 0이 아니고 MORAI 차량 위치와 같은 지역인지
- GPS GGA 고도가 지속적으로 갱신되는지
- IMU quaternion norm이 1에 가까운지
- IMU 각속도 단위가 rad/s인지
- 정지 상태에서 선가속도가 센서 축 기준으로 안정적인지
- 노드 로그의 IMU packet layout이 실제 송신 형식과 일치하는지

이 단계에서는 아직 EKF를 실행하지 않는다. UDP 원시 패킷 길이와 ROS 변환값을
먼저 확인한 다음, GPS → MGeo map 정렬과 시간 동기화를 검증한다.
