# MORAI Competition Pure Pursuit UDP

이 브랜치의 주행 메인은 `pure_pursuit_interpolated_stable.py`이다.
기존 `EgoVehicleStatus` 예제 코드는 사용하지 않고, 대회용 UDP 네트워크만 사용한다.

## 사용 네트워크

| 항목 | Host Port | Destination Port | 코드에서의 역할 |
|---|---:|---:|---|
| GPS | - | 3001 | NMEA0183 RMC/GGA 수신 |
| IMU | - | 4001 | quaternion, angular velocity, acceleration 수신 |
| Competition Vehicle Status | 9080 | 9081 | `ctrl_mode`, `gear`, 속도, wheelbase, link id 확인 |
| CollisionData | 9091 | 9092 | 충돌 시 강제 brake |
| Ego Ctrl Cmd | 9093 | 9094 | `longCmdType=1` accel/brake/steering 송신 |

현재 IP 기준:

- MORAI Host IP: `192.168.56.1`
- Algorithm PC Destination IP: `192.168.56.101`

MORAI에서 GPS/IMU/Competition Status/CollisionData의 Destination IP는
`192.168.56.101`로 설정한다. Ego Ctrl Cmd는 코드가 `9094 -> 192.168.56.1:9093`
형태로 송신한다.

## 알고리즘 구성

- 횡방향 제어: interpolated Pure Pursuit
- 종방향 제어: PID 기반 accel/brake 제어
- Localization: GPS + IMU + Competition Status signed velocity를 사용하는 15-state INS Error-State EKF
- 차량 제원 기본값: 2023 Hyundai IONIQ 5
  - length `4.635 m`
  - width `1.892 m`
  - wheelbase `3.0 m`
  - front overhang `0.845 m`
  - rear overhang `0.790 m`

Competition Status에서 정상 wheelbase가 들어오면 해당 값을 우선 사용하고,
수신 전/비정상 값일 때는 IONIQ 5 기본값 `3.0 m`를 사용한다.

## 실행 방법

```bash
cd ~/catkin_ws/src/ROI
git checkout dev/pure_pursuit
git pull origin dev/pure_pursuit

cd purepursuit
pip3 install -r requirements.txt
python3 pure_pursuit_interpolated_stable.py
```

ROS launch 파일은 사용하지 않는다.

## 실행 중 확인할 로그

정상 수신 전에는 다음처럼 brake 상태가 뜬다.

```text
BRAKE active: INS not ready/alignment, GPS stale, IMU stale, Competition stale
```

GPS/IMU/Competition Status가 모두 들어오고, MORAI 차량이 `ctrl_mode=2`, `gear=4`
상태가 되면 주행 로그가 출력된다.

```text
ego=( -131.23, -428.10) yaw= 61.20 speed= 3.10/10.0km/h idx=...
```

`ctrl_mode/gear not ready (1/4)`가 계속 뜨면 MORAI에서 `Q`를 눌러
AV-ExternalCtrl 상태로 전환해야 한다.
