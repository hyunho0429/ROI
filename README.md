# MORAI Pure Pursuit 주행 브랜치

`dev/pure_pursuit` 브랜치는 MORAI 25.S4 대회 환경에서 UDP만 사용해 전역 경로를 추종하는 standalone Python 주행 코드이다.

주행 메인 파일은 다음 파일이다.

```bash
purepursuit/pure_pursuit_interpolated_stable.py
```

## 핵심 구성

- 횡방향 제어: interpolated Pure Pursuit
- 종방향 제어: PID 기반 accel/brake 제어
- Localization: GPS + IMU + Competition Vehicle Status signed velocity 기반 INS-EKF
- 제어 송신: Ego Ctrl Cmd UDP, `longCmdType=1`
- 차량 제원 기본값: 2023 Hyundai IONIQ 5

## 현재 네트워크 설정

| 항목 | Host Port | Destination Port |
|---|---:|---:|
| GPS | - | 3001 |
| IMU | - | 4001 |
| Competition Vehicle Status | 9080 | 9081 |
| CollisionData | 9091 | 9092 |
| Ego Ctrl Cmd | 9093 | 9094 |

IP 설정:

- MORAI Host IP: `192.168.56.1`
- Algorithm PC Destination IP: `192.168.56.101`

## 실행 방법

```bash
cd ~/catkin_ws/src/ROI
git checkout dev/pure_pursuit
git pull origin dev/pure_pursuit

cd purepursuit
pip3 install -r requirements.txt
python3 pure_pursuit_interpolated_stable.py
```

ROS launch는 사용하지 않는다.

자세한 내용은 [purepursuit/README.md](purepursuit/README.md)를 참고한다.
