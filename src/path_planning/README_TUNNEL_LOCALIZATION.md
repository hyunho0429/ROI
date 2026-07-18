# MORAI 터널용 Stanley 위치추정 실행기

이 브랜치에는 GPS 음영 구간을 통과하기 위한 두 개의 독립 실행기가 있다. 두
실행기 모두 ROS/catkin 없이 Python과 UDP만으로 실행되며 같은 경로 CSV, Stanley
조향기, accel/brake PI 제어기와 충돌 제동을 사용한다.

## 공통 준비

```bash
python3 -m pip install -r src/path_planning/requirements.txt
```

MORAI 네트워크 포트 예시는 다음과 같다. 실제 Sensor/Network Settings의 포트와
명령행 옵션을 반드시 동일하게 맞춘다.

| UDP 항목 | 방향 | 기본 포트 |
|---|---|---:|
| GPS | SIM → 코드 | 9100 |
| IMU | SIM → 코드 | 9101 |
| Competition Vehicle Status | SIM → 코드 | 3315 |
| CollisionData | SIM → 코드 | 5678 |
| Ego Ctrl Cmd | 코드 → SIM | 9090 |

경로 CSV가 아직 없다면 먼저 키보드로 주행해 기록한다.

```bash
python3 src/path_planning/src/morai_global_csv_recorder.py \
  --port 3315 \
  --output src/path_planning/data/morai_global_path.csv
```

## 1. INS error-state EKF 버전

실행 파일은 `morai_stanley_ins_udp.py`이다. nominal state는 3차원 위치·속도,
body-to-ENU quaternion, gyro bias와 accelerometer bias이고, 15상태 오차 벡터는
`[dp, dv, dtheta, dbg, dba]`이다.

- IMU 각속도·specific force를 strapdown mechanization에 사용
- GPS 위치·RMC 속도로 위치, 속도와 bias 보정
- IMU quaternion으로 자세 오차 보정
- Competition 속도로 body 전방 속도 보정
- body 횡속도·수직속도가 0에 가깝다는 NHC 적용
- 정지 상태의 Competition 속도를 이용한 초기 gyro/accelerometer bias 보정
- GPS blackout 중에도 IMU + Competition 속도 + NHC로 예측 지속

```bash
python3 src/path_planning/src/morai_stanley_ins_udp.py \
  --path src/path_planning/data/morai_global_path.csv \
  --gps-port 9100 \
  --imu-port 9101 \
  --competition-status-port 3315 \
  --collision-port 5678 \
  --control-ip 127.0.0.1 \
  --control-port 9090 \
  --target-speed-kmh 10 \
  --max-gps-outage 120
```

실행 후 차량을 3–5초간 정지시켜 초기 bias가 수렴할 시간을 준다. INS 버전은
IMU 선가속도가 정지 시 body +Z 방향으로 약 `+9.80665 m/s²`를
출력하는 specific-force 규약을 전제로 한다. 첫 시험에서는 반드시 정지 상태에서
로그의 Z가 빠르게 변하지 않는지 확인한다. 반대 부호 또는 중력 제거된 가속도가
들어오면 mechanization 모델을 해당 센서 규약에 맞게 바꿔야 한다.

주요 튜닝 옵션:

- `--accel-noise-sigma`: accelerometer white-noise 표준편차
- `--gyro-noise-sigma-degps`: gyro white-noise 표준편차
- `--accel-bias-walk-sigma`: accelerometer bias random walk
- `--gyro-bias-walk-sigma-degps`: gyro bias random walk
- `--nhc-lateral-sigma`, `--nhc-vertical-sigma`: 차량 운동 제약 강도

## 2. Competition 속도 기반 dead-reckoning 버전

실행 파일은 `morai_stanley_dead_reckoning_udp.py`이다. IMU 가속도를 적분하지
않고 Competition Status의 signed speed를 IMU 자세의 차량 전방축 방향으로
적분한다.

```text
position_dot = R_body_to_ENU @ [vehicle_speed, 0, 0]
```

따라서 가속도 bias를 두 번 적분하는 INS보다 단순하며, Competition 속도가
안정적으로 제공되는 대회 환경에서는 우선 시험하기 좋은 방식이다. IMU pitch가
전방축에 반영되므로 경사 구간의 Z 변화도 계산한다.

```bash
python3 src/path_planning/src/morai_stanley_dead_reckoning_udp.py \
  --path src/path_planning/data/morai_global_path.csv \
  --gps-port 9100 \
  --imu-port 9101 \
  --competition-status-port 3315 \
  --collision-port 5678 \
  --control-ip 127.0.0.1 \
  --control-port 9090 \
  --target-speed-kmh 10 \
  --max-gps-outage 120
```

주요 튜닝 옵션:

- `--dr-position-drift-sigma`: GPS가 없을 때 위치 불확실성 증가율
- `--orientation-correction-gain`: gyro 적분 자세를 IMU quaternion 쪽으로 보정하는 비율
- `--gyro-bias-gain`: quaternion 잔차로 gyro bias를 보정하는 비율

## 안전 동작과 시험 순서

두 실행기는 다음 조건에서 즉시 brake를 전송한다.

- IMU가 기본 0.5초 이상 끊김
- Competition Status가 기본 0.5초 이상 끊김
- 유효 GPS를 한 번도 받지 못했거나 `--max-gps-outage`를 초과함
- CollisionData에서 충돌 감지
- 경로 끝 도달 또는 `Ctrl+C`

GPS Blackout의 `(0,0,0)` NMEA 위치는 유효한 측정으로 사용하지 않는다. GPS가
복구되면 증가한 공분산을 이용해 위치를 다시 보정한다.

시험은 다음 순서를 권장한다.

1. 외부 제어를 끄고 네 UDP 수신 로그와 패킷 길이를 확인한다.
2. 차량 정지 상태에서 INS Z drift와 DR 위치 정지를 확인한다.
3. 직선에서 5 km/h로 조향 부호와 ENU 축을 확인한다.
4. 10 km/h에서 짧은 GPS Blackout 구간을 시험한다.
5. INS와 DR의 터널 출구 위치 오차를 비교한 후 gain/noise를 조정한다.

`Competition Vehicle Status`는 공개 `Ego Vehicle Status`와 다른 패킷이며 현재
대회용 229-byte 형식을 엄격히 검사한다. 실제 25.01 패킷 길이가 다르면 오류
로그를 남기고 제어기는 출발하지 않는다.
