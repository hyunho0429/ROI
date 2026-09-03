# MORAI 터널용 Pure Pursuit 위치추정 실행기

권장 구성은 GPS 음영 구간을 통과하는 `morai_pure_pursuit_ins_udp.py`이다. 비교 시험을
위해 dead-reckoning 실행기도 유지한다. 두 실행기 모두 ROS/catkin 없이 Python과
UDP만으로 실행되며 같은 경로 CSV, Pure Pursuit 조향기, accel/brake PID 제어기와 충돌
제동을 사용한다.

## 공통 준비

```bash
python3 -m pip install -r src/control/path_planning/requirements.txt
```

MORAI 네트워크 포트 예시는 다음과 같다. 실제 Sensor/Network Settings의 포트와
명령행 옵션을 반드시 동일하게 맞춘다.

| UDP 항목 | 방향 | Host Port | Destination Port |
|---|---|---:|---:|
| GPS | SIM → 코드 | 센서 설정값 | 3001 |
| IMU | SIM → 코드 | 센서 설정값 | 4001 |
| Competition Vehicle Status | SIM → 코드 | 9080 | 9081 |
| CollisionData | SIM → 코드 | 9091 | 9092 |
| Ego Ctrl Cmd | 코드 → SIM | 9093 | 9094 |

경로 CSV가 아직 없다면 먼저 키보드로 주행해 기록한다.

```bash
python3 src/control/path_planning/src/morai_gps_csv_recorder.py \
  --port 3001 \
  --output src/control/path_planning/data/morai_global_path.csv \
  --sample-distance 0.5
```

공식 GPS 센서 저장 파일(`latitude, longitude, altitude, eastOffset,
northOffset`) 또는 이 GPS 열과 IMU 열이 합쳐진 CSV를 받는 경우에는 별도 변환
없이 그 파일을 두 실행기의 `--path`로 지정할 수 있다. `x/y/z`와
`origin_lat/origin_lon`이 함께 있으면 로더는 기록 당시 GPS 원점 기준 로컬 ENU를
사용하고 실시간 GPS도 같은 원점으로 변환한다. 공식 GPS 5열 파일은 기존 K-City
MGeo 원점 기준 ENU 변환을 사용한다.

경로는 최소 0.5 m 간격과 9점 이동평균을 적용한다. 횡제어는 속도 비례 lookahead
target과 kinematic bicycle model을 사용하는 Pure Pursuit이다. 경로에
`target_speed`가 없으면 기본 30 km/h를 `longCmdType 1`의 accel/brake PID 제어로
추종한다.

## 1. INS error-state EKF 버전

실행 파일은 `morai_pure_pursuit_ins_udp.py`이다. nominal state는 3차원 위치·속도,
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
python3 src/control/path_planning/src/morai_pure_pursuit_ins_udp.py \
  --path src/control/path_planning/data/morai_global_path.csv \
  --max-gps-outage 120
```

실행 후 기본 2초 동안 코드가 brake를 유지하고 IMU bias alignment를 수행한다.
필요하면 `--alignment-seconds`와 `--alignment-min-samples`를 조정한다. INS 버전은
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

실행 파일은 `morai_pure_pursuit_dead_reckoning_udp.py`이다. IMU 가속도를 적분하지
않고 Competition Status의 signed speed를 IMU 자세의 차량 전방축 방향으로
적분한다.

```text
position_dot = R_body_to_ENU @ [vehicle_speed, 0, 0]
```

따라서 가속도 bias를 두 번 적분하는 INS보다 단순하며, Competition 속도가
안정적으로 제공되는 대회 환경에서는 우선 시험하기 좋은 방식이다. IMU pitch가
전방축에 반영되므로 경사 구간의 Z 변화도 계산한다.

```bash
python3 src/control/path_planning/src/morai_pure_pursuit_dead_reckoning_udp.py \
  --path src/control/path_planning/data/morai_global_path.csv \
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
4. 30 km/h에서 짧은 GPS Blackout 구간을 시험한다.
5. INS와 DR의 터널 출구 위치 오차를 비교한 후 gain/noise를 조정한다.

`Competition Vehicle Status`는 `Ego Vehicle Status`와 같은 패킷 구조를
사용하지만 일부 정보만 제공한다. 현재 181-byte 형식과 229-byte 확장 형식의
header, data length와 tail을 엄격히 검사한다.
25.S4 현장 패킷 길이가 다르면 안전 제동 상태로 거부하고 실제 캡처를 기준으로
별도 레이아웃을 추가해야 한다.
그 외 패킷 길이가 들어오면 오류 로그를 남기고 제어기는 출발하지 않는다.
