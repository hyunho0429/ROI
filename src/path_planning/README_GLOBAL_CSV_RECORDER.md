# MORAI Ego Vehicle Status 경로 CSV 기록기 (비대회 진단용)

> 대회 허용 네트워크 목록에는 `Ego Vehicle Status`가 없으므로 이 기록기는
> 대회 경로 생성에 사용하지 않는다. UDP-only 대회 경로는
> [README_GPS_CSV_RECORDER.md](README_GPS_CSV_RECORDER.md)의 GPS 기록기를 쓴다.

`morai_global_csv_recorder.py`는 ROS나 catkin 없이 공식 181-byte `Ego Vehicle
Status` UDP 패킷의 위치를 CSV로 저장한다. Ego Status의 `position`은 맵 원점
기준 ENU이므로 GPS, UTM 변환이나 속도 적분이 필요하지 않다.

이 도구는 공개 시뮬레이터의 비대회 디버깅과 좌표 대조에만 사용한다. 실제 대회
경로 생성과 자율주행은 허용된 `Competition Vehicle Status`, GPS, IMU를 사용한다.

## MORAI 네트워크 설정

`Network Settings > Ego-0 > Publisher, Subscriber, Service`에서 다음처럼
설정한다.

- Message: `Ego Vehicle Status`
- Destination IP: 기록기를 실행하는 Ubuntu VM의 IP
- Destination Port: 예시 `9102`

포트는 실행 명령의 `--port`와 같아야 한다. `9102`처럼 1024보다 큰 포트를
사용하면 Linux에서 `sudo` 없이 실행할 수 있다.

## 실행

```bash
python3 src/path_planning/src/morai_global_csv_recorder.py \
  --bind-ip 0.0.0.0 \
  --port 9102 \
  --output src/path_planning/data/morai_global_path.csv \
  --sample-period 1.0
```

첫 위치는 즉시 저장하고 이후에는 차량 이동 여부와 관계없이 기본 1초마다 저장한다.
곡선 구간을 더 세밀하게 기록하려면 `--sample-period 0.1`처럼 주기를 줄인다.

정상 수신 시 다음처럼 0이 아닌 ENU 좌표가 출력된다.

```text
saved #1: (123.456, -78.901, 28.321)
saved #2: (124.802, -78.315, 28.326)
```

주요 CSV 열은 다음과 같다.

- `global_enu_x_m`, `global_enu_y_m`, `global_enu_z_m`: 맵 원점 기준 ENU 위치
- `roll_deg`, `pitch_deg`, `heading_deg`: 차량 자세
- `velocity_x_mps`, `velocity_y_mps`, `velocity_z_mps`: m/s 속도
- `wheelbase_m`, `overhang_m`, `rear_overhang_m`: 차량 제원
- `ctrl_mode`, `gear`, `map_data_id`, `link_id`: 차량 및 맵 상태

`(0,0,0)`이 반복되면 Competition Vehicle Status가 해당 포트로 들어오는지
확인한다. 기록기는 잘못된 0좌표를 CSV에 추가하지 않는다. 종료는 `Ctrl+C`를
사용한다.

Ego Status 위치가 앞차축, 뒤차축 또는 무게중심 중 어디를 기준으로 하는지는 공개
UDP 규격만으로 확인할 수 없다. 기록 단계에서 임의의 앞차축 보정을 적용하지 않고,
Pure Pursuit 실행 시 기준점 거리를 확인한 경우에만 `--control-point-offset`을 사용한다.
