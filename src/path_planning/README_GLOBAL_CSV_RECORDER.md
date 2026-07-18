# MORAI 24.R1 UDP 전역 ENU 경로 CSV 기록기

MORAI Ego 차량의 **UDP Ego Vehicle Status** 패킷을 직접 수신해 경로 CSV를 만든다. ROS, `roscore`, `rospy`, `morai_msgs`가 필요하지 않다.

이 구현은 [MORAI SIM: Drive 24.R1.0 Ego Vehicle Status UDP 프로토콜](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R1.0/ros-1#id-(24.R1.0-ko)%ED%86%B5%EC%8B%A0%EB%A9%94%EC%8B%9C%EC%A7%80%ED%94%84%EB%A1%9C%ED%86%A0%EC%BD%9C-EgoVehicleStatus.1)의 181 byte 패킷만 허용한다.

- Header: `#MoraiInfo$`
- `data_length`: 152
- 전체 패킷: 181 byte
- Timestamp: seconds 4 byte + nanoseconds 4 byte
- Tail: `0x0D 0x0A`

다른 버전의 패킷은 잘못된 좌표로 저장하지 않고 오류 메시지와 함께 무시한다.

## MORAI 설정

MORAI의 `Network Settings → Ego Network → UDP → Publisher`에서 `Ego Vehicle Status`를 활성화한다.

- Destination IP: 기록기를 실행하는 PC의 IP
- Destination Port: 기록기의 `--port`와 동일한 값(예: `909`)
- 동일 PC에서 실행하면 IP로 `127.0.0.1` 사용 가능
- 다른 PC, VM 또는 WSL이면 `hostname -I`로 실제 수신 IP 확인

포트에 패킷이 들어오는지는 다음처럼 확인할 수 있다.

```bash
sudo tcpdump -ni any udp port 909
```

## 실행

저장소 최상위 폴더에서 실행한다.

```bash
python3 src/path_planning/src/morai_global_csv_recorder.py \
  --bind-ip 0.0.0.0 \
  --port 909
```

기본 출력 파일:

```text
src/path_planning/data/morai_global_path.csv
```

출력 파일과 waypoint 간격을 지정할 수도 있다.

```bash
python3 src/path_planning/src/morai_global_csv_recorder.py \
  --bind-ip 0.0.0.0 \
  --port 909 \
  --output ~/morai_paths/run_01.csv \
  --sample-distance 0.5
```

종료는 `Ctrl+C`를 사용한다. 기존 CSV에 이어 쓰려면 `--append`를 추가한다. 열 구성이 다른 과거 ROS CSV에는 실수로 이어 쓰지 못하도록 차단한다.

## 저장 방식

첫 위치는 즉시 저장한다. 이후 누적 3차원 이동거리 `sqrt(dx^2 + dy^2 + dz^2)`가 0.5 m가 될 때마다 선형 보간점을 저장한다. 평지에서는 XY 이동만으로 저장되고, 오르막·내리막에서는 Z 변화도 전체 이동거리에 포함된다.

UDP 패킷의 `Velocity_XYZ`와 signed velocity는 문서상 km/h이므로 CSV에는 m/s로 변환해 기록한다.

주요 CSV 열:

- `global_enu_x_m`, `global_enu_y_m`, `global_enu_z_m`: MORAI 맵 원점 기준 ENU 위치
- `roll_deg`, `pitch_deg`, `heading_deg`: UDP 차량 회전 정보
- `velocity_x_mps`, `velocity_y_mps`, `velocity_z_mps`, `speed_mps`: m/s 변환 속도
- `wheelbase_m`, `overhang_m`, `rear_overhang_m`: UDP 차량 제원
- `ctrl_mode`, `gear`, `map_data_id`, `link_id`: UDP 상태 정보

`message_time_sec`은 UDP 패킷 timestamp다. MORAI 문서에 따르면 일반 모드에서는 Unix timestamp이고 Sync Mode에서는 시뮬레이터 시작 시간 기준이다. `receive_time_sec`은 기록기 PC에서 패킷을 받은 Unix 시간이다.

## 차량 위치 기준점 제한

24.R1 문서는 `posX`, `posY`, `posZ`를 차량 위치라고 설명하지만 앞차축·뒷차축·무게중심 중 어느 기준인지는 명시하지 않는다. 따라서 wheelbase가 패킷에 포함되어도 임의의 앞차축 보정은 적용하지 않는다.
