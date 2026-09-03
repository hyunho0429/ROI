# MORAI UDP-only GPS 경로 CSV 기록기

`morai_gps_csv_recorder.py`는 대회에서 허용된 GPS UDP만으로 수동 주행 경로를
기록한다. `Ego Vehicle Status`나 ROS 토픽은 사용하지 않는다.

## 왜 GPS와 ENU를 함께 저장하는가

Pure Pursuit 계산에는 metre 단위 직교좌표가 필요하므로 ENU `x/y/z`를 저장한다.
동시에 원본 위도·경도·고도를 보존하면 좌표 변환 설정을 검증하거나 다른 맵
원점으로 다시 변환할 수 있다. 한 행의 주요 열은 다음과 같다.

- `latitude_deg`, `longitude_deg`, `altitude_m`: GPS 원본
- `global_enu_x_m`, `global_enu_y_m`, `global_enu_z_m`: Pure Pursuit 입력
- `projection_crs`: 기본 K-City `EPSG:32652`
- `east_offset_m`, `north_offset_m`, `up_offset_m`: 고정 맵 원점
- `speed_mps`, `velocity_x_mps`, `velocity_y_mps`: RMC에 값이 있을 때의 진단값

과거 IMU quaternion, 각속도, 가속도는 경로 형상이 아니며 추종 시 현재 센서
대신 쓸 수 없으므로 CSV에 저장하지 않는다. 주행 시에는 실시간 GPS/IMU와
Competition Vehicle Status가 EKF-INS에 들어간다.

## EastOffset/NorthOffset 주의

MORAI ROS `GPSMessage`에는 `eastOffset/northOffset`이 있지만, UDP GPS는
NMEA0183 RMC/GGA라 이 두 상수를 보내지 않는다. UDP-only 환경에서는 저장소의
K-City MGeo `global_info.json`에서 읽거나 아래 실행 옵션으로 지정한다.

```bash
python3 src/control/path_planning/src/morai_gps_csv_recorder.py \
  --port 3001 \
  --output src/control/path_planning/data/morai_global_path.csv \
  --sample-distance 0.5
```

다른 맵이면 다음처럼 좌표계를 명시한다.

```bash
python3 src/control/path_planning/src/morai_gps_csv_recorder.py \
  --global-info /path/to/map/global_info.json \
  --utm-crs EPSG:32652 \
  --utm-origin-x 302459.942 \
  --utm-origin-y 4122635.537 \
  --utm-origin-z 28.919
```

기본은 수평 이동거리 0.5 m마다 한 점을 저장한다. `--sample-period 0.1`을 함께 주면
거리 조건과 최소 시간 조건을 모두 만족할 때만 저장한다. 정지 중 중복점은
저장하지 않는다. `Ctrl+C`로 종료하면 마지막 행까지 flush/fsync하고 닫는다.

기록한 파일은 그대로 추종기에 넣는다.

```bash
python3 src/control/path_planning/src/morai_pure_pursuit_ins_udp.py \
  --path src/control/path_planning/data/morai_global_path.csv
```
