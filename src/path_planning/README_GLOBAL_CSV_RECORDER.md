# MORAI Competition Vehicle Status 경로 CSV 기록기

`morai_global_csv_recorder.py`는 ROS나 catkin 없이 `Competition Vehicle
Status` UDP 패킷의 맵 원점 기준 ENU 위치를 CSV로 저장한다. 이 인터페이스는
공개 문서의 `Ego Vehicle Status`와 별개로 취급한다.

현재 파서는 실제 25.01에서 확인된 181-byte 형식(`data_length=152`)과 대회용
229-byte 확장 형식(`data_length=200`)을 지원한다.
패킷 길이, `#MoraiInfo$` 헤더, data length 또는 CRLF tail이 다르면 좌표를
임의로 해석해 저장하지 않고 실제 수신 길이가 포함된 오류를 출력한다.

## 실행

MORAI의 `Network Settings > Ego-0 > Publisher, Subscriber, Service`에서
`Competition Vehicle Status`의 Destination IP를 기록기 PC 주소로,
Destination Port를 아래 `--port`와 같게 설정한다.

```bash
python3 src/path_planning/src/morai_global_csv_recorder.py \
  --bind-ip 0.0.0.0 \
  --port 3315 \
  --output src/path_planning/data/morai_global_path.csv \
  --sample-distance 0.5
```

첫 위치는 즉시 저장한다. 이후에는 수신 시간이나 차량 속도와 무관하게 누적
3차원 거리 `sqrt(dx² + dy² + dz²)`를 기준으로 0.5 m 간격의 점을 선형
보간해 저장한다. 평지에서는 XY 이동으로 저장되고 경사 구간에서는 Z도 거리
계산에 반영된다. Z가 일정량 이상 변해야 저장되는 방식은 아니다.

주요 열은 다음과 같다.

- `global_enu_x_m`, `global_enu_y_m`, `global_enu_z_m`: 맵 원점 기준 ENU
- `roll_deg`, `pitch_deg`, `heading_deg`: 차량 자세
- `velocity_x_mps`, `velocity_y_mps`, `velocity_z_mps`: m/s 속도
- `wheelbase_m`, `overhang_m`, `rear_overhang_m`: 차량 제원
- `ctrl_mode`, `gear`, `map_data_id`, `link_id`: 상태 정보

종료는 `Ctrl+C`를 사용한다. 기존 CSV에 이어 쓰려면 헤더가 정확히 같은 경우에만
`--append`를 사용한다.

Competition Status의 위치 기준점이 앞차축, 뒤차축 또는 무게중심 중 어디인지는
확인된 공개 규격이 없다. 따라서 기록 단계에서는 임의의 앞차축 보정을 적용하지
않는다. Stanley 실행 시 GPS 장착점에서 앞차축까지의 전방 거리를 실측한 경우에만
`--control-point-offset`으로 보정한다.
