# 고속도로 추월(끼어들기) 플래너 — highway_overtake

기존 `highway_lane_strategy_node`를 대체하는 새 고속도로 노드입니다.
고정된 고속도로 구간 안에서 **왼쪽 차로로 한 차로씩, 최대 3번** 옮겨 가며
옆 차로 차량을 앞지르자마자 그 앞으로 들어가고, 마지막 차로(왼쪽이 실선)에
도착하면 앞차 간격만 맞추며 최대한 빨리 달린 뒤 구간 끝에서 전역 경로로
넘겨줍니다.

## 1. 동작

```
OFF ── 구간 시작점 통과(지도 기준 플래그) ──▶ WAIT_LANE
WAIT_LANE  카메라 차선이 처음 측정되면 ──▶ CHASE
CHASE      바로 왼쪽 선이 점선이고 변경 횟수 < 3 이면 매 주기 간격 판단
           최고속도로 달리다가 왼쪽 차를 조금이라도 앞지르면 바로 ──▶ CHANGE
CHANGE     진입각 5° 이하·횡가속도 1.5 m/s² 이하의 고정 경로로 한 차로 이동
           (도중에 카메라 값으로 경로를 바꾸지 않음) ──▶ REACQUIRE
REACQUIRE  새 차로의 양쪽 선이 측정되는 순간 ──▶ CHASE (다음 변경 시도)
           변경 3회 또는 바로 왼쪽 선이 실선이면 ──▶ FOLLOW
FOLLOW     차로 중앙 유지 + 앞차 간격 유지 + 구간 끝 감속(54 km/h)
DONE       구간 끝점에서 전역 경로로 넘겨줌
```

* **기준선(Frenet)**: 카메라 `lane_info`의 왼쪽 선 방향과 두 선의 중앙을 map
  좌표로 고정해 s(차로 방향)·d(왼쪽 +) 좌표계를 만듭니다. 오른쪽 선이 평행하지
  않으면(진입차로 테이퍼) 왼쪽 선만 씁니다. 차로 변경 중에는 이 좌표계를 얼려 둡니다.
* **주변 차량**: LiDAR 추적 결과를 이 좌표계로 옮겨 차로 번호 + 등속 모델로
  봅니다(차로 경계에서 번호가 흔들리지 않게 0.4 m 히스테리시스).
* **간격 판단(공격적)**: 차로를 옮기는 동안 목표 차로 차와 옆으로 겹치는 순간마다
  * 뒤차: 우리가 더 빠르면 범퍼 간격 **1 m**면 통과, 뒤차가 더 빠르면
    1 m + 0.5 s 차간 + 속도를 맞추는 제동거리(3 m/s²)
  * 앞차: 앞차가 더 빠르면 **3 m**, 더 느리면 3 m + 0.5 s 차간 + 제동거리
  * 측정 속도 ±0.5 m/s 오차와 가속 프로파일 2가지를 모두 검사
* **급정지**: 경로상 앞차와 간격 1.5 m 미만 또는 TTC 1 s 미만이고, 그 차와
  가까워지는 중일 때만.
* **진입차로**: 차로 변경 경로가 좁아지는 오른쪽 실선(테이퍼)에 앞바퀴가 닿지
  않는 범위에서 가장 빠른 설계 속도를 고릅니다. 합류를 못 하면 멈추지 않습니다
  (사용자 결정).

## 2. 고정 구간 (MGeo, `config/highway_overtake.yaml`)

| 항목 | 위치 | 전역 경로 s |
|---|---|---|
| 구간 시작 `zone_start_xy` | (62.43, 215.00) | ~1160 m |
| 진입차로 테이퍼 시작 | (63.01, 160.00) | ~1215 m |
| 진입차로 테이퍼 끝 | (66.90, 88.20) | ~1287 m |
| 구간 끝 `zone_end_xy` | (75.03, -214.54) | ~1590 m (전역 경로가 L3 중심과 만나는 곳) |

차로: 진입차로 → L1 → L2 → L3(왼쪽이 실선, 도착 차로), 실선 너머 L4는 사용하지 않음.

## 3. 파일

| 파일 | 내용 |
|---|---|
| `src/control/purepursuit_mgeo/src/purepursuit_mgeo/highway_overtake.py` | 판단 로직 전부 (ROS 없음) |
| `src/control/purepursuit_mgeo/scripts/highway_overtake_node.py` | ROS 입출력만 |
| `src/control/purepursuit_mgeo/config/highway_overtake.yaml` | 구간 좌표·파라미터 |
| `src/control/purepursuit_mgeo/launch/morai_highway_overtake.launch` | 최종 주행 스택 + 이 노드 |
| `src/control/purepursuit_mgeo/tools/highway_overtake_sim.py` | 오프라인 시뮬레이터 + 재생 뷰어 |
| `src/control/purepursuit_mgeo/test/test_highway_overtake*.py` | 단위·노드 연결 테스트 |

`morai_avoidance_highway_roundabout_final.launch`에는 인자 두 개만 추가했습니다
(`highway_ns`, `enable_highway_lane_strategy`). 기본값이면 예전과 똑같이 동작합니다.

## 4. 오프라인 시뮬레이터 (ROS 불필요)

실제 고속도로 구간과 같은 차로 배치, 샘플 시나리오(`data/scenarios/2026_molit_comp_sample_scene.json`)의
고속도로 교통(L1 40 / L2 50 / L3 60 km/h, 10초 간격, 차로 변경 없음, NPC는 앞차와
5 m·1 s 유지)을 매번 ±10 % 속도·±15 % 간격·무작위 위상으로 만들어 돌립니다.
카메라·LiDAR 출력, Pure Pursuit(조향 제한 포함), 차량 동역학을 흉내 내고 충돌·실선
접촉·횡가속도·구간 끝 넘겨주기를 채점합니다.

```bash
cd src/control/purepursuit_mgeo
python tools/highway_overtake_sim.py                              # 샘플 교통 10판
python tools/highway_overtake_sim.py --scenario all --seeds 10    # 샘플 / 카메라 잡음 / 빈 도로
python tools/highway_overtake_sim.py --scenario morai --seed 1 --view       # 재생 창
python tools/highway_overtake_sim.py --scenario morai --seed 1 --gif out.gif
python tools/highway_overtake_sim.py --set max_speed_mps=25       # 파라미터 바꿔 보기
```

재생 창: Space 일시정지, ←/→ 1초 이동, ↑/↓ 재생 속도. 빨간 차 = 차로 변경을 막고 있는
차, 주황 차 = 따라가는 앞차, 자홍 점 = 플래너 경로.

현재 결과(`--scenario all --seeds 10`): 30/30 통과, 매번 3회 변경, 충돌·실선 접촉·급정지 0,
구간(430 m) 통과 21.2~33.8 s(평균 27.9 s), 구간 끝 속도 약 55 km/h.

## 5. ROS 실행

```bash
cd ~/ROI
git fetch origin && git switch test/rrt_jy && git pull
source /opt/ros/noetic/setup.bash
catkin_make && source devel/setup.bash
roslaunch purepursuit_mgeo morai_highway_overtake.launch morai_host_ip:=<MORAI PC IP>
```

| launch 인자 | 기본 | 의미 |
|---|---|---|
| `target_speed_mps` | 15.0 | 구간 밖 속도 (60 km/h 미만 유지) |
| `overtake_max_speed_mps` | 22.0 | 구간 안 최고속도 |
| `enable_control` | true | false면 명령을 보내지 않음 |
| `enable_lane_debug_view` | true | 차선 창에 경로 점·상태 표시 |

## 6. 시뮬레이터에서 확인할 것

```bash
rostopic echo /highway_overtake/zone_active            # 구간 플래그
rostopic echo /highway_overtake/state                  # 상태·판단 이유(JSON)
rosrun rqt_console rqt_console                         # HIGHWAY_OVERTAKE 로그
```

* 로그 `HIGHWAY_OVERTAKE zone entered`, `LANE CHANGE 0->1 committed`, `lane change complete`,
  `new lane N measured`, `final lane reached`, `hand-over to global path at zone end (offset ...)`
  이 순서대로 나오는지.
* 넘겨줄 때 offset이 1 m 이내인지(`WARNING: not on the global path's lane`이 없어야 함).
* 차선 창(lane overlay): 자홍 점이 실제 차로 위에 그려지는지, 변경 중 주황색 경로.
* RViz: `/highway_overtake/active_path`(Path), `/highway_overtake/markers`(MarkerArray).
* `state`의 `reason`이 오래 `lane_not_fresh` / `left_not_dashed`이면 카메라 문제,
  `gap_target_rear` / `gap_target_front`이면 교통 때문에 기다리는 중.

기록해서 보내 주면 오프라인으로 그대로 분석할 수 있습니다.

```bash
rosbag record -O highway_overtake.bag /localization/odometry /perception/lidar/tracked_obstacles_map \
  /perception/camera/lane_info /highway_overtake/state /highway_overtake/active_path \
  /avoidance_path_manager/active_path /ctrl_cmd
```

## 7. 알려진 한계

* MORAI에서 아직 돌려 보지 않았습니다. 시뮬레이터의 Pure Pursuit·차량 응답은 근사입니다.
* 진입차로에서 L1 차와 같은 속도로 나란히 달려 끝까지 앞지르지 못하면 진입차로가 끝나도
  멈추지 않습니다(사용자 결정). 샘플 교통(L1 40 km/h, 자차 54 km/h)에서는 발생하지 않았습니다.
* L3 교통이 최고속도(79 km/h) 이상이면 앞지르지 못해 L2에서 구간 끝에 도달할 수 있고, 이때
  전역 경로로 넘겨주며 크게 꺾습니다(로그에 WARNING). 최고속도를 올리면 줄어듭니다.
* 카메라 timestamp는 wall clock 기준이라 rosbag을 `--clock`(sim time)으로 재생할 때는
  차선 정보가 오래된 것으로 처리될 수 있습니다.
