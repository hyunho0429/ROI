# MORAI 전역 ENU 경로 CSV 기록기

키보드로 MORAI Ego 차량을 주행하면 `/Ego_topic`의 `position`을 CSV 경로로 저장한다.

시간 주기 대신 차량이 **3차원 공간에서 0.5 m 이동할 때마다** waypoint를 만든다. 토픽 수신 사이에 0.5 m보다 많이 이동해도 선형 보간하므로 저장 간격이 일정하다. 이 방식은 속도에 따라 점 간격이 크게 달라지는 시간 기반 저장보다 Stanley 경로 추종용 기준 경로에 적합하다.

좌표는 별도 변환하지 않는다. MORAI 문서에 따라 `position.x/y/z`를 맵 원점 기준 ENU(`x=east`, `y=north`, `z=up`)로 그대로 기록한다. UTM offset은 임의로 더하지 않는다.

## 빌드 전 확인

ROS 1에서 `morai_msgs`가 보이는지 확인한다.

```bash
rosmsg show morai_msgs/EgoVehicleStatus
```

메시지 패키지가 없다면 catkin workspace의 `src` 아래에 MORAI 공식 메시지 저장소를 설치한 뒤 빌드한다.

```bash
cd ~/my-morai-admodule/src
git clone https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git

cd ~/my-morai-admodule
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

MORAI Network Settings에서 `/Ego_topic`이 발행되도록 설정하고 수신을 확인한다.

```bash
rostopic echo -n 1 /Ego_topic
```

## 실행

```bash
roslaunch path_planning morai_global_csv_recorder.launch
```

기본 출력 파일은 다음과 같다.

```text
src/path_planning/data/morai_global_path.csv
```

첫 메시지는 즉시 저장하고, 이후 누적 3차원 이동거리 `sqrt(dx^2 + dy^2 + dz^2)`가 0.5 m가 될 때마다 보간점을 저장한다. 정지 중에는 중복 행을 만들지 않는다. 모든 행은 즉시 flush하므로 실행 중에도 CSV에 반영된다.

간격이나 파일명을 바꾸려면 다음처럼 실행한다.

```bash
roslaunch path_planning morai_global_csv_recorder.launch \
  output_file:=/home/ubuntu/path/run_01.csv \
  sample_distance:=0.5
```

기존 CSV 뒤에 이어 쓰려면 `append:=true`를 지정한다.

## 주요 CSV 열

- `global_enu_x_m`, `global_enu_y_m`, `global_enu_z_m`: MORAI 맵 원점 기준 ENU 위치
- `heading_deg`: 차량 heading
- `velocity_x_mps`, `velocity_y_mps`, `velocity_z_mps`, `speed_mps`: 차량 속도

`z`는 단순 보관만 하는 것이 아니라 waypoint 간격 계산에도 포함된다. 고저차가 있거나 서로 겹치는 도로의 최근접 경로점을 찾을 때 활용할 수 있다.

## 차량 위치 기준점에 관한 제한

MORAI 26.R1 UDP 문서는 `posX`, `posY`, `posZ`를 단지 "차량의 위치"라고 설명한다. 앞차축 중심, 뒷차축 중심, 차량 무게중심 중 어느 기준인지 명시하지 않으므로 이 기록기는 임의의 축간거리 보정을 적용하지 않는다. Stanley 제어에서 앞차축 위치가 필요하면 차량 모델의 기준점이 확인된 뒤 wheelbase와 heading으로 별도 계산해야 한다.
