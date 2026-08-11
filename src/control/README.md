# Control 팀

MGeo 해석, 대회 경로 생성, 경로 인덱싱, 속도·조향 제어, 안전정지
우선순위, MORAI CtrlCmd 송신을 담당한다.

## 입력과 출력

입력:

- /localization/odometry
- /control/reference_path
- /detection/obstacles
- /detection/safety_stop

출력:

- /ctrl_cmd: morai_msgs/CtrlCmd

내부 제어 출력은 /control/ctrl_cmd로 관리할 수 있지만, MORAI로 보내는
최종 topic은 /ctrl_cmd로 통일한다.

beta_drive CtrlCmd의 필드명은 longlCmdType, accel, brake, steering,
velocity, acceleration이다. steering은 rad 기준으로 처리한다.

## 예정 패키지

- morai_map_path
- morai_route_manager
- morai_controller
- morai_ctrl_cmd_bridge

Control만 최종 CtrlCmd를 발행한다. 카메라·LiDAR·GPS·IMU 패킷을 직접
파싱하지 않고, localization과 detection의 표준 topic을 구독한다.

