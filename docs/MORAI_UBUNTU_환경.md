# MORAI와 Ubuntu 공통 환경

## Ubuntu 준비

Ubuntu에는 ROS1 환경과 beta_drive의 morai_msgs 패키지를 설치한다.
workspace를 빌드한 뒤 source를 적용한다.

    source /opt/ros/noetic/setup.bash
    cd ~/morai_ws
    catkin_make
    source devel/setup.bash

ROS 네이티브 운용에서는 카메라·LiDAR·GPS·IMU의 UDP 수신 포트를
localization이나 detection 노드가 직접 열지 않는다.

## 네트워크

현재 문서의 예시는 MORAI PC 192.168.0.151, Ubuntu PC 192.168.0.200이다.
실제 주소는 다음으로 확인한다.

    ip -4 addr

MORAI Network Settings에서 ROS를 선택하고 rosbridge IP, topic 이름, 주기를
설정한다. 카메라 4대는 같은 기본 topic을 사용하지 말고 센서별 고유 topic을
설정한다.

## ROS 확인

    rostopic list
    rostopic type /gps
    rostopic type /Imu
    rostopic type /Ego_topic
    rostopic type /lidar3D
    rosmsg show morai_msgs/GPSMessage
    rostopic echo /localization/odometry

기준 topic:

- /gps: morai_msgs/GPSMessage
- /Imu: sensor_msgs/Imu
- /Ego_topic: morai_msgs/EgoVehicleStatus
- /lidar3D: sensor_msgs/PointCloud2
- 카메라별 topic: sensor_msgs/CompressedImage
- /ctrl_cmd: morai_msgs/CtrlCmd

## UDP fallback 확인

ROS 네이티브 연결을 사용할 수 없는 실험에서만 다음 포트를 확인한다.

- 카메라 Ubuntu: 1101, 1111, 1121, 1131
- LiDAR Ubuntu: 2001
- GPS Ubuntu: 3001
- IMU Ubuntu: 4001

팀별로 서로 다른 ROS_MASTER_URI, topic, 메시지 이름을 만들지 않는다.
여러 PC에서 ROS를 사용할 경우에만 팀이 합의한 ROS_MASTER_URI와 ROS_IP를
사용한다.

## 기본 확인 순서

1. MORAI가 Play 상태인지 확인한다.
2. rostopic type으로 메시지 타입을 확인한다.
3. 카메라 4대와 LiDAR topic을 각각 확인한다.
4. GPS와 IMU의 timestamp·frame_id·주기를 확인한다.
5. /Ego_topic은 정렬 검증에만 사용한다.
6. Localization 상태를 확인한다.
7. 마지막에 Control을 실행한다.

센서가 들어오지 않을 때는 실행 코드를 수정하기 전에 ROS bridge IP,
topic 이름, rosnode 상태, frame_id, timestamp를 먼저 확인한다.

