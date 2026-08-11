# Detection 팀

카메라 4대와 VLP16 LiDAR의 ROS 입력, 센서 보정, 차선·신호·장애물 인식,
카메라·LiDAR 융합, 안전정지 근거를 담당한다.

## 공식 입력

- /camera/front/image/compressed: sensor_msgs/CompressedImage
- /camera/left/image/compressed: sensor_msgs/CompressedImage
- /camera/right/image/compressed: sensor_msgs/CompressedImage
- /camera/aux/image/compressed: sensor_msgs/CompressedImage
- /lidar3D: sensor_msgs/PointCloud2

MORAI 카메라 공식 기본 topic은 /imag_jpeg/compressed이지만, 카메라 4대를
구분하기 위해 센서별 고유 topic을 MORAI ROS 설정에 넣는다.

LiDAR는 ROS 좌표계인 x 전방, y 좌측, z 위쪽을 기준으로 처리한다. raw UDP
Velodyne packet을 직접 처리할 때만 별도 축 변환을 적용한다.

## 예정 패키지

- morai_camera_bridge
- morai_camera_perception
- morai_lidar_bridge
- morai_lidar_perception
- morai_sensor_fusion

각 센서가 독립적으로 timestamp, frame_id, 주기, 진단 정보를 만족한 뒤
융합한다. Detection은 /Object_topic이나 /GetTrafficLightStatus 같은
ground truth 토픽을 대회 인식 입력으로 사용하지 않는다.

Detection은 조향·가속을 직접 결정하지 않고 인식 결과와 안전정지 요청만 발행한다.

