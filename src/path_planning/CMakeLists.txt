cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/global_path_kcity_dijkstra.py
  src/morai_global_csv_recorder.py
  src/morai_gps_csv_recorder.py
  src/morai_udp_control_check.py
  src/morai_competition_status_inspect.py
  src/morai_lidar_intensity_inspect.py
  src/lidar_monitor.py
  src/morai_lidar_pointcloud_udp.py
  src/morai_lidar_pointcloud_udp_ground.py
  src/morai_pure_pursuit_ins_udp.py
  src/morai_pure_pursuit_dead_reckoning_udp.py
  src/morai_stanley_udp.py
  src/morai_stanley_ins_udp.py
  src/morai_stanley_dead_reckoning_udp.py
  src/mgeo_json_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch data mgeo rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
