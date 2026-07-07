cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/global_path_kcity_dijkstra.py
  src/mgeo_json_dijkstra.py
  src/mgeo_json_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
