cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  morai_msgs
  nav_msgs
  rospy
  sensor_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs morai_msgs nav_msgs rospy sensor_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/global_path_kcity_dijkstra.py
  src/morai_global_csv_recorder.py
  src/mgeo_json_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
