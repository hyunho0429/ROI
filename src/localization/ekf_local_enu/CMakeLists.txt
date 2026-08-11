cmake_minimum_required(VERSION 3.0.2)
project(ekf_local_enu)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ekf_local_enu_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
