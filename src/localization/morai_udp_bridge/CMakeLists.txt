cmake_minimum_required(VERSION 3.0.2)
project(morai_udp_bridge)

find_package(catkin REQUIRED COMPONENTS
  morai_msgs
  rospy
  sensor_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/morai_udp_bridge_node.py
  scripts/morai_udp_dump.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
