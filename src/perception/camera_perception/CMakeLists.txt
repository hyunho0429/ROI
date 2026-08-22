cmake_minimum_required(VERSION 3.0.2)
project(camera_perception)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs)

catkin_install_python(PROGRAMS
  scripts/camera_feature_runner.py
  scripts/highway_environment_gate_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
