cmake_minimum_required(VERSION 3.0.2)
project(camera_perception)

find_package(catkin REQUIRED COMPONENTS common nav_msgs rospy std_msgs)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS common nav_msgs rospy std_msgs)

catkin_install_python(PROGRAMS
  scripts/camera_feature_runner.py
  scripts/camera_object_detection_node.py
  scripts/highway_environment_gate_node.py
  scripts/intersection_environment_node.py
  scripts/pedestrian_crossing_fusion_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY lane models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
