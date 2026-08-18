cmake_minimum_required(VERSION 3.0.2)
project(purepursuit_mgeo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  morai_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/purepursuit_mgeo_node.py
  scripts/replay_mgeo_path_pose.py
  scripts/kinematic_vehicle_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
