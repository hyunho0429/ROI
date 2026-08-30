cmake_minimum_required(VERSION 3.0.2)
project(lidar_perception)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  LidarObstacle.msg
  LidarObstacleArray.msg
  MergeGapObstacle.msg
  MergeGapObstacleArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs rospy sensor_msgs std_msgs visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/morai_lidar_udp_source.py
  scripts/lidar_euclidean_node.py
  scripts/lidar_bbox_node.py
  scripts/lidar_tracking_node.py
  scripts/lidar_merge_gap_node.py
  scripts/lidar_tracked_obstacle_state_node.py
  scripts/lidar_left_parallel_dynamic_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
