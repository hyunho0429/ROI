cmake_minimum_required(VERSION 3.0.2)
project(purepursuit_mgeo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  lidar_perception
  morai_msgs
  nav_msgs
  path_planning
  rospy
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/purepursuit_mgeo_node.py
  scripts/replay_mgeo_path_pose.py
  scripts/kinematic_vehicle_sim.py
  scripts/avoidance_frenet_debug_node.py
  scripts/avoidance_path_manager_node.py
  scripts/bypass_lane_guard_node.py
  scripts/highway_lane_strategy_node.py
  scripts/lane_info_runner.py
  scripts/lane_info_semantic_adapter.py
  scripts/roundabout_merge_gate_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Imported modules need real files; catkin's executable relay does not export
# the globals of the source module to importers.
foreach(helper frenet_path frenet_sampling_planner trajectory_safety)
  configure_file(scripts/${helper}.py
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${helper}.py COPYONLY)
  install(FILES scripts/${helper}.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
