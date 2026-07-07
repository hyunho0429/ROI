# K-City 2025 Dijkstra Path Planning

This branch keeps only the K-City 2025 MGeo JSON data, Dijkstra global path generation, and RViz visualization.

## Build

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Run

```bash
roslaunch path_planning kcity_2025_dijkstra.launch use_odom_start:=false
```

In RViz, select `2D Nav Goal` and click a destination on the map. The planner chooses a reachable MGeo node near the clicked point, computes the shortest Dijkstra path from the start node, and publishes `/global_path`.

Useful topics:

```bash
rostopic echo /node
rostopic echo /link
rostopic echo /global_path
```

For a fixed start and goal node:

```bash
roslaunch path_planning kcity_2025_dijkstra.launch \
  interactive:=false \
  use_odom_start:=false \
  start_node:=A1256W000437 \
  goal_node:=A1256W000531
```
