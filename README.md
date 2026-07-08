# K-City 2025 Dijkstra Path Planning

This branch keeps only the K-City 2025 MGeo JSON data, Dijkstra global path generation, and RViz visualization.

## Build

```bash
cd ~/catkin_ws
sb
catkin_make
source devel/setup.bash
```

## Run

```bash
roslaunch path_planning kcity_2025_dijkstra.launch use_odom_start:=false
```

The launch shows a default example route from `A1256W000437` to `A1256W000531`.
In RViz, select `2D Nav Goal` and click another destination on the map to update the route. The planner chooses a reachable MGeo node near the clicked point, computes the shortest Dijkstra path from the start node, and publishes `/global_path` and `/global_path_marker`.

Useful topics:

```bash
rostopic echo /node
rostopic echo /link
rostopic echo /global_path
rostopic echo /global_path_marker
```

For a fixed start and goal node:

```bash
roslaunch path_planning kcity_2025_dijkstra.launch \
  interactive:=false \
  use_odom_start:=false \
  start_node:=A1256W000437 \
  goal_node:=A1256W000531
```
