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

## MORAI keyboard path CSV recording

To save Competition Vehicle Status ENU coordinates at uniform 0.5 m 3-D intervals:

```bash
python3 src/path_planning/src/morai_global_csv_recorder.py --bind-ip 0.0.0.0 --port 3315
```

The recorder is standalone and does not require ROS. The default output is `src/path_planning/data/morai_global_path.csv`. See `src/path_planning/README_GLOBAL_CSV_RECORDER.md` for MORAI network settings and parameters.

## MORAI UDP Stanley control

The `dev/stanley` branch adds a standalone Stanley controller with noisy
GPS/IMU sensor fusion and the documented MORAI UDP control packet. Install
`src/path_planning/requirements.txt`, then run:

```bash
python3 src/path_planning/src/morai_stanley_udp.py \
  --path src/path_planning/data/morai_global_path.csv \
  --gps-port 9100 --imu-port 9101 \
  --competition-status-port 3315 --collision-port 5678 \
  --control-ip 127.0.0.1 --control-port 9090
```

See `src/path_planning/README_STANLEY_UDP.md` for the MORAI 26.R1 public protocol basis,
coordinate conversion, network settings, safety behavior, and tuning values.

For a fixed start and goal node:

```bash
roslaunch path_planning kcity_2025_dijkstra.launch \
  interactive:=false \
  use_odom_start:=false \
  start_node:=A1256W000437 \
  goal_node:=A1256W000531
```
