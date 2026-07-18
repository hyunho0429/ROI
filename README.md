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

Path recording may use the documented Ego Vehicle Status interface even when
autonomous driving is restricted to Competition Vehicle Status. Configure Ego
Vehicle Status UDP Destination IP/Port and save its map-local ENU position once
per second:

```bash
python3 src/path_planning/src/morai_global_csv_recorder.py \
  --bind-ip 0.0.0.0 --port 9102 \
  --output src/path_planning/data/morai_global_path.csv \
  --sample-period 1.0
```

The recorder is standalone and does not require ROS or coordinate conversion.
Ego Vehicle Status position is already map-local ENU, which is the frame used by
the Stanley path tracker. `morai_gps_csv_recorder.py` remains available as a
GPS-based fallback.

## MORAI UDP Stanley control

The `dev/stanley` branch adds a standalone strict Stanley controller with a
15-state GPS/IMU/Competition-speed-aided EKF-INS. It ports AutoVehicle's local
ENU CSV conversion and waypoint preprocessing without its ROS dependencies.
Install `src/path_planning/requirements.txt`, then run the recommended INS
runner:

The competition UDP values are defined in
`src/path_planning/src/path_planning/morai_competition_config.py`: GPS `3001`,
IMU `4001`, Competition Status `909`, CollisionData `907`, control destination
`192.168.0.170:9090`, and target speed `10 km/h`. They do not need to be
repeated on the command line.

```bash
python3 src/path_planning/src/morai_stanley_ins_udp.py \
  --path src/path_planning/data/morai_global_path.csv
```

See `src/path_planning/README_STANLEY_UDP.md` for the MORAI 26.R1 public protocol basis,
coordinate conversion, network settings, safety behavior, and tuning values.

For comparison, the speed-aided dead-reckoning alternative remains available:

```bash
# Competition-speed-aided dead reckoning
python3 src/path_planning/src/morai_stanley_dead_reckoning_udp.py \
  --path src/path_planning/data/morai_global_path.csv
```

Both continue through a configurable GPS outage while IMU and Competition
Vehicle Status remain fresh. See `src/path_planning/README_TUNNEL_LOCALIZATION.md`.

For a fixed start and goal node:

```bash
roslaunch path_planning kcity_2025_dijkstra.launch \
  interactive:=false \
  use_odom_start:=false \
  start_node:=A1256W000437 \
  goal_node:=A1256W000531
```
