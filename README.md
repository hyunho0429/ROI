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

Record the reference path with the competition-allowed GPS UDP sensor while
driving manually. The recorder saves a point every 0.5 m by default:

```bash
python3 src/path_planning/src/morai_gps_csv_recorder.py \
  --bind-ip 0.0.0.0 --port 3001 \
  --output src/path_planning/data/morai_global_path.csv
```

The CSV contains raw latitude/longitude/altitude, derived map-local ENU, and the
fixed CRS/EastOffset/NorthOffset/UpOffset used for conversion. It deliberately
does not use Ego Vehicle Status or store historical IMU samples. See
`src/path_planning/README_GPS_CSV_RECORDER.md`.

## MORAI UDP Pure Pursuit control

The `dev/stanley` branch now runs a standalone Pure Pursuit controller with a
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
python3 src/path_planning/src/morai_pure_pursuit_ins_udp.py \
  --path src/path_planning/data/2026_molit_comp_global_path.txt
```

See `src/path_planning/README_PURE_PURSUIT_UDP.md` for the MORAI 25.S4 protocol basis,
coordinate conversion, network settings, safety behavior, and tuning values.

Before running the full controller, verify that MORAI reflects the safe brake
command in Competition Vehicle Status:

```bash
sudo "$(which python3)" src/path_planning/src/morai_udp_control_check.py
```

For comparison, the speed-aided dead-reckoning alternative remains available:

```bash
# Competition-speed-aided dead reckoning
python3 src/path_planning/src/morai_pure_pursuit_dead_reckoning_udp.py \
  --path src/path_planning/data/2026_molit_comp_global_path.txt
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
