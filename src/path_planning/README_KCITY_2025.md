# K-City 2025 MGeo path planning

## Data

The K-City 2025 MGeo JSON files are stored in:

```text
src/path_planning/mgeo/R_KR_PR_K-city_2025
```

Important files:

- `node_set.json`: road graph nodes
- `link_set.json`: directed road links and sampled link points
- `lane_node_set.json`, `lane_boundary_set.json`: lane boundary geometry
- `traffic_light_set.json`, `crosswalk_set.json`: signal and crosswalk metadata

## Dijkstra global path

Run the example launch:

```bash
roslaunch path_planning kcity_2025_dijkstra.launch rviz:=true
```

Set start and goal by node id:

```bash
roslaunch path_planning kcity_2025_dijkstra.launch \
  start_node:=A1256W000437 \
  goal_node:=A1256W000531 \
  rviz:=true
```

Or set coordinates. The nearest MGeo node is used:

```bash
rosrun path_planning global_path_kcity_dijkstra.py \
  _start_xy:="-300.0,560.0" \
  _goal_xy:="30.0,1000.0"
```

The node publishes:

- `/global_path` as `nav_msgs/Path`
- `/node` and `/link` as `sensor_msgs/PointCloud` when `mgeo_json_pub.py` is running

## RViz

Use `auto_driving/rviz/final.rviz`, or add these displays manually:

- `PointCloud` topic `/node`, fixed frame `map`
- `PointCloud` topic `/link`, fixed frame `map`
- `Path` topic `/global_path`, fixed frame `map`

For a quick manual run:

```bash
roscore
rosrun path_planning mgeo_json_pub.py
rosrun path_planning global_path_kcity_dijkstra.py _start_node:=A1256W000437 _goal_node:=A1256W000531
rviz -d $(rospack find auto_driving)/rviz/final.rviz
```
