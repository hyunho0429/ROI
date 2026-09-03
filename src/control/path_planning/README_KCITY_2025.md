# K-City 2025 MGeo path planning

## Data

The K-City 2025 MGeo JSON files are stored in:

```text
src/control/path_planning/mgeo/R_KR_PR_K-city_2025
```

Important files:

- `node_set.json`: road graph nodes
- `link_set.json`: directed road links and sampled link points
- `lane_node_set.json`, `lane_boundary_set.json`: lane boundary geometry
- `traffic_light_set.json`, `crosswalk_set.json`: signal and crosswalk metadata

## Interactive Dijkstra global path

Run the interactive RViz launch:

```bash
roslaunch path_planning kcity_2025_dijkstra.launch
```

The launch first shows a default route from `A1256W000437` to `A1256W000531`.

In RViz:

1. Set `Fixed Frame` to `map`.
2. Select `2D Nav Goal`.
3. Click the desired destination on the map.
4. The node converts the clicked destination to the nearest MGeo node, runs Dijkstra, and publishes `/global_path` and `/global_path_marker`.

The planner uses `/odom` as the start position when it is available. If `/odom` is not available, it falls back to `start_node`.
The clicked goal is converted to nearby MGeo nodes, and the first reachable node is used.

Set the fallback start node:

```bash
roslaunch path_planning kcity_2025_dijkstra.launch \
  start_node:=A1256W000437
```

You can also click with RViz `Publish Point`; it publishes `/clicked_point`, which is handled the same way.
If the selected point is near a disconnected or opposite-direction node, increase `goal_search_count`.

```bash
roslaunch path_planning kcity_2025_dijkstra.launch goal_search_count:=50
```

## Static Dijkstra global path

For a fixed start and goal node:

```bash
roslaunch path_planning kcity_2025_dijkstra.launch \
  interactive:=false \
  start_node:=A1256W000437 \
  goal_node:=A1256W000531
```

Or set coordinates directly. The nearest MGeo node is used:

```bash
rosrun path_planning global_path_kcity_dijkstra.py \
  _start_xy:="-300.0,560.0" \
  _goal_xy:="30.0,1000.0"
```

The node publishes:

- `/global_path` as `nav_msgs/Path`
- `/global_path_marker` as a green `visualization_msgs/Marker`
- `/node` and `/link` as `sensor_msgs/PointCloud` when `mgeo_json_pub.py` is running
- `/mgeo_nodes_marker` and `/mgeo_links_marker` as `visualization_msgs/Marker`

## RViz

Use `path_planning/rviz/kcity_2025_dijkstra.rviz`, or add these displays manually:

- `PointCloud` topic `/node`, fixed frame `map`
- `PointCloud` topic `/link`, fixed frame `map`
- `Path` topic `/global_path`, fixed frame `map`
- `Marker` topic `/global_path_marker`, fixed frame `map`

For a quick manual run:

```bash
roscore
rosrun path_planning mgeo_json_pub.py
rosrun path_planning global_path_kcity_dijkstra.py _interactive:=true _start_node:=A1256W000437
rviz -d $(rospack find path_planning)/rviz/kcity_2025_dijkstra.rviz
```
