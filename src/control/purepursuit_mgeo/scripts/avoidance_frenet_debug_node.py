#!/usr/bin/env python3
"""Stage A/B/C obstacle + Frenet sampling visualizer.

This node intentionally does NOT publish /ctrl_cmd and does NOT change the
existing Pure Pursuit path.  It exists only to validate, in order:

A. tracked LiDAR obstacles really line up in the map frame,
B. map -> Frenet projection gives sensible s,d,
C. MGeo-adjacent-lane candidates sampled in Frenet convert back to sensible
   map-frame paths.

Only after these three checks pass should collision rejection and path switching
be added.
"""

from __future__ import annotations

import ast
import json
import math
from typing import List, Optional

import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from lidar_perception.msg import LidarObstacleArray
from purepursuit_mgeo.path import PathPoint, load_mgeo_path
from frenet_path import FrenetProjection, ReferencePath
from frenet_sampling_planner import (
    FrenetLaneChangeCandidate,
    LinkSpatialIndex,
    available_adjacent_links,
    generate_frenet_lane_change_candidates,
    load_mgeo_links,
)


def _parse_float_list(value, default):
    if value is None:
        return tuple(default)
    if isinstance(value, str):
        value = ast.literal_eval(value)
    return tuple(float(v) for v in value)


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def _point(x: float, y: float, z: float = 0.0) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


class AvoidanceFrenetDebugNode:
    def __init__(self) -> None:
        rospy.init_node("avoidance_frenet_debug", anonymous=False)

        path_file = rospy.get_param("~path_file")
        link_set_file = rospy.get_param("~link_set_file")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odometry")
        self.obstacle_topic = rospy.get_param(
            "~obstacle_topic", "/perception/lidar/tracked_obstacles_map"
        )
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.local_length_m = float(rospy.get_param("~local_length_m", 45.0))
        self.sample_spacing_m = float(rospy.get_param("~sample_spacing_m", 0.5))
        self.obstacle_display_height_m = float(
            rospy.get_param("~obstacle_display_height_m", 1.5)
        )
        self.velocity_arrow_scale_s = float(
            rospy.get_param("~velocity_arrow_scale_s", 1.0)
        )

        self.start_distances_m = _parse_float_list(
            rospy.get_param("~lane_change_start_distances_m", [3.0, 7.0, 11.0]),
            [3.0, 7.0, 11.0],
        )
        self.change_lengths_m = _parse_float_list(
            rospy.get_param("~lane_change_lengths_m", [15.0, 22.0]),
            [15.0, 22.0],
        )

        global_points: List[PathPoint] = load_mgeo_path(path_file)
        self.reference = ReferencePath(
            global_points,
            grid_cell_size_m=float(rospy.get_param("~reference_grid_cell_size_m", 10.0)),
            grid_point_stride=int(rospy.get_param("~reference_grid_point_stride", 1)),
        )
        self.links = load_mgeo_links(link_set_file)
        self.link_index = LinkSpatialIndex(
            self.links,
            cell_size_m=float(rospy.get_param("~link_index_cell_size_m", 10.0)),
            point_stride=int(rospy.get_param("~link_index_point_stride", 3)),
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_obstacles: Optional[LidarObstacleArray] = None
        self.last_current_link_idx: Optional[str] = None

        self.global_pub = rospy.Publisher("~global_path", RosPath, queue_size=1, latch=True)
        self.candidate_pub = rospy.Publisher("~candidate_paths", MarkerArray, queue_size=1)
        self.obstacle_pub = rospy.Publisher("~obstacles", MarkerArray, queue_size=1)
        self.link_pub = rospy.Publisher("~mgeo_links", MarkerArray, queue_size=1)
        self.debug_pub = rospy.Publisher("~frenet_debug", String, queue_size=1)

        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_callback, queue_size=5
        )
        self.obstacle_sub = rospy.Subscriber(
            self.obstacle_topic, LidarObstacleArray, self._obstacle_callback, queue_size=1
        )

        self._publish_global_path()
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self._timer_callback
        )

        rospy.logwarn(
            "Frenet avoidance DEBUG only: NO /ctrl_cmd, NO path switching. "
            "odom=%s obstacles=%s global_points=%d links=%d",
            self.odom_topic,
            self.obstacle_topic,
            len(global_points),
            len(self.links),
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _obstacle_callback(self, msg: LidarObstacleArray) -> None:
        self.latest_obstacles = msg

    def _publish_global_path(self) -> None:
        msg = RosPath()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.Time.now()
        for p in self.reference.points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = p.x
            ps.pose.position.y = p.y
            ps.pose.position.z = p.z
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.global_pub.publish(msg)

    @staticmethod
    def _candidate_color(side: str, variant: int) -> ColorRGBA:
        c = ColorRGBA()
        c.a = 0.95
        if side == "left":
            c.r, c.g, c.b = 0.10, 0.75, 1.00
        else:
            c.r, c.g, c.b = 1.00, 0.55, 0.10
        factor = max(0.55, 1.0 - 0.06 * variant)
        c.r *= factor
        c.g *= factor
        c.b *= factor
        return c

    def _candidate_markers(
        self, candidates: List[FrenetLaneChangeCandidate]
    ) -> MarkerArray:
        result = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        result.markers.append(clear)

        side_counts = {"left": 0, "right": 0}
        for marker_id, candidate in enumerate(candidates):
            variant = side_counts[candidate.side]
            side_counts[candidate.side] += 1

            line = Marker()
            line.header.frame_id = self.map_frame
            line.header.stamp = rospy.Time.now()
            line.ns = "frenet_candidates"
            line.id = marker_id * 2
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.18
            line.color = self._candidate_color(candidate.side, variant)
            line.points = [_point(p.x, p.y, p.z + 0.12) for p in candidate.path]
            result.markers.append(line)

            label = Marker()
            label.header = line.header
            label.ns = "frenet_candidate_labels"
            label.id = marker_id * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.orientation.w = 1.0
            anchor = candidate.path[min(len(candidate.path) - 1, max(0, len(candidate.path) // 2))]
            label.pose.position.x = anchor.x
            label.pose.position.y = anchor.y
            label.pose.position.z = anchor.z + 1.0
            label.scale.z = 0.45
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = "{} start={:.0f} L={:.0f} d={:+.2f}".format(
                candidate.side[0].upper(),
                candidate.start_distance_m,
                candidate.change_length_m,
                candidate.target_d_m,
            )
            result.markers.append(label)
        return result

    def _link_markers(self, current_link) -> MarkerArray:
        result = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        result.markers.append(clear)

        def add_link(link, marker_id, namespace, rgb, width):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = width
            marker.color.r, marker.color.g, marker.color.b = rgb
            marker.color.a = 0.95
            marker.points = [_point(p.x, p.y, p.z + 0.08) for p in link.points]
            result.markers.append(marker)

        add_link(current_link, 0, "current_link", (0.35, 1.0, 0.35), 0.28)
        for index, (side, link) in enumerate(available_adjacent_links(current_link, self.links), start=1):
            rgb = (0.15, 0.65, 1.0) if side == "left" else (1.0, 0.45, 0.15)
            add_link(link, index, "adjacent_links", rgb, 0.22)
        return result

    def _obstacle_markers(
        self,
        obstacles: Optional[LidarObstacleArray],
        ego_projection: FrenetProjection,
    ):
        result = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        result.markers.append(clear)
        summaries = []

        if obstacles is None:
            return result, summaries

        for index, obstacle in enumerate(obstacles.obstacles):
            projection = self.reference.project(
                float(obstacle.center_x_map), float(obstacle.center_y_map)
            )
            rel_s = projection.s - ego_projection.s
            speed = math.hypot(
                float(obstacle.velocity_x_map), float(obstacle.velocity_y_map)
            )

            box = Marker()
            box.header.frame_id = self.map_frame
            box.header.stamp = obstacles.header.stamp
            box.ns = "tracked_obstacle_boxes"
            box.id = index * 4
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(obstacle.center_x_map)
            box.pose.position.y = float(obstacle.center_y_map)
            box.pose.position.z = 0.5 * self.obstacle_display_height_m
            box.pose.orientation = _yaw_to_quaternion(float(obstacle.yaw))
            box.scale.x = max(0.10, float(obstacle.length))
            box.scale.y = max(0.10, float(obstacle.width))
            box.scale.z = self.obstacle_display_height_m
            box.color.r, box.color.g, box.color.b, box.color.a = 1.0, 0.15, 0.15, 0.35
            result.markers.append(box)

            velocity = Marker()
            velocity.header = box.header
            velocity.ns = "tracked_obstacle_velocity"
            velocity.id = index * 4 + 1
            velocity.type = Marker.ARROW
            velocity.action = Marker.ADD
            velocity.pose.orientation.w = 1.0
            start = _point(float(obstacle.center_x_map), float(obstacle.center_y_map), 1.0)
            end = _point(
                start.x + float(obstacle.velocity_x_map) * self.velocity_arrow_scale_s,
                start.y + float(obstacle.velocity_y_map) * self.velocity_arrow_scale_s,
                1.0,
            )
            velocity.points = [start, end]
            velocity.scale.x = 0.10
            velocity.scale.y = 0.20
            velocity.scale.z = 0.25
            velocity.color.r, velocity.color.g, velocity.color.b, velocity.color.a = 1.0, 1.0, 0.15, 1.0
            result.markers.append(velocity)

            projection_line = Marker()
            projection_line.header = box.header
            projection_line.ns = "obstacle_frenet_projection"
            projection_line.id = index * 4 + 2
            projection_line.type = Marker.LINE_STRIP
            projection_line.action = Marker.ADD
            projection_line.pose.orientation.w = 1.0
            projection_line.scale.x = 0.06
            projection_line.color.r, projection_line.color.g = 0.95, 0.95
            projection_line.color.b, projection_line.color.a = 0.95, 0.8
            projection_line.points = [
                _point(float(obstacle.center_x_map), float(obstacle.center_y_map), 0.15),
                _point(projection.reference_x, projection.reference_y, 0.15),
            ]
            result.markers.append(projection_line)

            label = Marker()
            label.header = box.header
            label.ns = "tracked_obstacle_labels"
            label.id = index * 4 + 3
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.orientation.w = 1.0
            label.pose.position.x = float(obstacle.center_x_map)
            label.pose.position.y = float(obstacle.center_y_map)
            label.pose.position.z = self.obstacle_display_height_m + 0.7
            label.scale.z = 0.42
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = "id={} ds={:+.1f} d={:+.2f} v={:.1f}".format(
                int(obstacle.id), rel_s, projection.d, speed
            )
            result.markers.append(label)

            summaries.append(
                {
                    "id": int(obstacle.id),
                    "delta_s_m": round(rel_s, 2),
                    "d_m": round(projection.d, 2),
                    "speed_mps": round(speed, 2),
                }
            )

        return result, summaries

    def _timer_callback(self, _event) -> None:
        if self.latest_odom is None:
            rospy.logwarn_throttle(3.0, "Waiting for %s", self.odom_topic)
            return

        pose = self.latest_odom.pose.pose
        ego_projection = self.reference.project(pose.position.x, pose.position.y)
        reference_point = self.reference.frenet_to_map(ego_projection.s, 0.0)
        current_link = self.link_index.nearest_link(reference_point.x, reference_point.y)

        if current_link is None:
            rospy.logwarn_throttle(2.0, "No MGeo link near current global reference point")
            return

        candidates = generate_frenet_lane_change_candidates(
            reference=self.reference,
            ego_s=ego_projection.s,
            current_link=current_link,
            links=self.links,
            start_distances_m=self.start_distances_m,
            change_lengths_m=self.change_lengths_m,
            local_length_m=self.local_length_m,
            sample_spacing_m=self.sample_spacing_m,
        )

        obstacle_markers, obstacle_summaries = self._obstacle_markers(
            self.latest_obstacles, ego_projection
        )
        self.obstacle_pub.publish(obstacle_markers)
        self.link_pub.publish(self._link_markers(current_link))
        self.candidate_pub.publish(self._candidate_markers(candidates))

        adjacent = available_adjacent_links(current_link, self.links)
        debug_payload = {
            "ego": {
                "s_m": round(ego_projection.s, 3),
                "d_m": round(ego_projection.d, 3),
            },
            "current_link": current_link.idx,
            "ego_lane": current_link.ego_lane,
            "adjacent_links": [
                {"side": side, "link": link.idx} for side, link in adjacent
            ],
            "candidate_count": len(candidates),
            "obstacle_count": len(obstacle_summaries),
            "obstacles": obstacle_summaries,
        }
        self.debug_pub.publish(
            String(data=json.dumps(debug_payload, ensure_ascii=False, separators=(",", ":")))
        )

        if self.last_current_link_idx != current_link.idx:
            rospy.logwarn(
                "Current MGeo link=%s lane=%s left=%s right=%s",
                current_link.idx,
                str(current_link.ego_lane),
                str(current_link.left_lane_change_dst_link_idx),
                str(current_link.right_lane_change_dst_link_idx),
            )
            self.last_current_link_idx = current_link.idx

        rospy.loginfo_throttle(
            1.0,
            "Frenet DEBUG ego(s=%.1f,d=%+.2f) obstacles=%d candidates=%d",
            ego_projection.s,
            ego_projection.d,
            len(obstacle_summaries),
            len(candidates),
        )


def main() -> None:
    try:
        AvoidanceFrenetDebugNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
