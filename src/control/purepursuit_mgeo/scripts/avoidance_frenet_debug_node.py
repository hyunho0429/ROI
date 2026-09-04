#!/usr/bin/env python3
"""Stage C.5 Frenet avoidance visual/debug node.

Adds to the previous A/B/C stage:
* ego vehicle footprint visualization (4.635 x 1.892 x 2.434 m by default),
* obstacle-vs-global-corridor threat gating using obstacle OBB projection,
* one-lane/free-space bypass candidates only when a blocking obstacle exists
  and no adjacent MGeo lane is available,
* candidate swept-width / sparse vehicle-footprint visualization.

Stage D/E/F6 planner responsibilities included here:
* static obstacle OBB candidate rejection,
* curvature/steering/lateral-acceleration feasibility checks,
* deterministic best-candidate selection,
* small control-facing status topics for the separate path manager.

Still intentionally NOT implemented here:
* road-polygon / lane-boundary legality for bypass candidates,
* dynamic obstacle prediction,
* active-path commitment/state handling,
* /ctrl_cmd publishing.
"""

from __future__ import annotations

import ast
import copy
import json
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Union

import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from trajectory_safety import ObstacleBox, CandidateEvaluation, evaluate_candidate

from lidar_perception.msg import LidarObstacleArray
from purepursuit_mgeo.path import PathPoint, load_mgeo_path
from frenet_path import FrenetProjection, ReferencePath
from frenet_sampling_planner import (
    FrenetBypassCandidate,
    FrenetLaneChangeCandidate,
    LinkSpatialIndex,
    available_adjacent_links,
    generate_frenet_bypass_candidates,
    generate_frenet_lane_change_candidates,
    load_mgeo_links,
)

Candidate = Union[FrenetLaneChangeCandidate, FrenetBypassCandidate]


@dataclass
class ObstacleFrenetInfo:
    obstacle_id: int
    projection: FrenetProjection
    delta_s_m: float
    near_edge_distance_m: float
    longitudinal_half_m: float
    lateral_half_m: float
    speed_mps: float
    threatening: bool


def _parse_float_list(value, default):
    if value is None:
        return tuple(default)
    if isinstance(value, str):
        value = ast.literal_eval(value)
    return tuple(float(v) for v in value)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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


def _path_tangent(path: Sequence[PathPoint], index: int):
    if len(path) < 2:
        return 1.0, 0.0
    if index <= 0:
        a, b = path[0], path[1]
    elif index >= len(path) - 1:
        a, b = path[-2], path[-1]
    else:
        a, b = path[index - 1], path[index + 1]
    dx = b.x - a.x
    dy = b.y - a.y
    norm = math.hypot(dx, dy)
    if norm < 1.0e-9:
        return 1.0, 0.0
    return dx / norm, dy / norm


def _rectangle_corners(
    base_x: float,
    base_y: float,
    yaw: float,
    length_m: float,
    width_m: float,
    center_from_base_m: float,
):
    tx, ty = math.cos(yaw), math.sin(yaw)
    nx, ny = -ty, tx
    cx = base_x + center_from_base_m * tx
    cy = base_y + center_from_base_m * ty
    half_l = 0.5 * length_m
    half_w = 0.5 * width_m
    return [
        (cx + sx * half_l * tx + sy * half_w * nx,
         cy + sx * half_l * ty + sy * half_w * ny)
        for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))
    ]


class AvoidanceFrenetDebugNode:
    def __init__(self) -> None:
        rospy.init_node("avoidance_frenet_debug", anonymous=False)

        path_file = rospy.get_param("~path_file")
        link_set_file = rospy.get_param("~link_set_file")
        self.map_frame = rospy.get_param("~map_frame", "map")
        # Planner math remains in map.  These parameters add a second,
        # visualization-only output expressed in the LiDAR frame so the
        # existing perception RViz can keep Fixed Frame=morai_lidar.
        self.publish_lidar_visualization = bool(
            rospy.get_param("~publish_lidar_visualization", True)
        )
        self.lidar_viz_frame = rospy.get_param("~lidar_viz_frame", "morai_lidar")
        self.lidar_viz_prefix = str(
            rospy.get_param("~lidar_viz_prefix", "/avoidance_frenet_debug_lidar")
        ).rstrip("/")
        self.lidar_x_m = float(rospy.get_param("~lidar_x_m", 0.0))
        self.lidar_y_m = float(rospy.get_param("~lidar_y_m", 0.0))
        self.lidar_z_m = float(rospy.get_param("~lidar_z_m", 0.0))
        self.lidar_yaw_rad = math.radians(float(rospy.get_param("~lidar_yaw_deg", 0.0)))
        self.lidar_global_back_m = float(rospy.get_param("~lidar_global_back_m", 10.0))
        self.lidar_global_forward_m = float(
            rospy.get_param("~lidar_global_forward_m", 55.0)
        )
        self.lidar_global_spacing_m = float(
            rospy.get_param("~lidar_global_spacing_m", 0.75)
        )

        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odometry")
        self.obstacle_topic = rospy.get_param(
            "~obstacle_topic", "/perception/lidar/tracked_obstacles_map"
        )
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.local_length_m = float(rospy.get_param("~local_length_m", 55.0))
        self.sample_spacing_m = float(rospy.get_param("~sample_spacing_m", 0.5))
        self.obstacle_display_height_m = float(
            rospy.get_param("~obstacle_display_height_m", 1.5)
        )
        self.velocity_arrow_scale_s = float(
            rospy.get_param("~velocity_arrow_scale_s", 1.0)
        )

        # Confirmed vehicle dimensions.
        self.vehicle_length_m = float(rospy.get_param("~vehicle_length_m", 4.635))
        self.vehicle_width_m = float(rospy.get_param("~vehicle_width_m", 1.892))
        self.vehicle_height_m = float(rospy.get_param("~vehicle_height_m", 2.434))
        # Existing Pure Pursuit treats base_link as rear-axle center.  1.5 m is
        # a provisional center offset based on the 3.0 m wheelbase; verify in RViz.
        self.vehicle_center_from_base_m = float(
            rospy.get_param("~vehicle_center_from_base_m", 1.50)
        )
        self.candidate_corridor_margin_m = float(
            rospy.get_param("~candidate_corridor_margin_m", 0.45)
        )

        self.trigger_distance_m = float(rospy.get_param("~trigger_distance_m", 35.0))
        self.trigger_lateral_margin_m = float(
            rospy.get_param("~trigger_lateral_margin_m", 0.35)
        )

        self.start_distances_m = _parse_float_list(
            rospy.get_param("~lane_change_start_distances_m", [3.0, 7.0, 11.0]),
            [3.0, 7.0, 11.0],
        )
        self.change_lengths_m = _parse_float_list(
            rospy.get_param("~lane_change_lengths_m", [15.0, 22.0]),
            [15.0, 22.0],
        )

        self.bypass_lateral_margin_m = float(
            rospy.get_param("~bypass_lateral_margin_m", 0.85)
        )
        self.bypass_longitudinal_margin_m = float(
            rospy.get_param("~bypass_longitudinal_margin_m", 3.0)
        )
        self.bypass_departure_length_m = float(
            rospy.get_param("~bypass_departure_length_m", 16.0)
        )
        self.bypass_return_length_m = float(
            rospy.get_param("~bypass_return_length_m", 14.0)
        )
        self.bypass_extra_clearances_m = _parse_float_list(
            rospy.get_param("~bypass_extra_clearances_m", [0.0, 0.4, 0.8]),
            [0.0, 0.4, 0.8],
        )
        self.bypass_max_abs_d_m = float(rospy.get_param("~bypass_max_abs_d_m", 3.8))
        self.bypass_min_transition_length_m = float(
            rospy.get_param("~bypass_min_transition_length_m", 6.0)
        )
        self.allow_bypass_with_adjacent_lane = bool(
            rospy.get_param("~allow_bypass_with_adjacent_lane", False)
        )
        self.always_show_lane_candidates = bool(
            rospy.get_param("~always_show_lane_candidates", False)
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
        self.latest_odom_received_at: Optional[rospy.Time] = None
        self.latest_obstacles_received_at: Optional[rospy.Time] = None
        self.last_current_link_idx: Optional[str] = None

        # Last debug-only safety layer before active-path control integration.
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.50))
        self.obstacle_timeout_s = float(rospy.get_param("~obstacle_timeout_s", 0.60))
        self.collision_longitudinal_margin_m = float(rospy.get_param("~collision_longitudinal_margin_m", 0.40))
        self.collision_lateral_margin_m = float(rospy.get_param("~collision_lateral_margin_m", 0.45))
        self.wheelbase_m = float(rospy.get_param("~wheelbase_m", 3.0))
        self.max_steering_rad = float(rospy.get_param("~max_steering_rad", 0.6981317008))
        self.evaluation_speed_mps = float(rospy.get_param("~evaluation_speed_mps", 2.0))
        self.max_lateral_accel_mps2 = float(rospy.get_param("~max_lateral_accel_mps2", 2.5))
        self.collision_sample_stride = int(rospy.get_param("~collision_sample_stride", 1))
        self.escape_prefix_m = float(rospy.get_param("~escape_prefix_m", 3.0))
        self.plan_seq = 0

        # Control/diagnostic outputs stay in the planner's map frame.
        # RViz geometry is published only on the morai_lidar visualization namespace
        # below, avoiding duplicate map-frame visualization topics.
        self.debug_pub = rospy.Publisher("~frenet_debug", String, queue_size=1)
        self.selected_path_pub = rospy.Publisher("~selected_path", RosPath, queue_size=1)

        # Small control-facing status topics.  They deliberately use standard
        # messages so the path manager can remain independent of custom msgs.
        self.planner_ready_pub = rospy.Publisher("~planner_ready", Bool, queue_size=1)
        self.avoidance_required_pub = rospy.Publisher("~avoidance_required", Bool, queue_size=1)
        self.safe_path_available_pub = rospy.Publisher("~safe_path_available", Bool, queue_size=1)
        self.selected_kind_pub = rospy.Publisher("~selected_kind", String, queue_size=1)
        self.selected_side_pub = rospy.Publisher("~selected_side", String, queue_size=1)
        # F6 atomic control-facing status. selected_path.header.seq carries the
        # same sequence number so the Path Manager never combines a new status
        # with an old path (or vice versa). Legacy scalar topics stay published
        # for RViz/diagnostics.
        self.plan_status_pub = rospy.Publisher("~plan_status", String, queue_size=1)

        # Duplicate visualization topics in morai_lidar coordinates.  These are
        # ONLY for RViz; no planner/control computation uses this frame.
        if self.publish_lidar_visualization:
            self.lidar_global_pub = rospy.Publisher(
                self.lidar_viz_prefix + "/global_path", RosPath, queue_size=1
            )
            self.lidar_candidate_pub = rospy.Publisher(
                self.lidar_viz_prefix + "/candidate_paths", MarkerArray, queue_size=1
            )
            self.lidar_corridor_pub = rospy.Publisher(
                self.lidar_viz_prefix + "/candidate_corridors", MarkerArray, queue_size=1
            )
            self.lidar_obstacle_pub = rospy.Publisher(
                self.lidar_viz_prefix + "/obstacles", MarkerArray, queue_size=1
            )
            self.lidar_link_pub = rospy.Publisher(
                self.lidar_viz_prefix + "/mgeo_links", MarkerArray, queue_size=1
            )
            self.lidar_ego_pub = rospy.Publisher(
                self.lidar_viz_prefix + "/ego_footprint", MarkerArray, queue_size=1
            )
            self.lidar_selected_path_pub = rospy.Publisher(
                self.lidar_viz_prefix + "/selected_path", RosPath, queue_size=1
            )
            self.lidar_evaluation_pub = rospy.Publisher(
                self.lidar_viz_prefix + "/candidate_evaluations", MarkerArray, queue_size=1
            )

        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_callback, queue_size=5
        )
        self.obstacle_sub = rospy.Subscriber(
            self.obstacle_topic, LidarObstacleArray, self._obstacle_callback, queue_size=1
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self._timer_callback
        )

        rospy.loginfo(
            "Frenet avoidance F6.3: F6 control + morai_lidar-only RViz geometry; NO /ctrl_cmd. "
            "vehicle=%.3fx%.3fx%.3f center_from_base=%.2f",
            self.vehicle_length_m,
            self.vehicle_width_m,
            self.vehicle_height_m,
            self.vehicle_center_from_base_m,
        )
        if self.publish_lidar_visualization:
            rospy.loginfo(
                "LiDAR-frame debug visualization enabled: frame=%s prefix=%s "
                "extrinsic=(x=%.2f,y=%.2f,z=%.2f,yaw=%.1fdeg)",
                self.lidar_viz_frame,
                self.lidar_viz_prefix,
                self.lidar_x_m,
                self.lidar_y_m,
                self.lidar_z_m,
                math.degrees(self.lidar_yaw_rad),
            )

    def _next_plan_seq(self) -> int:
        self.plan_seq = (int(self.plan_seq) + 1) & 0xFFFFFFFF
        if self.plan_seq == 0:
            self.plan_seq = 1
        return self.plan_seq

    def _publish_control_bundle(
        self,
        path_msg: RosPath,
        planner_ready: bool,
        avoidance_required: bool,
        safe_path_available: bool,
        selected_kind: str = "",
        selected_side: str = "",
    ) -> None:
        # Publish path first, then one atomic JSON status carrying the exact same
        # sequence.  The path manager accepts a selected path only when both seqs
        # match.  This removes the F5 race between five independent scalar topics.
        seq = self._next_plan_seq()
        path_msg.header.seq = seq
        path_msg.header.stamp = rospy.Time.now()
        for ps in path_msg.poses:
            ps.header.seq = seq
            ps.header.stamp = path_msg.header.stamp
        self.selected_path_pub.publish(path_msg)

        payload = {
            "seq": seq,
            "planner_ready": bool(planner_ready),
            "avoidance_required": bool(avoidance_required),
            "safe_path_available": bool(safe_path_available),
            "selected_kind": str(selected_kind or ""),
            "selected_side": str(selected_side or ""),
        }
        self.plan_status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

        # Legacy topics remain for diagnostics / backwards compatibility only.
        self.planner_ready_pub.publish(Bool(data=bool(planner_ready)))
        self.avoidance_required_pub.publish(Bool(data=bool(avoidance_required)))
        self.safe_path_available_pub.publish(Bool(data=bool(safe_path_available)))
        self.selected_kind_pub.publish(String(data=str(selected_kind or "")))
        self.selected_side_pub.publish(String(data=str(selected_side or "")))

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_odom_received_at = rospy.Time.now()

    def _obstacle_callback(self, msg: LidarObstacleArray) -> None:
        self.latest_obstacles = msg
        self.latest_obstacles_received_at = rospy.Time.now()

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

    def _lidar_pose_in_map(self, odom: Odometry):
        """Return LiDAR origin/yaw in map using planar base->LiDAR extrinsics."""
        pose = odom.pose.pose
        ego_yaw = _quaternion_to_yaw(pose.orientation)
        c = math.cos(ego_yaw)
        sn = math.sin(ego_yaw)
        lidar_x_map = pose.position.x + c * self.lidar_x_m - sn * self.lidar_y_m
        lidar_y_map = pose.position.y + sn * self.lidar_x_m + c * self.lidar_y_m
        lidar_z_map = pose.position.z + self.lidar_z_m
        lidar_yaw_map = _normalize_angle(ego_yaw + self.lidar_yaw_rad)
        return lidar_x_map, lidar_y_map, lidar_z_map, lidar_yaw_map

    def _map_xyz_to_lidar(self, x: float, y: float, z: float, odom: Odometry):
        lx, ly, lz, lyaw = self._lidar_pose_in_map(odom)
        dx = float(x) - lx
        dy = float(y) - ly
        c = math.cos(lyaw)
        sn = math.sin(lyaw)
        # inverse planar rotation: map -> lidar
        return (
            c * dx + sn * dy,
            -sn * dx + c * dy,
            float(z) - lz,
        )

    def _map_yaw_to_lidar(self, yaw_map: float, odom: Odometry) -> float:
        _lx, _ly, _lz, lidar_yaw_map = self._lidar_pose_in_map(odom)
        return _normalize_angle(float(yaw_map) - lidar_yaw_map)

    def _marker_array_to_lidar(
        self, marker_array: MarkerArray, odom: Odometry
    ) -> MarkerArray:
        """Transform the debug markers from map coordinates to morai_lidar.

        Point-based markers keep identity pose and get every point transformed.
        Pose-based CUBE markers also have their yaw expressed relative to LiDAR.
        """
        result = copy.deepcopy(marker_array)
        for marker in result.markers:
            marker.header.frame_id = self.lidar_viz_frame
            if marker.action == Marker.DELETEALL:
                continue

            if marker.points:
                transformed = []
                for p in marker.points:
                    x, y, z = self._map_xyz_to_lidar(p.x, p.y, p.z, odom)
                    transformed.append(_point(x, y, z))
                marker.points = transformed
            else:
                x, y, z = self._map_xyz_to_lidar(
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z,
                    odom,
                )
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z

                if marker.type == Marker.CUBE:
                    yaw_map = _quaternion_to_yaw(marker.pose.orientation)
                    marker.pose.orientation = _yaw_to_quaternion(
                        self._map_yaw_to_lidar(yaw_map, odom)
                    )
        return result

    def _lidar_local_global_path(
        self, ego_s: float, odom: Odometry
    ) -> RosPath:
        """Publish only the nearby reference path in the moving LiDAR frame."""
        msg = RosPath()
        msg.header.frame_id = self.lidar_viz_frame
        msg.header.stamp = rospy.Time.now()

        start_s = max(0.0, float(ego_s) - self.lidar_global_back_m)
        end_s = min(
            self.reference.total_length_m,
            float(ego_s) + self.lidar_global_forward_m,
        )
        spacing = max(0.20, self.lidar_global_spacing_m)
        s_value = start_s
        while s_value <= end_s + 1.0e-6:
            p_map, tangent, _index, _ratio = self.reference.point_at_s(s_value)
            x, y, z = self._map_xyz_to_lidar(p_map.x, p_map.y, p_map.z, odom)
            yaw_map = math.atan2(tangent[1], tangent[0])
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation = _yaw_to_quaternion(
                self._map_yaw_to_lidar(yaw_map, odom)
            )
            msg.poses.append(ps)
            s_value += spacing

        if not msg.poses or end_s - (s_value - spacing) > 1.0e-3:
            p_map, tangent, _index, _ratio = self.reference.point_at_s(end_s)
            x, y, z = self._map_xyz_to_lidar(p_map.x, p_map.y, p_map.z, odom)
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation = _yaw_to_quaternion(
                self._map_yaw_to_lidar(math.atan2(tangent[1], tangent[0]), odom)
            )
            msg.poses.append(ps)
        return msg

    @staticmethod
    def _candidate_color(candidate: Candidate, variant: int) -> ColorRGBA:
        c = ColorRGBA()
        c.a = 0.95
        if candidate.kind == "bypass":
            if candidate.side == "left":
                c.r, c.g, c.b = 0.75, 0.20, 1.00
            else:
                c.r, c.g, c.b = 0.95, 0.20, 0.70
        elif candidate.side == "left":
            c.r, c.g, c.b = 0.10, 0.75, 1.00
        else:
            c.r, c.g, c.b = 1.00, 0.55, 0.10
        factor = max(0.55, 1.0 - 0.06 * variant)
        c.r *= factor
        c.g *= factor
        c.b *= factor
        return c

    def _candidate_markers(self, candidates: List[Candidate]) -> MarkerArray:
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
            line.scale.x = 0.20 if candidate.kind == "bypass" else 0.18
            line.color = self._candidate_color(candidate, variant)
            line.points = [_point(p.x, p.y, p.z + 0.12) for p in candidate.path]
            result.markers.append(line)

            label = Marker()
            label.header = line.header
            label.ns = "frenet_candidate_labels"
            label.id = marker_id * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.orientation.w = 1.0
            anchor = candidate.path[min(len(candidate.path) - 1, len(candidate.path) // 2)]
            label.pose.position.x = anchor.x
            label.pose.position.y = anchor.y
            label.pose.position.z = anchor.z + 1.0
            label.scale.z = 0.42
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            if candidate.kind == "bypass":
                label.text = "BP-{} obs={} d={:+.2f}".format(
                    candidate.side[0].upper(), candidate.obstacle_id, candidate.target_d_m
                )
            else:
                label.text = "LC-{} start={:.0f} L={:.0f} d={:+.2f}".format(
                    candidate.side[0].upper(),
                    candidate.start_distance_m,
                    candidate.change_length_m,
                    candidate.target_d_m,
                )
            result.markers.append(label)
        return result

    def _candidate_corridor_markers(self, candidates: List[Candidate]) -> MarkerArray:
        result = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        result.markers.append(clear)

        half_width = 0.5 * self.vehicle_width_m + self.candidate_corridor_margin_m
        marker_id = 0
        for candidate_index, candidate in enumerate(candidates):
            if len(candidate.path) < 2:
                continue
            color = self._candidate_color(candidate, candidate_index % 6)
            left_points = []
            right_points = []
            for i, p in enumerate(candidate.path):
                tx, ty = _path_tangent(candidate.path, i)
                nx, ny = -ty, tx
                left_points.append(_point(p.x + nx * half_width, p.y + ny * half_width, p.z + 0.06))
                right_points.append(_point(p.x - nx * half_width, p.y - ny * half_width, p.z + 0.06))

            for namespace, points in (("corridor_left", left_points), ("corridor_right", right_points)):
                line = Marker()
                line.header.frame_id = self.map_frame
                line.header.stamp = rospy.Time.now()
                line.ns = namespace
                line.id = marker_id
                marker_id += 1
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.pose.orientation.w = 1.0
                line.scale.x = 0.055
                line.color = color
                line.color.a = 0.48
                line.points = points
                result.markers.append(line)

            # Sparse 4.635 x 1.892 m vehicle footprints along the candidate.
            sample_indices = sorted(set((0, len(candidate.path) // 2, len(candidate.path) - 1)))
            for footprint_index, path_index in enumerate(sample_indices):
                p = candidate.path[path_index]
                tx, ty = _path_tangent(candidate.path, path_index)
                yaw = math.atan2(ty, tx)
                corners = _rectangle_corners(
                    p.x,
                    p.y,
                    yaw,
                    self.vehicle_length_m,
                    self.vehicle_width_m,
                    self.vehicle_center_from_base_m,
                )
                outline = Marker()
                outline.header.frame_id = self.map_frame
                outline.header.stamp = rospy.Time.now()
                outline.ns = "candidate_vehicle_footprints"
                outline.id = marker_id
                marker_id += 1
                outline.type = Marker.LINE_STRIP
                outline.action = Marker.ADD
                outline.pose.orientation.w = 1.0
                outline.scale.x = 0.045
                outline.color = color
                outline.color.a = 0.30
                outline.points = [_point(x, y, p.z + 0.10) for x, y in corners]
                outline.points.append(outline.points[0])
                result.markers.append(outline)

        return result

    def _ego_markers(self, odom: Odometry) -> MarkerArray:
        result = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        result.markers.append(clear)

        pose = odom.pose.pose
        yaw = _quaternion_to_yaw(pose.orientation)
        tx, ty = math.cos(yaw), math.sin(yaw)
        center_x = pose.position.x + self.vehicle_center_from_base_m * tx
        center_y = pose.position.y + self.vehicle_center_from_base_m * ty

        body = Marker()
        body.header.frame_id = self.map_frame
        body.header.stamp = odom.header.stamp if odom.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        body.ns = "ego_vehicle_body"
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x = center_x
        body.pose.position.y = center_y
        body.pose.position.z = pose.position.z + 0.5 * self.vehicle_height_m
        body.pose.orientation = _yaw_to_quaternion(yaw)
        body.scale.x = self.vehicle_length_m
        body.scale.y = self.vehicle_width_m
        body.scale.z = self.vehicle_height_m
        body.color.r, body.color.g, body.color.b, body.color.a = 0.15, 1.0, 0.35, 0.18
        result.markers.append(body)

        base = Marker()
        base.header = body.header
        base.ns = "ego_base_link"
        base.id = 1
        base.type = Marker.SPHERE
        base.action = Marker.ADD
        base.pose.position.x = pose.position.x
        base.pose.position.y = pose.position.y
        base.pose.position.z = pose.position.z + 0.25
        base.pose.orientation.w = 1.0
        base.scale.x = base.scale.y = base.scale.z = 0.35
        base.color.r, base.color.g, base.color.b, base.color.a = 1.0, 1.0, 1.0, 1.0
        result.markers.append(base)

        label = Marker()
        label.header = body.header
        label.ns = "ego_vehicle_label"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.orientation.w = 1.0
        label.pose.position.x = center_x
        label.pose.position.y = center_y
        label.pose.position.z = pose.position.z + self.vehicle_height_m + 0.7
        label.scale.z = 0.42
        label.color.r = label.color.g = label.color.b = 1.0
        label.color.a = 1.0
        label.text = "EGO {:.3f} x {:.3f}  base->center={:.2f}".format(
            self.vehicle_length_m, self.vehicle_width_m, self.vehicle_center_from_base_m
        )
        result.markers.append(label)
        return result

    def _candidate_to_ros_path(self, candidate: Optional[Candidate]) -> RosPath:
        msg = RosPath()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.Time.now()
        if candidate is None:
            return msg
        for i, p in enumerate(candidate.path):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = p.x
            ps.pose.position.y = p.y
            ps.pose.position.z = p.z + 0.15
            tx, ty = _path_tangent(candidate.path, i)
            ps.pose.orientation = _yaw_to_quaternion(math.atan2(ty, tx))
            msg.poses.append(ps)
        return msg

    def _ros_path_to_lidar(self, path_msg: RosPath, odom: Odometry) -> RosPath:
        out = RosPath()
        out.header.frame_id = self.lidar_viz_frame
        out.header.stamp = path_msg.header.stamp
        for ps_in in path_msg.poses:
            ps = PoseStamped()
            ps.header = out.header
            x, y, z = self._map_xyz_to_lidar(
                ps_in.pose.position.x, ps_in.pose.position.y, ps_in.pose.position.z, odom
            )
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
            yaw_map = _quaternion_to_yaw(ps_in.pose.orientation)
            ps.pose.orientation = _yaw_to_quaternion(self._map_yaw_to_lidar(yaw_map, odom))
            out.poses.append(ps)
        return out

    def _evaluation_markers(
        self,
        candidates: List[Candidate],
        evaluations: List[CandidateEvaluation],
        selected_index: Optional[int],
    ) -> MarkerArray:
        result = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        result.markers.append(clear)
        for ev in evaluations:
            if ev.candidate_index < 0 or ev.candidate_index >= len(candidates):
                continue
            candidate = candidates[ev.candidate_index]
            if not candidate.path:
                continue
            line = Marker()
            line.header.frame_id = self.map_frame
            line.header.stamp = rospy.Time.now()
            line.ns = "candidate_safety"
            line.id = ev.candidate_index * 2
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.32 if ev.candidate_index == selected_index else 0.12
            if ev.candidate_index == selected_index:
                line.color.r, line.color.g, line.color.b, line.color.a = 0.15, 1.0, 0.20, 1.0
            elif ev.valid:
                line.color.r, line.color.g, line.color.b, line.color.a = 0.20, 0.95, 0.35, 0.75
            elif ev.reason == "collision":
                line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.05, 0.05, 0.90
            else:
                line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.75, 0.05, 0.90
            line.points = [_point(p.x, p.y, p.z + 0.22) for p in candidate.path]
            result.markers.append(line)

            label = Marker()
            label.header = line.header
            label.ns = "candidate_safety_labels"
            label.id = ev.candidate_index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.orientation.w = 1.0
            anchor = candidate.path[min(len(candidate.path) - 1, len(candidate.path) // 2)]
            label.pose.position.x = anchor.x
            label.pose.position.y = anchor.y
            label.pose.position.z = anchor.z + 1.5
            label.scale.z = 0.34
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            prefix = "SELECT" if ev.candidate_index == selected_index else ("SAFE" if ev.valid else "REJECT")
            detail = ev.reason
            if ev.collision_obstacle_id is not None:
                detail += " obs={}".format(ev.collision_obstacle_id)
            label.text = "{} #{} {} k={:.3f} steer={:.2f} aY={:.2f}".format(
                prefix, ev.candidate_index, detail,
                ev.max_curvature_1pm, ev.max_steering_rad, ev.max_lateral_accel_mps2,
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

    def _obstacle_infos(
        self,
        obstacles: Optional[LidarObstacleArray],
        ego_projection: FrenetProjection,
    ) -> List[ObstacleFrenetInfo]:
        infos: List[ObstacleFrenetInfo] = []
        if obstacles is None:
            return infos

        ego_front_from_base_m = self.vehicle_center_from_base_m + 0.5 * self.vehicle_length_m
        ego_half_width_m = 0.5 * self.vehicle_width_m

        for obstacle in obstacles.obstacles:
            projection = self.reference.project(
                float(obstacle.center_x_map), float(obstacle.center_y_map)
            )
            rel_s = projection.s - ego_projection.s
            reference_yaw = math.atan2(projection.tangent_y, projection.tangent_x)
            relative_yaw = _normalize_angle(float(obstacle.yaw) - reference_yaw)
            abs_cos = abs(math.cos(relative_yaw))
            abs_sin = abs(math.sin(relative_yaw))
            length = max(0.0, float(obstacle.length))
            width = max(0.0, float(obstacle.width))

            longitudinal_half = 0.5 * (abs_cos * length + abs_sin * width)
            lateral_half = 0.5 * (abs_sin * length + abs_cos * width)
            near_edge = rel_s - longitudinal_half - ego_front_from_base_m
            lateral_limit = ego_half_width_m + lateral_half + self.trigger_lateral_margin_m
            lateral_overlap = abs(projection.d) <= lateral_limit
            ahead_or_overlapping = rel_s + longitudinal_half >= -0.5 * self.vehicle_length_m
            within_range = near_edge <= self.trigger_distance_m
            threatening = bool(lateral_overlap and ahead_or_overlapping and within_range)
            speed = math.hypot(
                float(obstacle.velocity_x_map), float(obstacle.velocity_y_map)
            )
            infos.append(
                ObstacleFrenetInfo(
                    obstacle_id=int(obstacle.id),
                    projection=projection,
                    delta_s_m=float(rel_s),
                    near_edge_distance_m=float(near_edge),
                    longitudinal_half_m=float(longitudinal_half),
                    lateral_half_m=float(lateral_half),
                    speed_mps=float(speed),
                    threatening=threatening,
                )
            )
        return infos

    def _obstacle_markers(
        self,
        obstacles: Optional[LidarObstacleArray],
        infos: List[ObstacleFrenetInfo],
    ):
        result = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        result.markers.append(clear)
        summaries = []
        if obstacles is None:
            return result, summaries

        by_id = {info.obstacle_id: info for info in infos}
        for index, obstacle in enumerate(obstacles.obstacles):
            info = by_id.get(int(obstacle.id))
            if info is None:
                continue

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
            if info.threatening:
                box.color.r, box.color.g, box.color.b, box.color.a = 1.0, 0.08, 0.08, 0.48
            else:
                box.color.r, box.color.g, box.color.b, box.color.a = 0.85, 0.70, 0.15, 0.25
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
                _point(info.projection.reference_x, info.projection.reference_y, 0.15),
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
            label.scale.z = 0.40
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = "id={} ds={:+.1f} d={:+.2f} near={:+.1f} {}".format(
                int(obstacle.id),
                info.delta_s_m,
                info.projection.d,
                info.near_edge_distance_m,
                "THREAT" if info.threatening else "clear",
            )
            result.markers.append(label)

            summaries.append(
                {
                    "id": info.obstacle_id,
                    "delta_s_m": round(info.delta_s_m, 2),
                    "d_m": round(info.projection.d, 2),
                    "near_edge_m": round(info.near_edge_distance_m, 2),
                    "long_half_m": round(info.longitudinal_half_m, 2),
                    "lat_half_m": round(info.lateral_half_m, 2),
                    "speed_mps": round(info.speed_mps, 2),
                    "threat": info.threatening,
                }
            )

        return result, summaries

    def _timer_callback(self, _event) -> None:
        now = rospy.Time.now()
        if self.latest_odom is None or self.latest_odom_received_at is None:
            rospy.logwarn_throttle(3.0, "Waiting for %s", self.odom_topic)
            self._publish_control_bundle(self._candidate_to_ros_path(None), False, False, False)
            return
        odom_age = (now - self.latest_odom_received_at).to_sec()
        if odom_age > self.odom_timeout_s:
            rospy.logwarn_throttle(1.0, "STALE odometry age=%.2fs; planner output suppressed", odom_age)
            self._publish_control_bundle(self._candidate_to_ros_path(None), False, False, False)
            return

        obstacles_fresh = (
            self.latest_obstacles is not None
            and self.latest_obstacles_received_at is not None
            and (now - self.latest_obstacles_received_at).to_sec() <= self.obstacle_timeout_s
        )
        planning_obstacles = self.latest_obstacles if obstacles_fresh else None

        pose = self.latest_odom.pose.pose
        ego_projection = self.reference.project(pose.position.x, pose.position.y)
        ego_yaw = _quaternion_to_yaw(pose.orientation)
        reference_yaw = math.atan2(
            ego_projection.tangent_y, ego_projection.tangent_x
        )
        ego_heading_error = _normalize_angle(ego_yaw - reference_yaw)
        # dd/ds = tan(heading error). Clamp noisy/localization spikes so a
        # single bad yaw sample cannot create an extreme replanning polynomial.
        ego_d_slope = max(-0.70, min(0.70, math.tan(ego_heading_error)))
        reference_point = self.reference.frenet_to_map(ego_projection.s, 0.0)
        current_link = self.link_index.nearest_link(reference_point.x, reference_point.y)

        if current_link is None:
            rospy.logwarn_throttle(2.0, "No MGeo link near current global reference point")
            self._publish_control_bundle(self._candidate_to_ros_path(None), False, False, False)
            return

        obstacle_infos = self._obstacle_infos(planning_obstacles, ego_projection)
        threats = [info for info in obstacle_infos if info.threatening]
        threats.sort(key=lambda info: info.near_edge_distance_m)
        active_threat = threats[0] if threats else None
        adjacent = available_adjacent_links(current_link, self.links)

        lane_candidates: List[FrenetLaneChangeCandidate] = []
        bypass_candidates: List[FrenetBypassCandidate] = []
        mode = "NORMAL"

        if active_threat is not None or self.always_show_lane_candidates:
            if adjacent:
                lane_candidates = generate_frenet_lane_change_candidates(
                    reference=self.reference,
                    ego_s=ego_projection.s,
                    current_link=current_link,
                    links=self.links,
                    start_distances_m=self.start_distances_m,
                    change_lengths_m=self.change_lengths_m,
                    local_length_m=self.local_length_m,
                    sample_spacing_m=self.sample_spacing_m,
                )
                if active_threat is not None:
                    mode = "LANE_CHANGE_CANDIDATES"
                elif lane_candidates:
                    mode = "DEBUG_LANE_CANDIDATES"

        if active_threat is not None and (not adjacent or self.allow_bypass_with_adjacent_lane):
            bypass_candidates = generate_frenet_bypass_candidates(
                reference=self.reference,
                ego_s=ego_projection.s,
                ego_d=ego_projection.d,
                ego_d_slope=ego_d_slope,
                obstacle_id=active_threat.obstacle_id,
                obstacle_s=active_threat.projection.s,
                obstacle_d=active_threat.projection.d,
                obstacle_longitudinal_half_m=active_threat.longitudinal_half_m,
                obstacle_lateral_half_m=active_threat.lateral_half_m,
                vehicle_width_m=self.vehicle_width_m,
                lateral_margin_m=self.bypass_lateral_margin_m,
                longitudinal_margin_m=self.bypass_longitudinal_margin_m,
                departure_length_m=self.bypass_departure_length_m,
                return_length_m=self.bypass_return_length_m,
                extra_clearances_m=self.bypass_extra_clearances_m,
                max_abs_d_m=self.bypass_max_abs_d_m,
                local_length_m=self.local_length_m,
                sample_spacing_m=self.sample_spacing_m,
                min_transition_length_m=self.bypass_min_transition_length_m,
            )
            mode = "BYPASS_CANDIDATES" if bypass_candidates else "BYPASS_NO_GEOMETRIC_CANDIDATE"

        candidates: List[Candidate] = list(lane_candidates) + list(bypass_candidates)

        safety_obstacles = []
        if planning_obstacles is not None:
            for obs in planning_obstacles.obstacles:
                safety_obstacles.append(
                    ObstacleBox(
                        obstacle_id=int(obs.id),
                        center_x=float(obs.center_x_map),
                        center_y=float(obs.center_y_map),
                        yaw=float(obs.yaw),
                        length=max(0.10, float(obs.length)),
                        width=max(0.10, float(obs.width)),
                    )
                )

        evaluations: List[CandidateEvaluation] = []
        for candidate_index, candidate in enumerate(candidates):
            evaluations.append(
                evaluate_candidate(
                    candidate_index=candidate_index,
                    candidate=candidate,
                    obstacles=safety_obstacles,
                    vehicle_length_m=self.vehicle_length_m,
                    vehicle_width_m=self.vehicle_width_m,
                    vehicle_center_from_base_m=self.vehicle_center_from_base_m,
                    collision_longitudinal_margin_m=self.collision_longitudinal_margin_m,
                    collision_lateral_margin_m=self.collision_lateral_margin_m,
                    wheelbase_m=self.wheelbase_m,
                    max_steering_rad=self.max_steering_rad,
                    evaluation_speed_mps=self.evaluation_speed_mps,
                    max_lateral_accel_mps2=self.max_lateral_accel_mps2,
                    collision_sample_stride=self.collision_sample_stride,
                    escape_prefix_m=self.escape_prefix_m,
                )
            )

        valid_evaluations = [ev for ev in evaluations if ev.valid]
        selected_evaluation = min(valid_evaluations, key=lambda ev: ev.cost) if valid_evaluations else None
        selected_index = selected_evaluation.candidate_index if selected_evaluation is not None else None
        selected_candidate = candidates[selected_index] if selected_index is not None else None
        selected_path_msg = self._candidate_to_ros_path(selected_candidate)
        evaluation_markers = self._evaluation_markers(candidates, evaluations, selected_index)

        planner_ready = bool(obstacles_fresh)
        self._publish_control_bundle(
            path_msg=selected_path_msg,
            planner_ready=planner_ready,
            avoidance_required=active_threat is not None,
            safe_path_available=selected_candidate is not None,
            selected_kind=getattr(selected_candidate, "kind", "") if selected_candidate else "",
            selected_side=getattr(selected_candidate, "side", "") if selected_candidate else "",
        )

        obstacle_markers, obstacle_summaries = self._obstacle_markers(
            planning_obstacles, obstacle_infos
        )
        ego_markers = self._ego_markers(self.latest_odom)
        link_markers = self._link_markers(current_link)
        candidate_markers = self._candidate_markers(candidates)
        corridor_markers = self._candidate_corridor_markers(candidates)

        # RViz geometry is published only in the moving LiDAR frame.
        # The map-frame selected_path remains available as the Path Manager's
        # control-facing interface; scalar/JSON diagnostics remain unchanged.
        if self.publish_lidar_visualization:
            self.lidar_global_pub.publish(
                self._lidar_local_global_path(ego_projection.s, self.latest_odom)
            )
            self.lidar_ego_pub.publish(
                self._marker_array_to_lidar(ego_markers, self.latest_odom)
            )
            self.lidar_obstacle_pub.publish(
                self._marker_array_to_lidar(obstacle_markers, self.latest_odom)
            )
            self.lidar_link_pub.publish(
                self._marker_array_to_lidar(link_markers, self.latest_odom)
            )
            self.lidar_candidate_pub.publish(
                self._marker_array_to_lidar(candidate_markers, self.latest_odom)
            )
            self.lidar_corridor_pub.publish(
                self._marker_array_to_lidar(corridor_markers, self.latest_odom)
            )
            self.lidar_selected_path_pub.publish(
                self._ros_path_to_lidar(selected_path_msg, self.latest_odom)
            )
            self.lidar_evaluation_pub.publish(
                self._marker_array_to_lidar(evaluation_markers, self.latest_odom)
            )

        debug_payload = {
            "mode": mode,
            "ego": {
                "s_m": round(ego_projection.s, 3),
                "d_m": round(ego_projection.d, 3),
                "heading_error_deg": round(math.degrees(ego_heading_error), 3),
                "d_slope": round(ego_d_slope, 4),
            },
            "vehicle": {
                "length_m": self.vehicle_length_m,
                "width_m": self.vehicle_width_m,
                "height_m": self.vehicle_height_m,
                "center_from_base_m": self.vehicle_center_from_base_m,
            },
            "current_link": current_link.idx,
            "ego_lane": current_link.ego_lane,
            "link_width_start_m": current_link.width_start_m,
            "link_width_end_m": current_link.width_end_m,
            "adjacent_links": [
                {"side": side, "link": link.idx} for side, link in adjacent
            ],
            "threat_count": len(threats),
            "active_threat_id": active_threat.obstacle_id if active_threat else None,
            "lane_candidate_count": len(lane_candidates),
            "bypass_candidate_count": len(bypass_candidates),
            "candidate_count": len(candidates),
            "safe_candidate_count": len(valid_evaluations),
            "selected_candidate_index": selected_index,
            "selected_candidate_kind": getattr(selected_candidate, "kind", None) if selected_candidate else None,
            "selected_candidate_side": getattr(selected_candidate, "side", None) if selected_candidate else None,
            "selected_cost": round(selected_evaluation.cost, 4) if selected_evaluation else None,
            "input_freshness": {
                "odom_age_s": round(odom_age, 3),
                "obstacles_fresh": obstacles_fresh,
                "obstacle_age_s": round((now - self.latest_obstacles_received_at).to_sec(), 3) if self.latest_obstacles_received_at else None,
            },
            "candidate_evaluations": [
                {
                    "index": ev.candidate_index,
                    "valid": ev.valid,
                    "reason": ev.reason,
                    "collision_obstacle_id": ev.collision_obstacle_id,
                    "first_collision_path_index": ev.first_collision_path_index,
                    "max_curvature_1pm": round(ev.max_curvature_1pm, 4),
                    "max_steering_rad": round(ev.max_steering_rad, 4),
                    "max_lateral_accel_mps2": round(ev.max_lateral_accel_mps2, 3),
                    "cost": round(ev.cost, 4) if math.isfinite(ev.cost) else None,
                    "escape_prefix_used": bool(getattr(ev, "escape_prefix_used", False)),
                    "escape_obstacle_id": getattr(ev, "escape_obstacle_id", None),
                } for ev in evaluations
            ],
            "obstacle_count": len(obstacle_summaries),
            "obstacles": obstacle_summaries,
            "escape_prefix_m": self.escape_prefix_m,
            "plan_seq": self.plan_seq,
            "bypass_note": "debug free-space only; road polygon/boundary check not implemented",
            "lidar_visualization": {
                "enabled": self.publish_lidar_visualization,
                "frame": self.lidar_viz_frame,
                "prefix": self.lidar_viz_prefix,
                "extrinsic": {
                    "x_m": self.lidar_x_m,
                    "y_m": self.lidar_y_m,
                    "z_m": self.lidar_z_m,
                    "yaw_deg": round(math.degrees(self.lidar_yaw_rad), 3),
                },
            },
        }
        self.debug_pub.publish(
            String(data=json.dumps(debug_payload, ensure_ascii=False, separators=(",", ":")))
        )

        if self.last_current_link_idx != current_link.idx:
            rospy.loginfo(
                "Current MGeo link=%s lane=%s width=(%s,%s) left=%s right=%s",
                current_link.idx,
                str(current_link.ego_lane),
                str(current_link.width_start_m),
                str(current_link.width_end_m),
                str(current_link.left_lane_change_dst_link_idx),
                str(current_link.right_lane_change_dst_link_idx),
            )
            self.last_current_link_idx = current_link.idx

        rospy.loginfo_throttle(
            1.0,
            "Frenet F5 mode=%s ego(s=%.1f,d=%+.2f) threats=%d lane=%d bypass=%d",
            mode,
            ego_projection.s,
            ego_projection.d,
            len(threats),
            len(lane_candidates),
            len(bypass_candidates),
        )


def main() -> None:
    try:
        AvoidanceFrenetDebugNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
