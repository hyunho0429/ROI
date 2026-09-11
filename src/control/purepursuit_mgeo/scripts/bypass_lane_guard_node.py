#!/usr/bin/env python3
"""Frenet BYPASS forwarding/safety supervisor.

Obstacle-triggered lane changes are disabled by default.  In the competition
configuration, normal F6 BYPASS is intentionally lane-agnostic: camera
lane_info is NOT required and lane boundaries do NOT veto a base planner BYPASS.
The base F6 planner still performs obstacle OBB collision, curvature, steering,
lateral-acceleration and max-|d| feasibility checks.

This node sits between the existing F6 bypass planner and the Path Manager.
It never publishes /ctrl_cmd.

Responsibilities
----------------
* consume the base planner's atomic selected_path + plan_status;
* consume /perception/camera/lane_info (std_msgs/String JSON);
* when an obstacle blocks the global lane, generate full-lane left/right
  avoidance candidates only across a detected dashed boundary;
* derive the adjacent-lane offset from camera lane center + measured lane width;
* reject target lanes that fail dynamic front/rear gap/TTC checks;
* evaluate candidate OBB collision, steering and lateral acceleration with the
  same trajectory_safety module as the base planner;
* prefer a legal/safe lane-change avoidance over free-space bypass;
* forward a base F6 BYPASS regardless of camera lane validity or lane markings;
* keep camera lane semantics reserved for the separate highway strategic
  lane-change controller, not for obstacle BYPASS permission;
* publish a new atomic path/status bundle for the Path Manager.

Lane-change paths leave the current lane, hold the adjacent lane through the
blocking obstacle, then return to d=0 on the original global route.  This makes
Path Manager completion semantics identical to the existing bypass maneuver.
"""

from __future__ import annotations

import json
import math
import statistics
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool, String

from lidar_perception.msg import LidarObstacleArray
from purepursuit_mgeo.path import PathPoint, load_mgeo_path
from frenet_path import ReferencePath
from trajectory_safety import ObstacleBox, CandidateEvaluation, evaluate_candidate


def _normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _quintic_smoothstep(u: float) -> float:
    u = max(0.0, min(1.0, float(u)))
    return 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5


def _quintic_lateral_transition(
    start_d: float,
    target_d: float,
    start_slope: float,
    length_m: float,
    u: float,
) -> float:
    """Quintic d(s) with d/ds and d2/ds2 endpoint continuity."""
    L = max(float(length_m), 1.0e-6)
    x = max(0.0, min(1.0, float(u))) * L
    d0 = float(start_d)
    d1 = float(target_d)
    v0 = float(start_slope)
    c0 = d0
    c1 = v0
    c2 = 0.0
    c3 = (20.0 * (d1 - d0) - 12.0 * v0 * L) / (2.0 * L ** 3)
    c4 = (30.0 * (d0 - d1) + 16.0 * v0 * L) / (2.0 * L ** 4)
    c5 = (12.0 * (d1 - d0) - 6.0 * v0 * L) / (2.0 * L ** 5)
    return c0 + c1 * x + c2 * x ** 2 + c3 * x ** 3 + c4 * x ** 4 + c5 * x ** 5


def _append_unique(path: List[PathPoint], p: PathPoint, min_dist: float = 0.05) -> None:
    if path:
        d = math.hypot(path[-1].x - p.x, path[-1].y - p.y)
        if d < min_dist:
            return
    path.append(p)


@dataclass
class ThreatInfo:
    obstacle_id: int
    s: float
    d: float
    rel_s: float
    near_edge_m: float
    longitudinal_half_m: float
    lateral_half_m: float


@dataclass
class LaneChangeAvoidanceCandidate:
    kind: str
    side: str
    target_d_m: float
    change_length_m: float
    obstacle_id: int
    hold_start_s: float
    hold_end_s: float
    return_end_s: float
    path: List[PathPoint]


class BypassLaneGuard:
    def __init__(self) -> None:
        rospy.init_node("bypass_lane_guard", anonymous=False)

        path_file = rospy.get_param("~path_file")
        global_points = load_mgeo_path(path_file)
        self.reference = ReferencePath(global_points)
        endpoint_gap = math.hypot(
            global_points[-1].x - global_points[0].x,
            global_points[-1].y - global_points[0].y,
        )
        self.closed_loop = endpoint_gap <= float(
            rospy.get_param("~closed_loop_endpoint_tolerance_m", 1.0)
        )

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_selected_path_topic = rospy.get_param(
            "~base_selected_path_topic", "/avoidance_frenet_debug/selected_path"
        )
        self.base_plan_status_topic = rospy.get_param(
            "~base_plan_status_topic", "/avoidance_frenet_debug/plan_status"
        )
        self.lane_info_topic = rospy.get_param(
            "~lane_info_topic", "/perception/camera/lane_info"
        )
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odometry")
        self.obstacle_topic = rospy.get_param(
            "~obstacle_topic", "/perception/lidar/tracked_obstacles_map"
        )
        self.highway_environment_topic = rospy.get_param(
            "~highway_environment_topic", "/perception/camera/highway_environment"
        )
        self.merge_available_topic = rospy.get_param(
            "~merge_available_topic", "/perception/merge_gap/available"
        )
        self.merge_unavailable_topic = rospy.get_param(
            "~merge_unavailable_topic", "/perception/merge_gap/unavailable"
        )

        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.base_timeout_s = float(rospy.get_param("~base_timeout_s", 0.7))
        self.lane_info_timeout_s = float(rospy.get_param("~lane_info_timeout_s", 0.55))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.50))
        self.obstacle_timeout_s = float(rospy.get_param("~obstacle_timeout_s", 0.60))

        self.vehicle_length_m = float(rospy.get_param("~vehicle_length_m", 4.635))
        self.vehicle_width_m = float(rospy.get_param("~vehicle_width_m", 1.892))
        self.vehicle_center_from_base_m = float(
            rospy.get_param("~vehicle_center_from_base_m", 1.50)
        )
        self.trigger_distance_m = float(rospy.get_param("~trigger_distance_m", 35.0))
        self.trigger_lateral_margin_m = float(
            rospy.get_param("~trigger_lateral_margin_m", 0.35)
        )

        self.min_lane_confidence = float(
            rospy.get_param("~min_lane_confidence", 0.55)
        )
        self.max_lane_heading_error_rad = float(
            rospy.get_param("~max_lane_heading_error_rad", math.radians(22.0))
        )
        self.lane_width_min_m = float(rospy.get_param("~lane_width_min_m", 2.7))
        self.lane_width_max_m = float(rospy.get_param("~lane_width_max_m", 4.2))
        self.max_abs_target_d_m = float(rospy.get_param("~max_abs_target_d_m", 4.8))
        self.camera_center_min_points = int(
            rospy.get_param("~camera_center_min_points", 3)
        )
        self.require_adjacent_far_boundary = bool(
            rospy.get_param("~require_adjacent_far_boundary", True)
        )
        self.adjacent_boundary_extrap_m = float(
            rospy.get_param("~adjacent_boundary_extrap_m", 1.0)
        )
        self.adjacent_lane_width_change_max_m = float(
            rospy.get_param("~adjacent_lane_width_change_max_m", 0.8)
        )

        self.local_length_m = float(rospy.get_param("~local_length_m", 75.0))
        self.sample_spacing_m = float(rospy.get_param("~sample_spacing_m", 0.5))
        self.lane_change_departure_lengths_m = self._float_list_param(
            "~lane_change_departure_lengths_m", [14.0, 18.0]
        )
        self.lane_change_return_length_m = float(
            rospy.get_param("~lane_change_return_length_m", 18.0)
        )
        self.lane_change_longitudinal_margin_m = float(
            rospy.get_param("~lane_change_longitudinal_margin_m", 4.0)
        )
        self.lane_change_min_transition_m = float(
            rospy.get_param("~lane_change_min_transition_m", 6.0)
        )

        self.target_lane_lateral_allowance_m = float(
            rospy.get_param("~target_lane_lateral_allowance_m", 0.45)
        )
        self.gap_search_range_m = float(rospy.get_param("~gap_search_range_m", 45.0))
        self.front_min_gap_m = float(rospy.get_param("~front_min_gap_m", 5.0))
        self.rear_min_gap_m = float(rospy.get_param("~rear_min_gap_m", 6.0))
        self.gap_time_headway_s = float(rospy.get_param("~gap_time_headway_s", 1.5))
        self.gap_min_ttc_s = float(rospy.get_param("~gap_min_ttc_s", 3.0))
        self.use_existing_left_merge_gap_on_highway = bool(
            rospy.get_param("~use_existing_left_merge_gap_on_highway", True)
        )
        self.merge_topic_timeout_s = float(
            rospy.get_param("~merge_topic_timeout_s", 0.7)
        )

        self.collision_longitudinal_margin_m = float(
            rospy.get_param("~collision_longitudinal_margin_m", 0.40)
        )
        self.collision_lateral_margin_m = float(
            rospy.get_param("~collision_lateral_margin_m", 0.45)
        )
        self.wheelbase_m = float(rospy.get_param("~wheelbase_m", 3.0))
        self.max_steering_rad = float(
            rospy.get_param("~max_steering_rad", math.radians(40.0))
        )
        self.evaluation_speed_mps = float(
            rospy.get_param("~evaluation_speed_mps", 2.0)
        )
        self.max_lateral_accel_mps2 = float(
            rospy.get_param("~max_lateral_accel_mps2", 2.5)
        )
        self.collision_sample_stride = int(
            rospy.get_param("~collision_sample_stride", 1)
        )
        self.escape_prefix_m = float(rospy.get_param("~escape_prefix_m", 3.0))

        self.prefer_lane_change = bool(rospy.get_param("~prefer_lane_change", False))
        self.enable_obstacle_lane_change = bool(rospy.get_param("~enable_obstacle_lane_change", False))
        self.enforce_camera_lane_bounds_on_bypass = bool(
            rospy.get_param("~enforce_camera_lane_bounds_on_bypass", False)
        )
        self.bypass_lane_boundary_margin_m = float(
            rospy.get_param("~bypass_lane_boundary_margin_m", 0.15)
        )
        self.allow_bypass_without_lane_info = bool(
            rospy.get_param("~allow_bypass_without_lane_info", False)
        )
        self.fallback_bypass_max_abs_d_m = float(
            rospy.get_param("~fallback_bypass_max_abs_d_m", 0.25)
        )

        # Intersection-only BYPASS policy.  Camera lane geometry is intentionally
        # not required here because the competition intersection uses guide lines
        # that the camera team does not classify as normal lane boundaries.
        self.enable_intersection_bypass = bool(
            rospy.get_param("~enable_intersection_bypass", True)
        )
        self.intersection_detected_topic = rospy.get_param(
            "~intersection_detected_topic", "/perception/intersection/detected"
        )
        self.intersection_bypass_request_topic = rospy.get_param(
            "~intersection_bypass_request_topic", "/planning/intersection_bypass_zone"
        )
        self.intersection_hold_s = float(
            rospy.get_param("~intersection_hold_s", 8.0)
        )
        self.intersection_bypass_max_abs_d_m = float(
            rospy.get_param("~intersection_bypass_max_abs_d_m", 3.8)
        )

        self.base_path: Optional[RosPath] = None
        self.base_path_at: Optional[rospy.Time] = None
        self.base_path_seq: Optional[int] = None
        self.base_status: Dict[str, object] = {}
        self.base_status_at: Optional[rospy.Time] = None
        self.base_status_seq: Optional[int] = None

        self.lane_info: Optional[Dict[str, object]] = None
        self.lane_info_at: Optional[rospy.Time] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_at: Optional[rospy.Time] = None
        self.latest_obstacles: Optional[LidarObstacleArray] = None
        self.latest_obstacles_at: Optional[rospy.Time] = None

        self.highway_environment = False
        self.highway_environment_at: Optional[rospy.Time] = None
        self.intersection_detected = False
        self.intersection_last_true_at: Optional[rospy.Time] = None
        self.intersection_bypass_request = False
        self.merge_available = False
        self.merge_unavailable = False
        self.merge_gap_at: Optional[rospy.Time] = None

        self.plan_seq = 0
        self.selected_path_pub = rospy.Publisher("~selected_path", RosPath, queue_size=1)
        self.plan_status_pub = rospy.Publisher("~plan_status", String, queue_size=1)
        self.planner_ready_pub = rospy.Publisher("~planner_ready", Bool, queue_size=1)
        self.avoidance_required_pub = rospy.Publisher("~avoidance_required", Bool, queue_size=1)
        self.safe_path_available_pub = rospy.Publisher("~safe_path_available", Bool, queue_size=1)
        self.selected_kind_pub = rospy.Publisher("~selected_kind", String, queue_size=1)
        self.selected_side_pub = rospy.Publisher("~selected_side", String, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        self.lane_change_path_pub = rospy.Publisher("~lane_change_path", RosPath, queue_size=1)

        rospy.Subscriber(self.base_selected_path_topic, RosPath, self._base_path_cb, queue_size=1)
        rospy.Subscriber(self.base_plan_status_topic, String, self._base_status_cb, queue_size=1)
        rospy.Subscriber(self.lane_info_topic, String, self._lane_info_cb, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber(self.obstacle_topic, LidarObstacleArray, self._obstacles_cb, queue_size=1)
        rospy.Subscriber(self.highway_environment_topic, Bool, self._highway_cb, queue_size=1)
        rospy.Subscriber(self.intersection_detected_topic, Bool, self._intersection_cb, queue_size=1)
        rospy.Subscriber(self.intersection_bypass_request_topic, Bool, self._intersection_request_cb, queue_size=1)
        rospy.Subscriber(self.merge_available_topic, Bool, self._merge_available_cb, queue_size=1)
        rospy.Subscriber(self.merge_unavailable_topic, Bool, self._merge_unavailable_cb, queue_size=1)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self._timer_cb
        )
        rospy.logwarn(
            "BYPASS guard started: lane_agnostic=%s camera_lane_info=%s obstacle_lane_change=%s",
            (not self.enforce_camera_lane_bounds_on_bypass),
            self.lane_info_topic,
            self.enable_obstacle_lane_change,
        )

    @staticmethod
    def _float_list_param(name: str, default: Sequence[float]) -> List[float]:
        raw = rospy.get_param(name, list(default))
        if isinstance(raw, str):
            try:
                raw = json.loads(raw)
            except Exception:
                raw = [x.strip() for x in raw.split(",") if x.strip()]
        try:
            return [float(x) for x in raw]
        except Exception:
            return [float(x) for x in default]

    def _base_path_cb(self, msg: RosPath) -> None:
        self.base_path = msg
        self.base_path_at = rospy.Time.now()
        self.base_path_seq = int(msg.header.seq)

    def _base_status_cb(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data or "{}"))
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Invalid base plan_status JSON: %s", str(exc))
            return
        self.base_status = payload
        self.base_status_at = rospy.Time.now()
        self.base_status_seq = int(payload.get("seq", -1))

    def _lane_info_cb(self, msg: String) -> None:
        try:
            self.lane_info = json.loads(str(msg.data or "{}"))
            self.lane_info_at = rospy.Time.now()
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Invalid lane_info JSON: %s", str(exc))

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_odom_at = rospy.Time.now()

    def _obstacles_cb(self, msg: LidarObstacleArray) -> None:
        self.latest_obstacles = msg
        self.latest_obstacles_at = rospy.Time.now()

    def _highway_cb(self, msg: Bool) -> None:
        self.highway_environment = bool(msg.data)
        self.highway_environment_at = rospy.Time.now()

    def _intersection_cb(self, msg: Bool) -> None:
        now = rospy.Time.now()
        self.intersection_detected = bool(msg.data)
        if self.intersection_detected:
            self.intersection_last_true_at = now

    def _intersection_request_cb(self, msg: Bool) -> None:
        self.intersection_bypass_request = bool(msg.data)

    def _intersection_bypass_active(self, now: rospy.Time) -> Tuple[bool, str]:
        if not self.enable_intersection_bypass:
            return False, "disabled"
        if self.intersection_bypass_request:
            return True, "mission_request"
        if self.intersection_detected:
            return True, "sensor_detected"
        if self.intersection_last_true_at is not None:
            age = (now - self.intersection_last_true_at).to_sec()
            if 0.0 <= age <= self.intersection_hold_s:
                return True, "sensor_latched"
        return False, "inactive"

    def _merge_available_cb(self, msg: Bool) -> None:
        self.merge_available = bool(msg.data)
        self.merge_gap_at = rospy.Time.now()

    def _merge_unavailable_cb(self, msg: Bool) -> None:
        self.merge_unavailable = bool(msg.data)
        self.merge_gap_at = rospy.Time.now()

    def _signed_s_delta(self, s: float, ego_s: float) -> float:
        delta = float(s) - float(ego_s)
        if not self.closed_loop:
            return delta
        L = max(float(self.reference.total_length_m), 1.0e-6)
        while delta > 0.5 * L:
            delta -= L
        while delta < -0.5 * L:
            delta += L
        return delta

    def _base_bundle_fresh(self, now: rospy.Time) -> bool:
        if self.base_path_at is None or self.base_status_at is None:
            return False
        if (now - self.base_path_at).to_sec() > self.base_timeout_s:
            return False
        if (now - self.base_status_at).to_sec() > self.base_timeout_s:
            return False
        return (
            self.base_path_seq is not None
            and self.base_status_seq is not None
            and int(self.base_path_seq) == int(self.base_status_seq)
        )

    def _sensors_fresh(self, now: rospy.Time) -> bool:
        if self.latest_odom_at is None or self.latest_obstacles_at is None:
            return False
        return (
            (now - self.latest_odom_at).to_sec() <= self.odom_timeout_s
            and (now - self.latest_obstacles_at).to_sec() <= self.obstacle_timeout_s
        )

    def _lane_snapshot(self, now: rospy.Time) -> Tuple[Optional[dict], str]:
        if self.lane_info is None or self.lane_info_at is None:
            return None, "lane_info_missing"
        if (now - self.lane_info_at).to_sec() > self.lane_info_timeout_s:
            return None, "lane_info_stale"
        info = self.lane_info
        if not bool(info.get("lane_valid", False)):
            return None, "lane_invalid"
        confidence = float(info.get("confidence", 0.0) or 0.0)
        if confidence < self.min_lane_confidence:
            return None, "lane_low_confidence"
        heading = info.get("heading_error_rad")
        if heading is None or abs(float(heading)) > self.max_lane_heading_error_rad:
            return None, "lane_heading_error"
        width = info.get("lane_width_m")
        if width is None:
            return None, "lane_width_missing"
        width = float(width)
        if not (self.lane_width_min_m <= width <= self.lane_width_max_m):
            return None, "lane_width_invalid"
        center_pts = info.get("centerline_points") or []
        if len(center_pts) < self.camera_center_min_points:
            return None, "centerline_too_short"
        return info, "ok"

    def _camera_center_d(self, info: dict, ego_projection, pose) -> Optional[float]:
        yaw = _yaw_from_quaternion(pose.orientation)
        c = math.cos(yaw)
        s = math.sin(yaw)
        ds: List[float] = []
        for raw in info.get("centerline_points") or []:
            try:
                bx, by = float(raw[0]), float(raw[1])
            except Exception:
                continue
            mx = float(pose.position.x) + c * bx - s * by
            my = float(pose.position.y) + s * bx + c * by
            proj = self.reference.project(mx, my)
            rel = self._signed_s_delta(proj.s, ego_projection.s)
            if -1.0 <= rel <= 35.0:
                ds.append(float(proj.d))
        if len(ds) < self.camera_center_min_points:
            return None
        return float(statistics.median(ds))

    @staticmethod
    def _boundary_allows(info: dict, side: str) -> Tuple[bool, str]:
        key = "left_lane" if side == "left" else "right_lane"
        lane = info.get(key) or {}
        if not bool(lane.get("detected", False)):
            return False, f"{side}_boundary_missing"
        lane_type = str(lane.get("type") or "").lower()
        dashed = lane.get("dashed")
        if lane_type == "yellow" or "solid" in lane_type:
            return False, f"{side}_boundary_solid"
        if dashed is not True:
            return False, f"{side}_boundary_not_dashed"
        return True, "ok"

    def _lane_meta_y(self, lane: dict, x: float) -> Optional[float]:
        coef = lane.get("coef")
        if not coef:
            return None
        xr = lane.get("x_range_m")
        if isinstance(xr, (list, tuple)) and len(xr) >= 2:
            lo, hi = float(xr[0]), float(xr[1])
            if x < lo - self.adjacent_boundary_extrap_m or x > hi + self.adjacent_boundary_extrap_m:
                return None
        try:
            y = 0.0
            for c in coef:
                y = y * float(x) + float(c)
            return y if math.isfinite(y) else None
        except Exception:
            return None

    def _adjacent_target_geometry(
        self,
        info: dict,
        side: str,
        ego_projection,
        pose,
    ) -> Tuple[Optional[float], Optional[float], str]:
        """Confirm an actual adjacent lane using the far-side camera boundary.

        A dashed ego boundary alone is not sufficient: near merges/gore areas a
        dashed line may not guarantee a usable parallel destination lane.  We
        therefore require another detected lane marking on the far side and
        build the adjacent center from the two real boundaries.
        """
        key = "left_lane" if side == "left" else "right_lane"
        near = info.get(key) or {}
        all_lanes = info.get("all_lanes") or []
        if not self.require_adjacent_far_boundary:
            width = float(info.get("lane_width_m"))
            current_center = self._camera_center_d(info, ego_projection, pose)
            if current_center is None:
                return None, None, "current_center_missing"
            return (
                current_center + (width if side == "left" else -width),
                width,
                "inferred_from_current_width",
            )
        if not all_lanes:
            return None, None, "adjacent_far_boundary_missing"

        near_track = near.get("track_id")
        xs = [5.0, 8.0, 11.0, 14.0, 17.0, 20.0, 23.0]
        candidates = []
        for lane in all_lanes:
            if not isinstance(lane, dict) or not bool(lane.get("detected", False)):
                continue
            if near_track is not None and lane.get("track_id") == near_track:
                continue
            sep_samples = []
            center_samples = []
            for x in xs:
                yn = self._lane_meta_y(near, x)
                yf = self._lane_meta_y(lane, x)
                if yn is None or yf is None:
                    continue
                sep = (yf - yn) if side == "left" else (yn - yf)
                if sep <= 0.20:
                    continue
                sep_samples.append(sep)
                center_samples.append((x, 0.5 * (yn + yf)))
            if len(sep_samples) < self.camera_center_min_points:
                continue
            width = float(statistics.median(sep_samples))
            if not (self.lane_width_min_m <= width <= self.lane_width_max_m):
                continue
            if max(sep_samples) - min(sep_samples) > self.adjacent_lane_width_change_max_m:
                continue
            candidates.append((abs(width - float(info.get("lane_width_m"))), width, center_samples))

        if not candidates:
            return None, None, "adjacent_far_boundary_not_valid"
        _score, width, center_samples = min(candidates, key=lambda x: x[0])

        yaw = _yaw_from_quaternion(pose.orientation)
        c = math.cos(yaw)
        ss = math.sin(yaw)
        ds = []
        for bx, by in center_samples:
            mx = float(pose.position.x) + c * bx - ss * by
            my = float(pose.position.y) + ss * bx + c * by
            proj = self.reference.project(mx, my)
            rel = self._signed_s_delta(proj.s, ego_projection.s)
            if -1.0 <= rel <= 35.0:
                ds.append(float(proj.d))
        if len(ds) < self.camera_center_min_points:
            return None, None, "adjacent_center_projection_failed"
        return float(statistics.median(ds)), float(width), "ok"

    def _threatening_obstacles(self, ego_projection) -> List[ThreatInfo]:
        if self.latest_obstacles is None:
            return []
        ego_front = self.vehicle_center_from_base_m + 0.5 * self.vehicle_length_m
        ego_half_w = 0.5 * self.vehicle_width_m
        threats: List[ThreatInfo] = []
        for obs in self.latest_obstacles.obstacles:
            proj = self.reference.project(float(obs.center_x_map), float(obs.center_y_map))
            rel_s = self._signed_s_delta(proj.s, ego_projection.s)
            ref_yaw = math.atan2(proj.tangent_y, proj.tangent_x)
            rel_yaw = _normalize_angle(float(obs.yaw) - ref_yaw)
            ac = abs(math.cos(rel_yaw))
            asi = abs(math.sin(rel_yaw))
            length = max(0.10, float(obs.length))
            width = max(0.10, float(obs.width))
            long_half = 0.5 * (ac * length + asi * width)
            lat_half = 0.5 * (asi * length + ac * width)
            near = rel_s - long_half - ego_front
            lateral_overlap = abs(proj.d) <= ego_half_w + lat_half + self.trigger_lateral_margin_m
            ahead_or_overlap = rel_s + long_half >= -0.5 * self.vehicle_length_m
            if lateral_overlap and ahead_or_overlap and near <= self.trigger_distance_m:
                threats.append(
                    ThreatInfo(
                        obstacle_id=int(obs.id),
                        s=float(proj.s),
                        d=float(proj.d),
                        rel_s=float(rel_s),
                        near_edge_m=float(near),
                        longitudinal_half_m=float(long_half),
                        lateral_half_m=float(lat_half),
                    )
                )
        threats.sort(key=lambda x: x.near_edge_m)
        return threats

    def _safety_boxes(self) -> List[ObstacleBox]:
        boxes: List[ObstacleBox] = []
        if self.latest_obstacles is None:
            return boxes
        for obs in self.latest_obstacles.obstacles:
            boxes.append(
                ObstacleBox(
                    obstacle_id=int(obs.id),
                    center_x=float(obs.center_x_map),
                    center_y=float(obs.center_y_map),
                    yaw=float(obs.yaw),
                    length=max(0.10, float(obs.length)),
                    width=max(0.10, float(obs.width)),
                )
            )
        return boxes

    def _target_lane_gap_safe(
        self,
        target_d: float,
        lane_width: float,
        ego_projection,
        ego_speed: float,
        active_threat_id: int,
        side: str,
        now: rospy.Time,
    ) -> Tuple[bool, str, List[dict]]:
        diagnostics: List[dict] = []
        if self.latest_obstacles is None:
            return False, "obstacles_missing", diagnostics

        ego_front = self.vehicle_center_from_base_m + 0.5 * self.vehicle_length_m
        ego_rear = max(0.0, 0.5 * self.vehicle_length_m - self.vehicle_center_from_base_m)

        for obs in self.latest_obstacles.obstacles:
            proj = self.reference.project(float(obs.center_x_map), float(obs.center_y_map))
            rel_s = self._signed_s_delta(proj.s, ego_projection.s)
            if abs(rel_s) > self.gap_search_range_m:
                continue
            ref_yaw = math.atan2(proj.tangent_y, proj.tangent_x)
            rel_yaw = _normalize_angle(float(obs.yaw) - ref_yaw)
            ac = abs(math.cos(rel_yaw))
            asi = abs(math.sin(rel_yaw))
            length = max(0.10, float(obs.length))
            width = max(0.10, float(obs.width))
            long_half = 0.5 * (ac * length + asi * width)
            lat_half = 0.5 * (asi * length + ac * width)

            target_overlap = abs(float(proj.d) - target_d) <= (
                0.5 * lane_width + lat_half + self.target_lane_lateral_allowance_m
            )
            if not target_overlap:
                continue

            obs_long_speed = (
                float(obs.velocity_x_map) * proj.tangent_x
                + float(obs.velocity_y_map) * proj.tangent_y
            )
            item = {
                "id": int(obs.id),
                "ds": round(rel_s, 2),
                "d": round(float(proj.d), 2),
                "v_long": round(obs_long_speed, 2),
            }

            if rel_s >= 0.0:
                clearance = rel_s - long_half - ego_front
                required = max(
                    self.front_min_gap_m,
                    self.gap_time_headway_s * max(ego_speed, 0.0),
                )
                closing = ego_speed - obs_long_speed
                ttc = clearance / closing if closing > 0.2 and clearance > 0.0 else float("inf")
                item.update({"position": "front", "clearance": round(clearance, 2), "ttc": None if not math.isfinite(ttc) else round(ttc, 2)})
                diagnostics.append(item)
                if clearance < required:
                    return False, f"front_gap_id_{int(obs.id)}", diagnostics
                if math.isfinite(ttc) and ttc < self.gap_min_ttc_s:
                    return False, f"front_ttc_id_{int(obs.id)}", diagnostics
            else:
                clearance = -rel_s - long_half - ego_rear
                required = max(
                    self.rear_min_gap_m,
                    self.gap_time_headway_s * max(obs_long_speed, 0.0),
                )
                closing = obs_long_speed - ego_speed
                ttc = clearance / closing if closing > 0.2 and clearance > 0.0 else float("inf")
                item.update({"position": "rear", "clearance": round(clearance, 2), "ttc": None if not math.isfinite(ttc) else round(ttc, 2)})
                diagnostics.append(item)
                if clearance < required:
                    return False, f"rear_gap_id_{int(obs.id)}", diagnostics
                if math.isfinite(ttc) and ttc < self.gap_min_ttc_s:
                    return False, f"rear_ttc_id_{int(obs.id)}", diagnostics

        # On the sensor team's highway mode, preserve their existing LEFT merge
        # gap result as an additional veto. Outside highway mode we do not use it
        # because that perception node is intentionally gated/idle there.
        if (
            side == "left"
            and self.use_existing_left_merge_gap_on_highway
            and self.highway_environment
            and self.merge_gap_at is not None
            and (now - self.merge_gap_at).to_sec() <= self.merge_topic_timeout_s
        ):
            if self.merge_unavailable or not self.merge_available:
                return False, "sensor_merge_gap_unavailable", diagnostics

        return True, "ok", diagnostics

    def _generate_lane_change_candidate(
        self,
        ego_projection,
        ego_d_slope: float,
        target_d: float,
        side: str,
        threat: ThreatInfo,
        departure_length_m: float,
    ) -> Optional[LaneChangeAvoidanceCandidate]:
        ego_s = float(ego_projection.s)
        ego_d = float(ego_projection.d)
        total = max(float(self.reference.total_length_m), 1.0e-6)

        # Work in an unwrapped forward-s coordinate so a maneuver can cross the
        # closed-loop route seam.  Convert back to reference s only at map lookup.
        threat_rel = self._signed_s_delta(float(threat.s), ego_s)
        if threat_rel < -1.0:
            return None
        threat_u = ego_s + threat_rel
        local_end_u = ego_s + self.local_length_m
        if not self.closed_loop:
            local_end_u = min(local_end_u, total)

        obstacle_front_u = threat_u - threat.longitudinal_half_m
        obstacle_rear_u = threat_u + threat.longitudinal_half_m
        hold_start_u = obstacle_front_u - self.lane_change_longitudinal_margin_m
        hold_end_u = obstacle_rear_u + self.lane_change_longitudinal_margin_m
        return_end_u = hold_end_u + self.lane_change_return_length_m
        if hold_end_u <= ego_s or return_end_u > local_end_u:
            return None

        transition_start_u = max(ego_s, hold_start_u - max(0.5, departure_length_m))
        if hold_start_u > transition_start_u + 1.0e-6:
            transition_end_u = hold_start_u
        else:
            remaining = max(0.0, hold_end_u - transition_start_u)
            if remaining < 0.2:
                return None
            transition_end_u = min(
                hold_end_u,
                transition_start_u + max(self.lane_change_min_transition_m, 0.5 * remaining),
            )
        transition_len = transition_end_u - transition_start_u
        if transition_len < 0.1 and abs(target_d - ego_d) > 0.05:
            return None

        def map_point(u: float, d: float) -> PathPoint:
            ref_s = (u % total) if self.closed_loop else max(0.0, min(total, u))
            return self.reference.frenet_to_map(ref_s, d)

        spacing = max(0.1, self.sample_spacing_m)
        path: List[PathPoint] = []
        u = ego_s
        while u <= local_end_u + 1.0e-6:
            if u < transition_start_u:
                d = ego_d
            elif u <= transition_end_u and transition_len > 1.0e-6:
                q = (u - transition_start_u) / transition_len
                d = _quintic_lateral_transition(
                    ego_d, target_d, ego_d_slope, transition_len, q
                )
            elif u <= hold_end_u:
                d = target_d
            elif u <= return_end_u:
                q = (u - hold_end_u) / max(return_end_u - hold_end_u, 1.0e-6)
                d = target_d * (1.0 - _quintic_smoothstep(q))
            else:
                d = 0.0
            _append_unique(path, map_point(u, d))
            u += spacing

        for exact_u, exact_d in (
            (ego_s, ego_d),
            (transition_start_u, ego_d),
            (transition_end_u, target_d),
            (hold_end_u, target_d),
            (return_end_u, 0.0),
        ):
            if ego_s <= exact_u <= local_end_u:
                _append_unique(path, map_point(exact_u, exact_d))

        path.sort(key=lambda p: self._signed_s_delta(self.reference.project(p.x, p.y).s, ego_s))
        deduped: List[PathPoint] = []
        for p in path:
            _append_unique(deduped, p)
        if len(deduped) < 3:
            return None
        return LaneChangeAvoidanceCandidate(
            kind="lane_change",
            side=side,
            target_d_m=float(target_d),
            change_length_m=float(transition_len),
            obstacle_id=int(threat.obstacle_id),
            hold_start_s=float(transition_end_u),
            hold_end_s=float(hold_end_u),
            return_end_s=float(return_end_u),
            path=deduped,
        )

    def _candidate_to_ros_path(self, candidate: Optional[LaneChangeAvoidanceCandidate]) -> RosPath:
        msg = RosPath()
        msg.header.frame_id = self.map_frame
        if candidate is None:
            return msg
        for p in candidate.path:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = float(p.x)
            ps.pose.position.y = float(p.y)
            ps.pose.position.z = float(p.z)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    @staticmethod
    def _points_from_ros_path(msg: RosPath) -> List[PathPoint]:
        return [
            PathPoint(
                float(ps.pose.position.x),
                float(ps.pose.position.y),
                float(ps.pose.position.z),
            )
            for ps in msg.poses
        ]

    def _base_bypass_road_legal(
        self,
        base_path: RosPath,
        lane_info: Optional[dict],
        current_center_d: Optional[float],
        intersection_mode: bool = False,
    ) -> Tuple[bool, str]:
        points = self._points_from_ros_path(base_path)
        if len(points) < 3:
            return False, "base_bypass_path_short"
        ds = [float(self.reference.project(p.x, p.y).d) for p in points]

        # Final competition policy: obstacle BYPASS is lane-agnostic.
        # Camera lane validity/solid/dashed boundaries never veto an F6 BYPASS.
        # The upstream F6 planner already enforces its configured max-|d| and
        # performs OBB collision, curvature, steering and lateral-acceleration
        # feasibility checks.
        if not self.enforce_camera_lane_bounds_on_bypass:
            return True, "lane_agnostic_bypass"

        # Optional legacy mode only: if camera bounds are explicitly re-enabled,
        # an intersection exception can still be used.  The final launch does not
        # enable this branch.
        if intersection_mode:
            max_abs_d = max(abs(x) for x in ds)
            if max_abs_d <= self.intersection_bypass_max_abs_d_m:
                return True, "intersection_global_d_bound"
            return False, "intersection_bypass_d_limit"
        if lane_info is None or current_center_d is None:
            if not self.allow_bypass_without_lane_info:
                return False, "lane_info_missing_bypass_disabled"
            if max(abs(x) for x in ds) <= self.fallback_bypass_max_abs_d_m:
                return True, "fallback_global_d_bound"
            return False, "lane_info_missing_for_cross_lane_bypass"

        width = float(lane_info.get("lane_width_m"))
        half_available = max(
            0.05,
            0.5 * width
            - 0.5 * self.vehicle_width_m
            - self.bypass_lane_boundary_margin_m,
        )
        low = current_center_d - half_available
        high = current_center_d + half_available
        if min(ds) < low or max(ds) > high:
            return False, "base_bypass_crosses_lane_boundary"
        return True, "inside_current_lane"

    def _publish_bundle(
        self,
        path_msg: RosPath,
        planner_ready: bool,
        avoidance_required: bool,
        safe_path_available: bool,
        selected_kind: str,
        selected_side: str,
        debug_payload: dict,
    ) -> None:
        self.plan_seq = (self.plan_seq + 1) & 0xFFFFFFFF
        if self.plan_seq == 0:
            self.plan_seq = 1
        seq = self.plan_seq
        stamp = rospy.Time.now()
        path_msg.header.seq = seq
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.map_frame
        for ps in path_msg.poses:
            ps.header.seq = seq
            ps.header.stamp = stamp
            ps.header.frame_id = self.map_frame
        self.selected_path_pub.publish(path_msg)

        status = {
            "seq": seq,
            "planner_ready": bool(planner_ready),
            "avoidance_required": bool(avoidance_required),
            "safe_path_available": bool(safe_path_available),
            "selected_kind": str(selected_kind or ""),
            "selected_side": str(selected_side or ""),
        }
        self.plan_status_pub.publish(String(data=json.dumps(status, separators=(",", ":"))))
        self.planner_ready_pub.publish(Bool(data=bool(planner_ready)))
        self.avoidance_required_pub.publish(Bool(data=bool(avoidance_required)))
        self.safe_path_available_pub.publish(Bool(data=bool(safe_path_available)))
        self.selected_kind_pub.publish(String(data=str(selected_kind or "")))
        self.selected_side_pub.publish(String(data=str(selected_side or "")))
        debug_payload["output"] = status
        self.status_pub.publish(
            String(data=json.dumps(debug_payload, ensure_ascii=False, separators=(",", ":")))
        )

    def _timer_cb(self, _event) -> None:
        now = rospy.Time.now()
        empty = RosPath()
        empty.header.frame_id = self.map_frame

        base_fresh = self._base_bundle_fresh(now)
        sensors_fresh = self._sensors_fresh(now)
        if not base_fresh or not sensors_fresh or self.latest_odom is None:
            self._publish_bundle(
                empty,
                False,
                bool(self.base_status.get("avoidance_required", False)),
                False,
                "",
                "",
                {"reason": "upstream_or_sensor_stale", "base_fresh": base_fresh, "sensors_fresh": sensors_fresh},
            )
            return

        base_ready = bool(self.base_status.get("planner_ready", False))
        avoidance_required = bool(self.base_status.get("avoidance_required", False))
        base_safe = bool(self.base_status.get("safe_path_available", False))
        base_kind = str(self.base_status.get("selected_kind", "") or "")
        base_side = str(self.base_status.get("selected_side", "") or "")

        pose = self.latest_odom.pose.pose
        ego_projection = self.reference.project(pose.position.x, pose.position.y)
        ego_yaw = _yaw_from_quaternion(pose.orientation)
        ref_yaw = math.atan2(ego_projection.tangent_y, ego_projection.tangent_x)
        ego_d_slope = max(-0.70, min(0.70, math.tan(_normalize_angle(ego_yaw - ref_yaw))))
        ego_speed = math.hypot(
            self.latest_odom.twist.twist.linear.x,
            self.latest_odom.twist.twist.linear.y,
        )

        lane_info, lane_reason = self._lane_snapshot(now)
        current_center_d = (
            self._camera_center_d(lane_info, ego_projection, pose)
            if lane_info is not None
            else None
        )
        if lane_info is not None and current_center_d is None:
            lane_reason = "camera_center_projection_failed"
            lane_info = None

        intersection_mode, intersection_source = self._intersection_bypass_active(now)

        debug: Dict[str, object] = {
            "base": {
                "planner_ready": base_ready,
                "avoidance_required": avoidance_required,
                "safe": base_safe,
                "kind": base_kind,
                "side": base_side,
            },
            "lane": {
                "valid_for_planning": lane_info is not None,
                "reason": lane_reason,
                "current_center_d": None if current_center_d is None else round(current_center_d, 3),
            },
            "intersection_bypass": {
                "active": intersection_mode,
                "source": intersection_source,
                "sensor_detected": self.intersection_detected,
                "mission_request": self.intersection_bypass_request,
                "max_abs_d_m": self.intersection_bypass_max_abs_d_m,
            },
        }

        if not avoidance_required:
            self._publish_bundle(empty, base_ready, False, False, "", "", debug)
            return

        threats = self._threatening_obstacles(ego_projection)
        threat = threats[0] if threats else None
        debug["threat"] = None if threat is None else {
            "id": threat.obstacle_id,
            "near_edge_m": round(threat.near_edge_m, 2),
            "d_m": round(threat.d, 2),
        }

        lane_evals: List[Tuple[LaneChangeAvoidanceCandidate, CandidateEvaluation, str, List[dict]]] = []
        if self.enable_obstacle_lane_change and lane_info is not None and current_center_d is not None and threat is not None:
            safety_boxes = self._safety_boxes()
            for side in ("left", "right"):
                allowed, boundary_reason = self._boundary_allows(lane_info, side)
                if not allowed:
                    debug.setdefault("lane_change_reject", {})[side] = boundary_reason
                    continue
                target_d, target_width, geometry_reason = self._adjacent_target_geometry(
                    lane_info, side, ego_projection, pose
                )
                if target_d is None or target_width is None:
                    debug.setdefault("lane_change_reject", {})[side] = geometry_reason
                    continue
                if abs(target_d) > self.max_abs_target_d_m:
                    debug.setdefault("lane_change_reject", {})[side] = "target_d_limit"
                    continue
                gap_ok, gap_reason, gap_diag = self._target_lane_gap_safe(
                    target_d,
                    target_width,
                    ego_projection,
                    ego_speed,
                    threat.obstacle_id,
                    side,
                    now,
                )
                if not gap_ok:
                    debug.setdefault("lane_change_reject", {})[side] = gap_reason
                    debug.setdefault("gap", {})[side] = gap_diag
                    continue
                for dep_len in self.lane_change_departure_lengths_m:
                    candidate = self._generate_lane_change_candidate(
                        ego_projection,
                        ego_d_slope,
                        target_d,
                        side,
                        threat,
                        dep_len,
                    )
                    if candidate is None:
                        continue
                    ev = evaluate_candidate(
                        candidate_index=len(lane_evals),
                        candidate=candidate,
                        obstacles=safety_boxes,
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
                    lane_evals.append((candidate, ev, gap_reason, gap_diag))

        valid_lane = [x for x in lane_evals if x[1].valid]
        selected_lane = min(valid_lane, key=lambda x: x[1].cost) if valid_lane else None
        if selected_lane is not None:
            candidate, ev, _gap_reason, gap_diag = selected_lane
            lane_path = self._candidate_to_ros_path(candidate)
            self.lane_change_path_pub.publish(lane_path)
            debug["selected_lane_change"] = {
                "side": candidate.side,
                "target_d_m": round(candidate.target_d_m, 3),
                "cost": round(ev.cost, 4),
                "max_curvature_1pm": round(ev.max_curvature_1pm, 5),
                "max_steering_rad": round(ev.max_steering_rad, 4),
                "gap_objects": gap_diag,
            }
            if self.prefer_lane_change:
                self._publish_bundle(
                    lane_path,
                    base_ready,
                    True,
                    True,
                    "lane_change",
                    candidate.side,
                    debug,
                )
                return

        # Fallback to the base planner's BYPASS only. Old MGeo lane_change output
        # is deliberately never forwarded by this final supervisor.
        bypass_ok = False
        bypass_reason = "base_bypass_unavailable"
        if base_safe and base_kind == "bypass" and self.base_path is not None:
            bypass_ok, bypass_reason = self._base_bypass_road_legal(
                self.base_path, lane_info, current_center_d, intersection_mode
            )
        debug["base_bypass_road_check"] = {"ok": bypass_ok, "reason": bypass_reason}

        if bypass_ok and self.base_path is not None:
            self._publish_bundle(
                self.base_path,
                base_ready,
                True,
                True,
                "bypass",
                base_side,
                debug,
            )
            return

        if self.enable_obstacle_lane_change and selected_lane is not None:
            candidate, ev, _gap_reason, gap_diag = selected_lane
            lane_path = self._candidate_to_ros_path(candidate)
            self.lane_change_path_pub.publish(lane_path)
            self._publish_bundle(
                lane_path,
                base_ready,
                True,
                True,
                "lane_change",
                candidate.side,
                debug,
            )
            return

        self._publish_bundle(empty, base_ready, True, False, "", "", debug)
        rospy.logwarn_throttle(
            1.0,
            "No legal safe maneuver: lane=%s base_kind=%s bypass=%s",
            lane_reason,
            base_kind or "-",
            bypass_reason,
        )


def main() -> None:
    try:
        BypassLaneGuard()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
