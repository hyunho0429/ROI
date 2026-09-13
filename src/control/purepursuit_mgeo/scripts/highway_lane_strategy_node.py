#!/usr/bin/env python3
"""Highway-only proactive lane-change + lane-hold + longitudinal safety manager.

Camera-team files are treated as read-only inputs. This node consumes:
  /perception/camera/highway_environment
  /perception/camera/lane_info
  /perception/merge_gap/available
  /perception/merge_gap/unavailable
and LiDAR tracked obstacles / odometry.

Outside the highway scenario, it simply republishes the existing avoidance
PathManager path/stop and the cruise speed. During the highway scenario it:
  1) waits for a safe LEFT merge gap while staying on the base path;
  2) generates a committed shallow diagonal shift with eased entry/exit;
  3) keeps following the camera-reported current lane centerline after the
     change, so the vehicle does NOT get pulled back to the original outer
     global path;
  4) adapts target speed to the lead vehicle and uses front/rear vehicle speed
     when selecting a lane-change speed;
  5) returns to the base/global path only after the physical lanes converge
     and the ego vehicle is again close to the global path.

It never publishes /ctrl_cmd.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool, Float64, String

from lidar_perception.msg import LidarObstacleArray
from purepursuit_mgeo.path import PathPoint, load_mgeo_path
from purepursuit_mgeo.motion import diagonal_progress


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def smoothstep5(u: float) -> float:
    u = clamp(float(u), 0.0, 1.0)
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def polyline_arclength(points: Sequence[Tuple[float, float]]) -> List[float]:
    out = [0.0]
    for i in range(1, len(points)):
        out.append(out[-1] + math.hypot(points[i][0] - points[i-1][0], points[i][1] - points[i-1][1]))
    return out


def tangent_at(points: Sequence[Tuple[float, float]], i: int) -> Tuple[float, float]:
    if len(points) < 2:
        return 1.0, 0.0
    if i <= 0:
        dx = points[1][0] - points[0][0]
        dy = points[1][1] - points[0][1]
    elif i >= len(points) - 1:
        dx = points[-1][0] - points[-2][0]
        dy = points[-1][1] - points[-2][1]
    else:
        dx = points[i+1][0] - points[i-1][0]
        dy = points[i+1][1] - points[i-1][1]
    n = math.hypot(dx, dy)
    if n < 1e-6:
        return 1.0, 0.0
    return dx / n, dy / n


def interp_y(points: Sequence[Tuple[float, float]], x: float) -> Optional[float]:
    if not points:
        return None
    pts = sorted(points, key=lambda p: p[0])
    if x <= pts[0][0]:
        return pts[0][1]
    if x >= pts[-1][0]:
        return pts[-1][1]
    for i in range(1, len(pts)):
        x0, y0 = pts[i-1]
        x1, y1 = pts[i]
        if x0 <= x <= x1 and x1 > x0 + 1e-6:
            u = (x - x0) / (x1 - x0)
            return y0 + u * (y1 - y0)
    return None


@dataclass
class LocalObstacle:
    oid: int
    x: float
    y: float
    vx: float
    vy: float
    length: float
    width: float
    yaw: float


class HighwayLaneStrategyNode:
    OFF = "OFF"
    WAIT_GAP = "WAIT_GAP"
    LANE_CHANGE = "LANE_CHANGE"
    INNER_HOLD = "INNER_HOLD"
    REJOIN = "REJOIN"
    DONE = "DONE"

    def __init__(self) -> None:
        rospy.init_node("highway_lane_strategy", anonymous=False)

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.path_file = rospy.get_param("~path_file")
        self.global_points = load_mgeo_path(self.path_file)

        self.base_path_topic = rospy.get_param("~base_path_topic", "/avoidance_path_manager/active_path")
        self.base_stop_topic = rospy.get_param("~base_stop_topic", "/avoidance_path_manager/stop_required")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odometry")
        self.obstacle_topic = rospy.get_param("~obstacle_topic", "/perception/lidar/tracked_obstacles_map")
        self.lane_info_topic = rospy.get_param("~lane_info_topic", "/perception/camera/lane_info")
        self.highway_topic = rospy.get_param("~highway_topic", "/perception/camera/highway_environment")
        self.highway_request_topic = rospy.get_param("~highway_request_topic", "/planning/highway_lane_change_request")
        self.merge_available_topic = rospy.get_param("~merge_available_topic", "/perception/merge_gap/available")
        self.merge_unavailable_topic = rospy.get_param("~merge_unavailable_topic", "/perception/merge_gap/unavailable")

        self.cruise_speed_mps = float(rospy.get_param("~cruise_speed_mps", 6.0))
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.target_left_lane_changes = max(1, min(2, int(rospy.get_param("~target_left_lane_changes", 2))))
        self.min_lane_hold_before_next_change_m = float(rospy.get_param("~min_lane_hold_before_next_change_m", 8.0))
        self.highway_confirm_s = float(rospy.get_param("~highway_confirm_s", 0.5))
        self.ready_confirm_s = float(rospy.get_param("~ready_confirm_s", 0.5))
        self.lane_info_timeout_s = float(rospy.get_param("~lane_info_timeout_s", 0.6))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.5))
        self.obstacle_timeout_s = float(rospy.get_param("~obstacle_timeout_s", 0.6))
        self.base_timeout_s = float(rospy.get_param("~base_timeout_s", 0.8))
        self.merge_timeout_s = float(rospy.get_param("~merge_timeout_s", 0.8))
        self.mission_request_bypass_sensor_merge_gate = bool(
            rospy.get_param("~mission_request_bypass_sensor_merge_gate", True)
        )

        self.min_lane_confidence = float(rospy.get_param("~min_lane_confidence", 0.45))
        self.lane_width_min_m = float(rospy.get_param("~lane_width_min_m", 2.7))
        self.lane_width_max_m = float(rospy.get_param("~lane_width_max_m", 4.2))
        self.nominal_lane_width_m = float(rospy.get_param("~nominal_lane_width_m", 3.5))
        self.max_heading_error_rad = float(rospy.get_param("~max_heading_error_rad", math.radians(22.0)))
        self.require_left_dashed = bool(rospy.get_param("~require_left_dashed", True))

        # Geometry sanity: the detected LEFT dashed divider must really be the
        # divider immediately next to the ego lane. The lane-info publisher can
        # fall back to a one-sided lane estimate; blindly shifting that estimated
        # centerline by one full lane width can command an accidental two-lane
        # jump. Highway lane-change geometry therefore uses the LEFT divider
        # itself and rejects a divider that is not adjacent to the ego lane.
        self.left_divider_expected_tol_m = float(
            rospy.get_param("~left_divider_expected_tol_m", 0.90)
        )
        self.inner_center_max_abs_y_m = float(
            rospy.get_param("~inner_center_max_abs_y_m", 0.90)
        )
        self.inner_lane_invalid_grace_s = float(
            rospy.get_param("~inner_lane_invalid_grace_s", 1.20)
        )
        self.inner_handover_confirm_s = float(
            rospy.get_param("~inner_handover_confirm_s", 0.30)
        )
        self.inner_lane_grace_speed_mps = float(
            rospy.get_param("~inner_lane_grace_speed_mps", 1.50)
        )
        self.inner_fallback_path_length_m = float(
            rospy.get_param("~inner_fallback_path_length_m", 60.0)
        )

        self.vehicle_length_m = float(rospy.get_param("~vehicle_length_m", 4.635))
        self.vehicle_width_m = float(rospy.get_param("~vehicle_width_m", 1.892))
        self.vehicle_center_from_base_m = float(rospy.get_param("~vehicle_center_from_base_m", 1.50))

        self.change_start_m = float(rospy.get_param("~change_start_m", 3.0))
        self.change_min_length_m = float(rospy.get_param("~change_min_length_m", 32.0))
        self.change_max_length_m = float(rospy.get_param("~change_max_length_m", 40.0))
        self.change_ramp_ratio = float(rospy.get_param("~change_ramp_ratio", 0.2))
        self.change_max_heading_rad = math.radians(float(rospy.get_param("~change_max_heading_deg", 8.0)))
        if not 0.0 < self.change_ramp_ratio < 0.5 or not 0.0 < self.change_max_heading_rad < math.pi/4:
            raise ValueError("invalid diagonal lane-change ramp or heading limit")
        self.inner_path_blend_time_s = max(0.05, float(rospy.get_param("~inner_path_blend_time_s", 0.35)))
        self.inner_path_max_jump_m = float(rospy.get_param("~inner_path_max_jump_m", 0.75))
        self.inner_path_join_length_m = float(rospy.get_param("~inner_path_join_length_m", 8.0))
        self.change_time_s = float(rospy.get_param("~change_time_s", 5.5))
        self.change_post_hold_m = float(rospy.get_param("~change_post_hold_m", 10.0))
        self.change_complete_min_ratio = float(rospy.get_param("~change_complete_min_ratio", 0.72))
        self.change_center_error_m = float(rospy.get_param("~change_center_error_m", 0.45))
        self.change_heading_error_rad = float(rospy.get_param("~change_heading_error_rad", math.radians(10.0)))
        self.change_complete_confirm_s = float(rospy.get_param("~change_complete_confirm_s", 0.45))
        # Never let the finite committed lane-change path reach Pure Pursuit's
        # ordinary goal-stop condition.  Once the lateral transition is done and
        # only a few metres of post-hold remain, hand over to the receding-horizon
        # INNER_HOLD path.
        self.change_endpoint_guard_m = float(rospy.get_param("~change_endpoint_guard_m", 6.0))

        self.front_min_gap_m = float(rospy.get_param("~front_min_gap_m", 6.0))
        self.rear_min_gap_m = float(rospy.get_param("~rear_min_gap_m", 7.0))
        self.time_headway_s = float(rospy.get_param("~time_headway_s", 1.5))
        self.min_ttc_s = float(rospy.get_param("~min_ttc_s", 3.0))
        self.gap_search_range_m = float(rospy.get_param("~gap_search_range_m", 50.0))
        # Lane-membership gates use obstacle CENTER distance from each lane center.
        # The old footprint-style gate was ~3 m wide and could classify an
        # adjacent-lane car as the current-lane lead, which caused false stops.
        self.current_lane_center_gate_m = float(rospy.get_param("~current_lane_center_gate_m", 1.15))
        self.target_lane_center_gate_m = float(rospy.get_param("~target_lane_center_gate_m", 1.50))
        self.target_lane_lateral_extra_m = float(rospy.get_param("~target_lane_lateral_extra_m", 0.35))

        self.follow_standstill_gap_m = float(rospy.get_param("~follow_standstill_gap_m", 4.0))
        self.follow_time_headway_s = float(rospy.get_param("~follow_time_headway_s", 1.6))
        self.follow_gain = float(rospy.get_param("~follow_gain", 0.35))
        self.follow_search_m = float(rospy.get_param("~follow_search_m", 60.0))
        self.emergency_gap_m = float(rospy.get_param("~emergency_gap_m", 1.5))
        self.emergency_ttc_s = float(rospy.get_param("~emergency_ttc_s", 1.0))
        self.change_settle_m = float(rospy.get_param("~change_settle_m", 6.0))
        self.speed_rise_mps2 = float(rospy.get_param("~speed_rise_mps2", 0.8))
        self.speed_fall_mps2 = float(rospy.get_param("~speed_fall_mps2", 1.8))

        self.collision_long_margin_m = float(rospy.get_param("~collision_long_margin_m", 0.5))
        self.collision_lat_margin_m = float(rospy.get_param("~collision_lat_margin_m", 0.35))
        self.dynamic_prediction_horizon_s = float(
            rospy.get_param("~dynamic_prediction_horizon_s", 5.0)
        )
        self.max_lateral_accel_mps2 = float(rospy.get_param("~max_lateral_accel_mps2", 2.5))

        self.release_global_d_m = float(rospy.get_param("~release_global_d_m", 0.55))
        self.release_confirm_s = float(rospy.get_param("~release_confirm_s", 0.6))
        self.min_inner_hold_after_change_m = float(rospy.get_param("~min_inner_hold_after_change_m", 8.0))
        self.rejoin_start_global_d_m = float(rospy.get_param("~rejoin_start_global_d_m", 1.8))
        self.rejoin_length_m = float(rospy.get_param("~rejoin_length_m", 18.0))
        self.rejoin_complete_global_d_m = float(rospy.get_param("~rejoin_complete_global_d_m", 0.45))

        self.latest_base_path: Optional[RosPath] = None
        self.base_path_at: Optional[rospy.Time] = None
        self.base_stop = True
        self.base_stop_at: Optional[rospy.Time] = None
        self.latest_odom: Optional[Odometry] = None
        self.odom_at: Optional[rospy.Time] = None
        self.latest_obstacles: Optional[LidarObstacleArray] = None
        self.obstacles_at: Optional[rospy.Time] = None
        self.lane_info = None
        self.lane_info_at: Optional[rospy.Time] = None
        self.highway_environment = False
        self.highway_at: Optional[rospy.Time] = None
        self.highway_request = False
        self.merge_available = False
        self.merge_unavailable = True
        self.merge_at: Optional[rospy.Time] = None

        self.state = self.OFF
        self.highway_true_since: Optional[rospy.Time] = None
        self.ready_since: Optional[rospy.Time] = None
        self.complete_since: Optional[rospy.Time] = None
        self.release_since: Optional[rospy.Time] = None
        self.lane_changes_done = 0
        self.completed_once = False

        self.committed_path: Optional[RosPath] = None
        self.committed_speed_mps = self.cruise_speed_mps
        self.committed_change_length_m = self.change_min_length_m
        self.committed_rejoin_path: Optional[RosPath] = None
        self.rejoin_travel_m = 0.0
        self.last_rejoin_xy: Optional[Tuple[float, float]] = None
        self.change_travel_m = 0.0
        self.last_change_xy: Optional[Tuple[float, float]] = None
        self.inner_hold_travel_m = 0.0
        self.last_hold_xy: Optional[Tuple[float, float]] = None

        self.last_output_speed = self.cruise_speed_mps
        self.last_timer_time: Optional[rospy.Time] = None
        self.last_inner_path: Optional[RosPath] = None
        self.lane_invalid_since: Optional[rospy.Time] = None
        self.inner_handover_pending = False
        self.inner_lane_candidate_since: Optional[rospy.Time] = None
        self.inner_lane_candidate_signature = None

        self.path_pub = rospy.Publisher("~active_path", RosPath, queue_size=1)
        self.stop_pub = rospy.Publisher("~stop_required", Bool, queue_size=1)
        self.speed_pub = rospy.Publisher("~target_speed_mps", Float64, queue_size=1)
        self.active_pub = rospy.Publisher("~active", Bool, queue_size=1)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1)

        rospy.Subscriber(self.base_path_topic, RosPath, self._base_path_cb, queue_size=1)
        rospy.Subscriber(self.base_stop_topic, Bool, self._base_stop_cb, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber(self.obstacle_topic, LidarObstacleArray, self._obstacles_cb, queue_size=1)
        rospy.Subscriber(self.lane_info_topic, String, self._lane_info_cb, queue_size=1)
        rospy.Subscriber(self.highway_topic, Bool, self._highway_cb, queue_size=1)
        rospy.Subscriber(self.highway_request_topic, Bool, self._highway_request_cb, queue_size=1)
        rospy.Subscriber(self.merge_available_topic, Bool, self._merge_available_cb, queue_size=1)
        rospy.Subscriber(self.merge_unavailable_topic, Bool, self._merge_unavailable_cb, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self._tick)
        rospy.logwarn(
            "Highway lane strategy: LEFT lane changes=%d cruise=%.2f m/s; real-lane centerline enabled",
            self.target_left_lane_changes, self.cruise_speed_mps,
        )

    def _base_path_cb(self, msg: RosPath) -> None:
        self.latest_base_path = msg
        self.base_path_at = rospy.Time.now()

    def _base_stop_cb(self, msg: Bool) -> None:
        self.base_stop = bool(msg.data)
        self.base_stop_at = rospy.Time.now()

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.odom_at = rospy.Time.now()

    def _obstacles_cb(self, msg: LidarObstacleArray) -> None:
        self.latest_obstacles = msg
        self.obstacles_at = rospy.Time.now()

    def _lane_info_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.lane_info = data
                self.lane_info_at = rospy.Time.now()
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "lane_info JSON parse failed: %s", exc)

    def _highway_cb(self, msg: Bool) -> None:
        self.highway_environment = bool(msg.data)
        self.highway_at = rospy.Time.now()

    def _highway_request_cb(self, msg: Bool) -> None:
        self.highway_request = bool(msg.data)

    def _merge_available_cb(self, msg: Bool) -> None:
        self.merge_available = bool(msg.data)
        self.merge_at = rospy.Time.now()

    def _merge_unavailable_cb(self, msg: Bool) -> None:
        self.merge_unavailable = bool(msg.data)
        self.merge_at = rospy.Time.now()

    def _fresh(self, stamp: Optional[rospy.Time], timeout: float, now: rospy.Time) -> bool:
        return stamp is not None and (now - stamp).to_sec() <= timeout

    def _activation_present(self) -> bool:
        return bool(self.highway_environment or self.highway_request)

    def _lane_valid(self, now: rospy.Time, require_measured_width: bool = False) -> Tuple[bool, str]:
        if self.lane_info is None or not self._fresh(self.lane_info_at, self.lane_info_timeout_s, now):
            return False, "lane_info_missing_or_stale"
        d = self.lane_info
        if not bool(d.get("lane_valid", False)):
            return False, "lane_invalid"
        straddling = d.get("straddling_lane") or {}
        if bool(straddling.get("detected", False)):
            return False, "lane_straddling"
        confidence_value = d.get("confidence")
        if confidence_value is None:
            boundary_confidences = [
                float(lane.get("confidence", 0.0) or 0.0)
                for lane in (d.get("left_lane") or {}, d.get("right_lane") or {})
                if bool(lane.get("detected", False))
            ]
            confidence_value = min(boundary_confidences) if boundary_confidences else 0.0
        if float(confidence_value or 0.0) < self.min_lane_confidence:
            return False, "lane_confidence"
        width = d.get("lane_width_m")
        if require_measured_width and width is None:
            return False, "lane_width"
        if width is not None and not (self.lane_width_min_m <= float(width) <= self.lane_width_max_m):
            return False, "lane_width"
        pts = self._centerline_local()
        if len(pts) < 3:
            return False, "centerline_short"
        heading = d.get("heading_error_rad")
        if heading is None:
            y_near = interp_y(pts, 5.0)
            y_far = interp_y(pts, 12.0)
            if y_near is None or y_far is None:
                return False, "lane_heading"
            heading = math.atan2(float(y_far)-float(y_near), 7.0)
        if abs(float(heading)) > self.max_heading_error_rad:
            return False, "lane_heading"
        return True, "ok"

    def _left_dashed_ok(self) -> Tuple[bool, str]:
        if not self.require_left_dashed:
            return True, "disabled"
        left = (self.lane_info or {}).get("left_lane") or {}
        if not bool(left.get("detected", False)):
            return False, "left_not_detected"
        if bool(left.get("from_guide", False)):
            return False, "left_from_guide"
        if bool(left.get("coasted", False)):
            return False, "left_coasted"
        if left.get("dashed") is True:
            return True, "ok"
        return False, "left_not_dashed"

    def _centerline_local(self) -> List[Tuple[float, float]]:
        straddling = (self.lane_info or {}).get("straddling_lane") or {}
        if bool(straddling.get("detected", False)):
            return [(0.0, 0.0)]

        raw = (self.lane_info or {}).get("centerline_points") or []
        reported: List[Tuple[float, float]] = [(0.0, 0.0)]
        for p in raw:
            try:
                x, y = float(p[0]), float(p[1])
            except Exception:
                continue
            if math.isfinite(x) and math.isfinite(y) and x > 0.5:
                reported.append((x, y))
        reported.sort(key=lambda q: q[0])

        # Use the real-lane pipeline center as the primary lane-hold path after
        # checking it against the two physical boundaries.  If the reported
        # center is inconsistent, fall back to a direct boundary midpoint.
        left = self._boundary_local("left_boundary_points")
        right = self._boundary_local("right_boundary_points")
        midpoint: List[Tuple[float, float]] = [(0.0, 0.0)]
        if len(left) >= 3 and len(right) >= 3:
            x_start = max(0.5, left[0][0], right[0][0])
            x_end = min(left[-1][0], right[-1][0])
            reported_width = (self.lane_info or {}).get("lane_width_m")
            expected_width = float(reported_width) if reported_width is not None else None
            x = math.ceil(x_start)
            while x <= x_end + 1e-6:
                ly = interp_y(left, x)
                ry = interp_y(right, x)
                if ly is not None and ry is not None:
                    width = float(ly) - float(ry)
                    width_ok = self.lane_width_min_m <= width <= self.lane_width_max_m
                    agrees = expected_width is None or abs(width-expected_width) <= 0.8
                    if width_ok and agrees:
                        midpoint.append((float(x), 0.5*(float(ly)+float(ry))))
                x += 1.0
            if len(midpoint) >= 4:
                center_errors = []
                for x, y in reported[1:]:
                    midpoint_y = interp_y(midpoint, x)
                    if midpoint_y is not None:
                        center_errors.append(abs(y-midpoint_y))
                if len(reported) >= 4 and len(center_errors) >= 3 and max(center_errors) <= 0.35:
                    return reported
                return midpoint

        if len(reported) >= 4:
            return reported

        # The six-class pipeline reports lane_valid for a stable single physical
        # boundary.  It cannot measure width then, but lane hold can still use a
        # nominal-width center.  Lane-change authorization below continues to
        # require a measured width and a fresh dashed divider.
        one_side_width = float((self.lane_info or {}).get("lane_width_m")
                               or self.nominal_lane_width_m)
        left_meta = (self.lane_info or {}).get("left_lane") or {}
        right_meta = (self.lane_info or {}).get("right_lane") or {}
        if len(left) >= 3 and not bool(left_meta.get("from_guide", False)):
            return [(0.0, 0.0)] + [(x, y-0.5*one_side_width) for x, y in left if x > 0.5]
        if len(right) >= 3 and not bool(right_meta.get("from_guide", False)):
            return [(0.0, 0.0)] + [(x, y+0.5*one_side_width) for x, y in right if x > 0.5]

        # Remove near-duplicates.
        out: List[Tuple[float, float]] = []
        for p in reported:
            if not out or math.hypot(p[0]-out[-1][0], p[1]-out[-1][1]) > 0.15:
                out.append(p)
        return out

    def _lane_signature(self):
        info = self.lane_info or {}
        left = info.get("left_lane") or {}
        right = info.get("right_lane") or {}
        return (
            info.get("lane_state"),
            left.get("track_id") if bool(left.get("detected", False)) else None,
            right.get("track_id") if bool(right.get("detected", False)) else None,
        )


    def _boundary_local(self, key: str) -> List[Tuple[float, float]]:
        """Return a lane boundary in base_link and extrapolate it back to x=0."""
        raw = (self.lane_info or {}).get(key) or []
        pts: List[Tuple[float, float]] = []
        for q in raw:
            try:
                x, y = float(q[0]), float(q[1])
            except Exception:
                continue
            if math.isfinite(x) and math.isfinite(y) and x > 0.2:
                pts.append((x, y))
        pts.sort(key=lambda q: q[0])
        out: List[Tuple[float, float]] = []
        for q in pts:
            if not out or math.hypot(q[0]-out[-1][0], q[1]-out[-1][1]) > 0.15:
                out.append(q)
        if len(out) >= 2 and out[0][0] > 0.5:
            x0, y0 = out[0]
            x1, y1 = out[1]
            dx = x1 - x0
            slope = (y1-y0)/dx if abs(dx) > 1e-6 else 0.0
            out.insert(0, (0.0, y0 - slope*x0))
        return out

    def _left_divider_sanity(self, lane_width: float) -> Tuple[bool, str, dict]:
        divider = self._boundary_local("left_boundary_points")
        if len(divider) < 3:
            return False, "left_divider_short", {"n": len(divider)}
        y5 = interp_y(divider, 5.0)
        if y5 is None:
            y5 = divider[min(1, len(divider)-1)][1]
        expected = 0.5*lane_width
        err = abs(float(y5) - expected)
        diag = {
            "left_divider_y5_m": round(float(y5), 3),
            "expected_half_width_m": round(expected, 3),
            "divider_error_m": round(err, 3),
        }
        if float(y5) <= 0.0:
            return False, "left_divider_wrong_side", diag
        if err > self.left_divider_expected_tol_m:
            return False, "left_divider_not_adjacent", diag
        return True, "ok", diag

    def _inner_center_sanity(self) -> Tuple[bool, str, Optional[float]]:
        center = self._centerline_local()
        if len(center) < 3:
            return False, "inner_center_short", None
        y = interp_y(center, 8.0)
        if y is None:
            y = interp_y(center, 5.0)
        if y is None:
            return False, "inner_center_missing", None
        if abs(float(y)) > self.inner_center_max_abs_y_m:
            return False, "inner_center_not_ego_lane", float(y)
        return True, "ok", float(y)

    def _odom_pose(self) -> Tuple[float, float, float, float]:
        odom = self.latest_odom
        pose = odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        speed = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
        return float(pose.position.x), float(pose.position.y), float(yaw), float(speed)

    def _local_to_map(self, pts: Sequence[Tuple[float, float]], stamp: rospy.Time) -> RosPath:
        ex, ey, yaw, _ = self._odom_pose()
        c, s = math.cos(yaw), math.sin(yaw)
        msg = RosPath()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        for x, y in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = ex + c*x - s*y
            ps.pose.position.y = ey + s*x + c*y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def _extend_local_polyline(self, points: Sequence[Tuple[float, float]], target_length_m: float) -> List[Tuple[float, float]]:
        pts = list(points)
        if len(pts) < 2:
            return pts
        arc = polyline_arclength(pts)
        if arc[-1] >= target_length_m:
            return pts
        tx, ty = tangent_at(pts, len(pts)-1)
        x, y = pts[-1]
        remain = target_length_m - arc[-1]
        step = 1.0
        while remain > 1e-6:
            ds = min(step, remain)
            x += tx * ds
            y += ty * ds
            pts.append((x, y))
            remain -= ds
        return pts

    def _path_map_to_local(self, path: Optional[RosPath]) -> List[Tuple[float, float]]:
        if path is None or self.latest_odom is None:
            return []
        ex, ey, yaw, _ = self._odom_pose()
        c, s = math.cos(yaw), math.sin(yaw)
        out: List[Tuple[float, float]] = [(0.0, 0.0)]
        for ps in path.poses:
            dx = float(ps.pose.position.x) - ex
            dy = float(ps.pose.position.y) - ey
            x = c*dx + s*dy
            y = -s*dx + c*dy
            if x > 0.5 and math.isfinite(x) and math.isfinite(y):
                out.append((x, y))
        out.sort(key=lambda q: q[0])
        dedup: List[Tuple[float, float]] = []
        for q in out:
            if not dedup or math.hypot(q[0]-dedup[-1][0], q[1]-dedup[-1][1]) > 0.15:
                dedup.append(q)
        return dedup

    def _generate_rejoin_path(self, now: rospy.Time) -> Optional[RosPath]:
        """Blend the camera lane centerline into the base/global path smoothly.

        Rejoin starts only when the two physical lanes are already close, so a
        local-y quintic blend is enough and avoids a hard path-source switch.
        """
        inner = self._centerline_local()
        base = self._path_map_to_local(self.latest_base_path)
        if len(inner) < 3 or len(base) < 3:
            return None
        target_len = max(self.rejoin_length_m + 10.0, 30.0)
        inner = self._extend_local_polyline(inner, target_len)
        base = self._extend_local_polyline(base, target_len)
        arc = polyline_arclength(inner)
        blended: List[Tuple[float, float]] = []
        for i, (x, yi) in enumerate(inner):
            yb = interp_y(base, x)
            if yb is None:
                yb = yi
            u = (arc[i] - 1.5) / max(self.rejoin_length_m, 1e-6)
            w = smoothstep5(u)
            blended.append((x, (1.0-w)*yi + w*yb))
        return self._local_to_map(blended, now)

    def _generate_lane_change_local(self, lane_width: float, speed_mps: float) -> Tuple[List[Tuple[float, float]], float]:
        """One shallow diagonal lane move with eased entry and exit."""
        divider = self._boundary_local("left_boundary_points")
        if len(divider) < 3:
            return [], self.change_min_length_m
        tx, ty = tangent_at(divider, 0)
        # Include ego's initial offset from the detected source-lane center.
        shift_m = max(lane_width, math.hypot(
            divider[0][0]-0.5*lane_width*ty, divider[0][1]+0.5*lane_width*tx))
        heading_length = shift_m / ((1.0-self.change_ramp_ratio) * math.tan(self.change_max_heading_rad))
        if heading_length > self.change_max_length_m:
            return [], heading_length
        length = clamp(
            max(max(speed_mps, 1.0) * self.change_time_s, heading_length),
            self.change_min_length_m,
            self.change_max_length_m,
        )
        target_len = self.change_start_m + length + self.change_post_hold_m
        divider = self._extend_local_polyline(divider, target_len)
        if len(divider) < 3:
            return [], length
        # Camera samples can start five metres ahead. Uniform sampling prevents
        # that first long segment from skipping the eased steering entry.
        divider_arc = polyline_arclength(divider)
        sampled = []
        j = 1
        for k in range(int(divider_arc[-1]/0.5)+1):
            distance = k*0.5
            while j < len(divider)-1 and divider_arc[j] < distance:
                j += 1
            fraction = (distance-divider_arc[j-1])/max(divider_arc[j]-divider_arc[j-1], 1e-6)
            a, b = divider[j-1], divider[j]
            sampled.append((a[0]+fraction*(b[0]-a[0]), a[1]+fraction*(b[1]-a[1])))
        divider = sampled

        current: List[Tuple[float, float]] = []
        target: List[Tuple[float, float]] = []
        for i, (x, y) in enumerate(divider):
            tx, ty = tangent_at(divider, i)
            nx, ny = -ty, tx
            current.append((x - 0.5*lane_width*nx, y - 0.5*lane_width*ny))
            target.append((x + 0.5*lane_width*nx, y + 0.5*lane_width*ny))

        origin_x, origin_y = current[0]
        arc = polyline_arclength(current)

        shifted: List[Tuple[float, float]] = []
        for i, (cx, cy) in enumerate(current):
            u = (arc[i] - self.change_start_m) / max(length, 1e-6)
            w = diagonal_progress(u, self.change_ramp_ratio)
            tx, ty = target[i]
            shifted.append(((1.0-w)*(cx-origin_x) + w*tx, (1.0-w)*(cy-origin_y) + w*ty))
        return shifted, length

    def _committed_alignment(self) -> Tuple[bool, dict]:
        """Check actual pose against the final lane, not odometry distance alone."""
        if self.committed_path is None or len(self.committed_path.poses) < 3:
            return False, {"reason": "committed_path_missing"}
        points = [(p.pose.position.x, p.pose.position.y) for p in self.committed_path.poses]
        arc = polyline_arclength(points)
        ex, ey, yaw, _ = self._odom_pose()
        best = None
        for i in range(len(points)-1):
            ax, ay = points[i]
            bx, by = points[i+1]
            dx, dy = bx-ax, by-ay
            length2 = dx*dx+dy*dy
            if length2 < 1e-9:
                continue
            u = clamp(((ex-ax)*dx+(ey-ay)*dy)/length2, 0.0, 1.0)
            error = math.hypot(ex-ax-u*dx, ey-ay-u*dy)
            heading = math.atan2(math.sin(yaw-math.atan2(dy, dx)), math.cos(yaw-math.atan2(dy, dx)))
            if best is None or error < best[0]:
                best = (error, abs(heading), arc[i]+u*math.sqrt(length2))
        if best is None:
            return False, {"reason": "committed_path_degenerate"}
        error, heading, progress = best
        aligned = (error <= self.change_center_error_m
                   and heading <= self.change_heading_error_rad
                   and progress >= self.change_start_m+self.committed_change_length_m)
        return aligned, {"path_error_m": round(error, 3), "heading_error_rad": round(heading, 4), "path_progress_m": round(progress, 2)}

    def _filtered_inner_path(self, now: rospy.Time, dt: float) -> Tuple[Optional[RosPath], str]:
        """Blend camera updates with the previous path in a common map frame.

        Corrections are spread over both distance and time. A large but valid
        camera correction is rate-limited instead of rejecting the path and
        eventually stopping at the finite committed-path endpoint.
        """
        camera = self._extend_local_polyline(self._centerline_local(), 40.0)
        previous = self._path_map_to_local(self.last_inner_path)
        if len(camera) < 3:
            return None, "inner_camera_path_short"
        if len(previous) < 2:
            # The committed path can have no remaining samples if hand-over is
            # delayed near its endpoint. Continue from the current heading.
            previous = [(0.0, 0.0), (1.0, 0.0)]
        previous = self._extend_local_polyline(previous, 40.0)
        alpha = 1.0-math.exp(-max(0.0, dt)/self.inner_path_blend_time_s)
        limited = False
        blended = []
        for x, camera_y in camera:
            previous_y = interp_y(previous, x)
            delta = camera_y-previous_y
            bounded_delta = clamp(delta, -self.inner_path_max_jump_m, self.inner_path_max_jump_m)
            limited = limited or abs(delta) > self.inner_path_max_jump_m
            # Start the correction just ahead of the bumper and complete it in
            # the normal Pure Pursuit look-ahead range.  The old 18 m join made
            # the effective time constant around x=5 m tens of seconds, so an
            # offset at lane-change completion was effectively preserved.
            spatial = smoothstep5((x-0.5)/max(self.inner_path_join_length_m, 1.0))
            blended.append((x, previous_y+alpha*spatial*bounded_delta))
        return self._local_to_map(blended, now), "limited" if limited else "ok"

    def _rolling_inner_fallback(self, now: rospy.Time) -> Optional[RosPath]:
        """Keep a receding path after the camera briefly loses the new lane.

        The committed lane-change path has only a few metres left when the
        completion hand-over occurs.  Re-expressing it from the current pose
        and extending its final tangent prevents Pure Pursuit from reaching a
        finite endpoint while lane perception settles after crossing a line.
        """
        previous = self._path_map_to_local(self.last_inner_path or self.committed_path)
        if len(previous) < 2:
            previous = [(0.0, 0.0), (1.0, 0.0)]
        previous = self._extend_local_polyline(
            previous, max(20.0, self.inner_fallback_path_length_m)
        )
        return self._local_to_map(previous, now) if len(previous) >= 3 else None

    def _map_obstacles_local(self) -> List[LocalObstacle]:
        if self.latest_obstacles is None or self.latest_odom is None:
            return []
        ex, ey, yaw, _ = self._odom_pose()
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for o in self.latest_obstacles.obstacles:
            dx = float(o.center_x_map) - ex
            dy = float(o.center_y_map) - ey
            x = c*dx + s*dy
            y = -s*dx + c*dy
            vx = c*float(o.velocity_x_map) + s*float(o.velocity_y_map)
            vy = -s*float(o.velocity_x_map) + c*float(o.velocity_y_map)
            out.append(LocalObstacle(
                int(o.id), x, y, vx, vy,
                max(0.5, float(o.length)), max(0.4, float(o.width)), float(o.yaw),
            ))
        return out

    def _target_lane_neighbors(self, lane_width: float) -> Tuple[Optional[LocalObstacle], Optional[LocalObstacle], List[LocalObstacle]]:
        center = self._centerline_local()
        obs = self._map_obstacles_local()
        front = None
        rear = None
        considered = []
        for o in obs:
            if abs(o.x) > self.gap_search_range_m:
                continue
            cy = interp_y(center, clamp(o.x, 0.0, 25.0))
            if cy is None:
                cy = 0.0
            target_y = cy + lane_width
            # Center-to-center lane membership.  The previous footprint-style
            # allowance (~3 m for a normal lane/car) could mix current- and
            # target-lane vehicles.
            allowance = self.target_lane_center_gate_m
            if abs(o.y - target_y) > allowance:
                continue
            considered.append(o)
            if o.x >= 0.0 and (front is None or o.x < front.x):
                front = o
            if o.x < 0.0 and (rear is None or o.x > rear.x):
                rear = o
        return front, rear, considered

    def _gap_safe_for_speed(self, candidate_speed: float, lane_width: float, change_length: float) -> Tuple[bool, str, dict]:
        front, rear, considered = self._target_lane_neighbors(lane_width)
        t = (self.change_start_m + change_length) / max(candidate_speed, 0.5)
        diag = {"candidate_speed": round(candidate_speed, 2), "t_change": round(t, 2), "objects": [o.oid for o in considered]}

        if front is not None:
            rel_now = front.x - (self.vehicle_center_from_base_m + 0.5*self.vehicle_length_m) - 0.5*front.length
            rel_future = rel_now + (front.vx - candidate_speed) * t
            required = max(self.front_min_gap_m, self.time_headway_s * candidate_speed)
            closing = candidate_speed - front.vx
            ttc = rel_now / closing if closing > 0.05 and rel_now > 0.0 else float("inf")
            diag["front"] = {"id": front.oid, "gap": round(rel_now,2), "future_gap": round(rel_future,2), "v": round(front.vx,2), "ttc": None if not math.isfinite(ttc) else round(ttc,2)}
            if rel_now < required or rel_future < required * 0.75:
                return False, "front_gap", diag
            if ttc < self.min_ttc_s:
                return False, "front_ttc", diag

        if rear is not None:
            ego_rear_from_base = self.vehicle_center_from_base_m - 0.5*self.vehicle_length_m
            rel_now = -rear.x - 0.5*rear.length + ego_rear_from_base
            # Positive means separation behind ego; relative separation evolves by ego - rear speed.
            rel_future = rel_now + (candidate_speed - rear.vx) * t
            required = max(self.rear_min_gap_m, self.time_headway_s * max(rear.vx, 0.0))
            closing = rear.vx - candidate_speed
            ttc = rel_now / closing if closing > 0.05 and rel_now > 0.0 else float("inf")
            diag["rear"] = {"id": rear.oid, "gap": round(rel_now,2), "future_gap": round(rel_future,2), "v": round(rear.vx,2), "ttc": None if not math.isfinite(ttc) else round(ttc,2)}
            if rel_now < required or rel_future < required * 0.75:
                return False, "rear_gap", diag
            if ttc < self.min_ttc_s:
                return False, "rear_ttc", diag

        return True, "ok", diag

    def _path_curvature_ok(self, pts: Sequence[Tuple[float, float]], speed_mps: float) -> Tuple[bool, float]:
        max_k = 0.0
        for i in range(1, len(pts)-1):
            ax, ay = pts[i-1]
            bx, by = pts[i]
            cx, cy = pts[i+1]
            ab = math.hypot(bx-ax, by-ay)
            bc = math.hypot(cx-bx, cy-by)
            ac = math.hypot(cx-ax, cy-ay)
            denom = ab*bc*ac
            if denom < 1e-6:
                continue
            cross = (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)
            k = abs(2.0*cross/denom)
            max_k = max(max_k, k)
        return speed_mps*speed_mps*max_k <= self.max_lateral_accel_mps2 + 1e-6, max_k

    def _dynamic_path_safe(self, path: Optional[RosPath], candidate_speed: float, max_arc_m: Optional[float] = None) -> Tuple[bool, str]:
        if self.latest_obstacles is None or self.latest_odom is None or path is None or len(path.poses) < 3:
            return False, "no_obstacles_or_short_path"
        # A committed path starts at the OLD ego pose. Predict from the current
        # pose and remaining path, otherwise elapsed travel is counted twice and
        # obstacles behind us can veto a maneuver that has already passed them.
        ex, ey, ego_yaw, _ = self._odom_pose()
        nearest = min(range(len(path.poses)), key=lambda i: (
            (path.poses[i].pose.position.x-ex)**2 + (path.poses[i].pose.position.y-ey)**2
        ))
        points = [(ex, ey)] + [
            (ps.pose.position.x, ps.pose.position.y) for ps in path.poses[nearest+1:]
        ]
        arc = [0.0]
        for i in range(1, len(points)):
            a, b = points[i-1], points[i]
            arc.append(arc[-1] + math.hypot(b[0]-a[0], b[1]-a[1]))

        # This check is repeated every control tick.  Extrapolating the entire
        # 40~60 m hold path at a low speed creates a 20+ second prediction and
        # turns normal rear traffic into a false future collision.  A bounded
        # horizon is safer and is refreshed before the vehicle reaches it.
        horizon_arc_m = (
            max(candidate_speed, 0.5) * max(self.dynamic_prediction_horizon_s, 0.5)
            + self.vehicle_length_m
        )
        effective_max_arc_m = (
            horizon_arc_m if max_arc_m is None else min(max_arc_m, horizon_arc_m)
        )

        # A vehicle wholly behind the ego cannot be avoided by braking.  Rear
        # traffic is checked before lane-change commitment by the dedicated gap
        # and TTC rules; it must not stop INNER_HOLD after the merge succeeds.
        ego_rear_x = self.vehicle_center_from_base_m - 0.5*self.vehicle_length_m
        obs = []
        for o in self.latest_obstacles.obstacles:
            dx0 = float(o.center_x_map) - ex
            dy0 = float(o.center_y_map) - ey
            lon0 = math.cos(ego_yaw)*dx0 + math.sin(ego_yaw)*dy0
            obstacle_front_x = lon0 + 0.5*max(0.5, float(o.length))
            if obstacle_front_x <= ego_rear_x:
                continue
            obs.append(o)
        for i, (px, py) in enumerate(points):
            if arc[i] > effective_max_arc_m:
                break
            p0 = points[max(0, i-1)]
            p1 = points[min(len(points)-1, i+1)]
            yaw = ego_yaw if i == 0 else math.atan2(p1[1]-p0[1], p1[0]-p0[0])
            c, s = math.cos(yaw), math.sin(yaw)
            t = arc[i] / max(candidate_speed, 0.5)
            ego_cx = px + self.vehicle_center_from_base_m*c
            ego_cy = py + self.vehicle_center_from_base_m*s
            for o in obs:
                ox = float(o.center_x_map) + float(o.velocity_x_map)*t
                oy = float(o.center_y_map) + float(o.velocity_y_map)*t
                dx, dy = ox-ego_cx, oy-ego_cy
                lon = c*dx + s*dy
                lat = -s*dx + c*dy
                lon_lim = 0.5*self.vehicle_length_m + 0.5*max(0.5,float(o.length)) + self.collision_long_margin_m
                lat_lim = 0.5*self.vehicle_width_m + 0.5*max(0.4,float(o.width)) + self.collision_lat_margin_m
                if abs(lon) <= lon_lim and abs(lat) <= lat_lim:
                    return False, "predicted_collision_id_%d" % int(o.id)
        return True, "ok"

    def _choose_lane_change(self, now: rospy.Time) -> Tuple[Optional[RosPath], Optional[float], Optional[float], str, dict]:
        ok, reason = self._lane_valid(now, require_measured_width=True)
        if not ok:
            return None, None, None, reason, {}
        dashed, dreason = self._left_dashed_ok()
        if not dashed:
            return None, None, None, dreason, {}
        # The sensor-team merge-gap node is itself gated by
        # /perception/camera/highway_environment.  Therefore an explicit mission
        # request would otherwise deadlock whenever that upstream highway gate is
        # false.  Under an explicit request we may skip only that upstream veto;
        # the lane-change still must pass this node's own LiDAR front/rear gap,
        # TTC, predicted-collision, curvature and dashed-line checks below.
        use_sensor_merge_gate = not (
            self.highway_request and self.mission_request_bypass_sensor_merge_gate
        )
        if use_sensor_merge_gate:
            if not self._fresh(self.merge_at, self.merge_timeout_s, now):
                return None, None, None, "merge_gap_stale", {}
            if self.merge_unavailable or not self.merge_available:
                return None, None, None, "sensor_merge_gap_unavailable", {}
        if not self._fresh(self.obstacles_at, self.obstacle_timeout_s, now):
            return None, None, None, "obstacles_stale", {}

        width = float(self.lane_info.get("lane_width_m"))
        divider_ok, divider_reason, divider_diag = self._left_divider_sanity(width)
        if not divider_ok:
            return None, None, None, divider_reason, {"divider": divider_diag}

        _, _, _, ego_speed = self._odom_pose()
        raw_candidates = [
            min(self.cruise_speed_mps, max(ego_speed, 2.0) + 0.5),
            min(self.cruise_speed_mps, max(ego_speed, 2.0)),
            max(1.5, min(self.cruise_speed_mps, ego_speed - 1.0)),
            max(1.5, min(self.cruise_speed_mps, ego_speed - 2.0)),
        ]
        candidates = sorted({round(v, 2) for v in raw_candidates if v > 0.1}, reverse=True)
        diagnostics = {}
        for v in candidates:
            local, length = self._generate_lane_change_local(width, v)
            if len(local) < 3:
                diagnostics[str(v)] = {"divider": divider_diag, "reason": "lane_change_geometry_short"}
                continue
            curv_ok, max_k = self._path_curvature_ok(local, v)
            gap_ok, gap_reason, gap_diag = self._gap_safe_for_speed(v, width, length)
            path = self._local_to_map(local, now)
            dyn_ok, dyn_reason = self._dynamic_path_safe(
                path, v, self.change_start_m + length + 4.0
            )
            diagnostics[str(v)] = {
                "curvature": round(max_k,5),
                "gap": gap_diag,
                "gap_reason": gap_reason,
                "dyn": dyn_reason,
                "divider": divider_diag,
            }
            if curv_ok and gap_ok and dyn_ok:
                return path, v, length, "ok", diagnostics
        return None, None, None, "no_safe_speed_path_pair", diagnostics

    def _current_lane_lead(self) -> Tuple[Optional[LocalObstacle], Optional[float], Optional[float]]:
        # During and immediately after a change, classify traffic against the
        # path actually being driven.  The camera may still label the previous
        # ego lane for a few frames while yaw is returning to zero.
        if self.state == self.LANE_CHANGE:
            center = self._path_map_to_local(self.committed_path)
        elif self.state == self.INNER_HOLD:
            center = self._path_map_to_local(self.last_inner_path)
        else:
            center = self._centerline_local()
        if len(center) < 2:
            center = self._centerline_local()
        lane_width = float((self.lane_info or {}).get("lane_width_m") or 3.5)
        best = None
        best_gap = None
        best_ttc = None
        _, _, _, ego_speed = self._odom_pose()
        for o in self._map_obstacles_local():
            if o.x <= 0.0 or o.x > self.follow_search_m:
                continue
            cy = interp_y(center, clamp(o.x, 0.0, 25.0))
            if cy is None:
                cy = 0.0
            # Strict current-lane center gate.  Adjacent-lane vehicles must
            # never trigger emergency following/stop while ego is still in the
            # current lane.
            if abs(o.y - cy) > self.current_lane_center_gate_m:
                continue
            # Lane membership already excludes adjacent traffic. Keep a negative
            # bumper gap: a close same-lane obstacle is an emergency, not an
            # absent lead. Discarding it would restore cruise as it gets closer.
            ego_front_x = self.vehicle_center_from_base_m + 0.5*self.vehicle_length_m
            obstacle_rear_x = o.x - 0.5*o.length
            gap = obstacle_rear_x - ego_front_x
            if best_gap is None or gap < best_gap:
                closing = ego_speed - o.vx
                ttc = max(gap, 0.0)/closing if closing > 0.05 else float("inf")
                best, best_gap, best_ttc = o, gap, ttc
        return best, best_gap, best_ttc

    def _gap_shaping_speed(self, lane_width: float) -> Tuple[float, dict]:
        """Choose a safe longitudinal speed that tends to create a LEFT-lane slot.

        This never relaxes the merge/TTC rules. It only changes ego speed within
        the normal cruise limit while WAIT_GAP so the vehicle does not passively
        arrive at the physical lane merge with no usable slot.
        """
        _, _, _, ego_speed = self._odom_pose()
        front, rear, _ = self._target_lane_neighbors(lane_width)
        horizon = 3.0
        candidates = []
        v = 1.5
        while v < self.cruise_speed_mps + 1e-6:
            candidates.append(round(v, 2))
            v += 0.5
        candidates.extend([round(clamp(ego_speed, 1.5, self.cruise_speed_mps), 2), round(self.cruise_speed_mps, 2)])
        candidates = sorted(set(candidates), reverse=True)

        best_v = min(self.cruise_speed_mps, max(1.5, ego_speed))
        best_score = -1e9
        best_diag = {}
        for cand in candidates:
            score = 10.0
            diag = {"candidate": cand}
            if front is not None:
                gap0 = front.x - (self.vehicle_center_from_base_m + 0.5*self.vehicle_length_m) - 0.5*front.length
                gapf = gap0 + (front.vx-cand)*horizon
                req = max(self.front_min_gap_m, self.time_headway_s*cand)
                score = min(score, gapf/max(req,0.1))
                diag["front_future_gap"] = round(gapf,2)
            if rear is not None:
                ego_rear_from_base = self.vehicle_center_from_base_m - 0.5*self.vehicle_length_m
                gap0 = -rear.x - 0.5*rear.length + ego_rear_from_base
                gapf = gap0 + (cand-rear.vx)*horizon
                req = max(self.rear_min_gap_m, self.time_headway_s*max(rear.vx,0.0))
                score = min(score, gapf/max(req,0.1))
                diag["rear_future_gap"] = round(gapf,2)
            # Prefer the higher speed when two candidates produce similar gap quality.
            score += 0.02*cand
            if score > best_score:
                best_score = score
                best_v = cand
                best_diag = diag
        best_diag["score"] = round(best_score,3)
        return float(best_v), best_diag

    def _adaptive_speed(self, cruise: float) -> Tuple[float, bool, dict]:
        if self.latest_odom is None:
            return 0.0, True, {"reason": "no_odom"}
        _, _, _, ego_speed = self._odom_pose()
        lead, gap, ttc = self._current_lane_lead()
        if lead is None or gap is None:
            return cruise, False, {"lead": None}
        desired = self.follow_standstill_gap_m + self.follow_time_headway_s * ego_speed
        target = min(cruise, max(0.0, lead.vx + self.follow_gain*(gap-desired)))
        emergency = gap < self.emergency_gap_m or (ttc is not None and math.isfinite(ttc) and ttc < self.emergency_ttc_s)
        return target, emergency, {
            "lead": lead.oid,
            "gap": round(gap,2),
            "lead_v": round(lead.vx,2),
            "ttc": None if ttc is None or not math.isfinite(ttc) else round(ttc,2),
            "desired_gap": round(desired,2),
        }

    def _limit_speed_rate(self, target: float, dt: float) -> float:
        target = clamp(target, 0.0, self.cruise_speed_mps)
        if target > self.last_output_speed:
            out = min(target, self.last_output_speed + self.speed_rise_mps2*dt)
        else:
            out = max(target, self.last_output_speed - self.speed_fall_mps2*dt)
        self.last_output_speed = out
        return out

    def _global_signed_d(self) -> Optional[float]:
        if self.latest_odom is None or len(self.global_points) < 2:
            return None
        x, y, _, _ = self._odom_pose()
        best_d2 = float("inf")
        best_signed = None
        n = len(self.global_points)
        for i in range(n-1):
            a = self.global_points[i]
            b = self.global_points[i+1]
            vx, vy = b.x-a.x, b.y-a.y
            l2 = vx*vx + vy*vy
            if l2 < 1e-9:
                continue
            t = clamp(((x-a.x)*vx + (y-a.y)*vy)/l2, 0.0, 1.0)
            px, py = a.x+t*vx, a.y+t*vy
            dx, dy = x-px, y-py
            d2 = dx*dx+dy*dy
            if d2 < best_d2:
                l = math.sqrt(l2)
                tx, ty = vx/l, vy/l
                signed = tx*dy - ty*dx  # +left
                best_d2 = d2
                best_signed = signed
        return best_signed

    def _publish(self, path: Optional[RosPath], stop: bool, speed: float, active: bool, status: dict, now: rospy.Time, dt: float) -> None:
        if path is not None:
            path.header.stamp = now
            if not path.header.frame_id:
                path.header.frame_id = self.map_frame
            self.path_pub.publish(path)
        else:
            stop = True
        speed_out = self._limit_speed_rate(0.0 if stop else speed, dt)
        self.stop_pub.publish(Bool(data=bool(stop)))
        self.speed_pub.publish(Float64(data=float(speed_out)))
        self.active_pub.publish(Bool(data=bool(active)))
        status.update({"state": self.state, "active": bool(active), "stop": bool(stop), "target_speed_mps": round(speed_out,2), "lane_changes_done": self.lane_changes_done})
        self.state_pub.publish(String(data=json.dumps(status, separators=(",", ":"))))
        if stop:
            rospy.logwarn_throttle(0.5, "HIGHWAY STOP state=%s reason=%s follow=%s", self.state, str(status.get("reason")), json.dumps(status.get("follow", {}), separators=(",", ":")))

    def _tick(self, _event) -> None:
        now = rospy.Time.now()
        if self.last_timer_time is None:
            dt = 1.0/max(self.rate_hz,1.0)
        else:
            dt = clamp((now-self.last_timer_time).to_sec(), 0.001, 0.2)
        self.last_timer_time = now

        base_fresh = self.latest_base_path is not None and self._fresh(self.base_path_at, self.base_timeout_s, now)
        base_stop_fresh = self._fresh(self.base_stop_at, self.base_timeout_s, now)
        odom_fresh = self.latest_odom is not None and self._fresh(self.odom_at, self.odom_timeout_s, now)
        obs_fresh = self.latest_obstacles is not None and self._fresh(self.obstacles_at, self.obstacle_timeout_s, now)
        activation = self._activation_present()

        if not odom_fresh:
            self._publish(self.latest_base_path if base_fresh else None, True, 0.0, self.state != self.OFF, {"reason":"odom_stale"}, now, dt)
            return

        # Distance accumulation for committed states.
        ex, ey, _, ego_speed = self._odom_pose()
        if self.state == self.LANE_CHANGE:
            if self.last_change_xy is not None:
                self.change_travel_m += math.hypot(ex-self.last_change_xy[0], ey-self.last_change_xy[1])
            self.last_change_xy = (ex,ey)
        elif self.state == self.INNER_HOLD:
            if self.last_hold_xy is not None:
                self.inner_hold_travel_m += math.hypot(ex-self.last_hold_xy[0], ey-self.last_hold_xy[1])
            self.last_hold_xy = (ex,ey)
        elif self.state == self.REJOIN:
            if self.last_rejoin_xy is not None:
                self.rejoin_travel_m += math.hypot(ex-self.last_rejoin_xy[0], ey-self.last_rejoin_xy[1])
            self.last_rejoin_xy = (ex,ey)

        if self.completed_once and self.state != self.DONE:
            self.state = self.DONE

        # OFF: transparent pass-through until highway is confidently detected/requested.
        if self.state == self.OFF:
            if activation and not self.completed_once:
                if self.highway_true_since is None:
                    self.highway_true_since = now
                elif (now-self.highway_true_since).to_sec() >= self.highway_confirm_s:
                    self.state = self.WAIT_GAP
                    self.ready_since = None
            else:
                self.highway_true_since = None
            stop = (not base_fresh) or (not base_stop_fresh) or self.base_stop
            self._publish(self.latest_base_path if base_fresh else None, stop, self.cruise_speed_mps, False, {"reason":"base_pass"}, now, dt)
            return

        # DONE: do not re-arm in the same latched-highway scenario.
        if self.state == self.DONE:
            stop = (not base_fresh) or (not base_stop_fresh) or self.base_stop
            self._publish(self.latest_base_path if base_fresh else None, stop, self.cruise_speed_mps, False, {"reason":"highway_strategy_done"}, now, dt)
            return

        # WAIT_GAP: stay on the existing global/avoidance path, but start adapting speed.
        if self.state == self.WAIT_GAP:
            adaptive, emergency, follow = self._adaptive_speed(self.cruise_speed_mps) if obs_fresh and self.lane_info is not None else (self.cruise_speed_mps, False, {})
            shaping = self.cruise_speed_mps
            shaping_diag = {}
            lane_ok_for_shape, _ = self._lane_valid(now, require_measured_width=True)
            if obs_fresh and lane_ok_for_shape:
                shaping, shaping_diag = self._gap_shaping_speed(float(self.lane_info.get("lane_width_m")))
            wait_speed = min(adaptive, shaping)
            path, cand_speed, length, reason, diag = self._choose_lane_change(now)
            if path is not None:
                if self.ready_since is None:
                    self.ready_since = now
                elif (now-self.ready_since).to_sec() >= self.ready_confirm_s:
                    self.committed_path = path
                    self.committed_speed_mps = float(cand_speed)
                    self.committed_change_length_m = float(length)
                    self.change_travel_m = 0.0
                    self.last_change_xy = (ex,ey)
                    self.complete_since = None
                    self.state = self.LANE_CHANGE
                    rospy.logwarn("HIGHWAY lane change COMMITTED speed=%.2f length=%.1f", cand_speed, length)
            else:
                self.ready_since = None
            stop = (not base_fresh) or (not base_stop_fresh) or self.base_stop or emergency
            self._publish(self.latest_base_path if base_fresh else None, stop, wait_speed, True, {"reason":reason, "follow":follow, "gap_shaping":shaping_diag, "candidate_diag":diag}, now, dt)
            return

        if self.state == self.LANE_CHANGE:
            # Geometry remains committed. Do not regenerate from camera because the
            # camera's ego-lane identity can switch midway through the maneuver.
            lane_ok, lane_reason = self._lane_valid(now)
            adaptive, emergency, follow = self._adaptive_speed(self.committed_speed_mps) if obs_fresh and lane_ok else (self.committed_speed_mps, False, {})
            speed = min(self.committed_speed_mps, adaptive)

            # Do not re-run the entry gap threshold after commitment, but keep
            # checking actual predicted collisions along the remaining trajectory
            # even if the camera changes lane identity or drops out.
            path_safe, path_reason = self._dynamic_path_safe(self.committed_path, speed) if obs_fresh else (False, "obstacles_stale")
            stop = emergency or not path_safe

            lat = None if self.lane_info is None else self.lane_info.get("lateral_error_m")
            head = None if self.lane_info is None else self.lane_info.get("heading_error_rad")
            transition_done = self.change_travel_m >= (self.change_start_m + self.committed_change_length_m)
            settle_needed = self.change_start_m + self.committed_change_length_m + min(self.change_settle_m, self.change_post_hold_m)
            settled = self.change_travel_m >= settle_needed
            progressed = settled
            centered = lane_ok and lat is not None and head is not None and abs(float(lat)) <= self.change_center_error_m and abs(float(head)) <= self.change_heading_error_rad

            # The committed lane-change path is finite, while the sensor-team
            # Pure Pursuit intentionally stops at the end of any finite path.
            # Do not wait for a camera-centering condition all the way to the
            # endpoint: after the planned lateral transition plus the settle
            # distance, check actual position and heading before hand-over. A second
            # endpoint-distance guard guarantees hand-over before PP enters its
            # ~1.5 m goal-stop zone even if odometry distance accumulation is a
            # little noisy.
            remaining_to_end = None
            if self.committed_path is not None and self.committed_path.poses:
                ep = self.committed_path.poses[-1].pose.position
                remaining_to_end = math.hypot(float(ep.x)-ex, float(ep.y)-ey)
            endpoint_guard_complete = (
                transition_done
                and remaining_to_end is not None
                and remaining_to_end <= self.change_endpoint_guard_m
            )
            geometry_complete = settled or endpoint_guard_complete
            aligned, alignment_diag = self._committed_alignment()

            if geometry_complete and aligned and not stop:
                if self.complete_since is None:
                    self.complete_since = now
                elif (now-self.complete_since).to_sec() >= self.change_complete_confirm_s:
                    self.lane_changes_done += 1
                    self.state = self.INNER_HOLD
                    self.inner_hold_travel_m = 0.0
                    self.last_hold_xy = (ex,ey)
                    self.release_since = None
                    self.lane_invalid_since = None
                    self.ready_since = None
                    self.last_inner_path = self.committed_path
                    self.inner_handover_pending = True
                    self.inner_lane_candidate_since = None
                    self.inner_lane_candidate_signature = None
                    why = "settled" if settled else "endpoint_guard"
                    rospy.logwarn(
                        "HIGHWAY lane change COMPLETE count=%d reason=%s remaining=%.2fm",
                        self.lane_changes_done, why,
                        -1.0 if remaining_to_end is None else remaining_to_end,
                    )
            else:
                self.complete_since = None

            stop_reason = "lead_emergency" if emergency else (path_reason if not path_safe else lane_reason)
            self._publish(self.committed_path, stop, speed, True, {
                "reason":stop_reason,
                "travel_m":round(self.change_travel_m,2),
                "transition_done":transition_done,
                "settled":settled,
                "settle_needed_m":round(settle_needed,2),
                "progressed":progressed,
                "centered":centered,
                "remaining_to_end_m":None if remaining_to_end is None else round(remaining_to_end,2),
                "endpoint_guard_complete":endpoint_guard_complete,
                "geometry_complete":geometry_complete,
                "aligned":aligned,
                "alignment":alignment_diag,
                "follow":follow,
            }, now, dt)
            return

        if self.state == self.INNER_HOLD:
            lane_ok, lane_reason = self._lane_valid(now)
            center_ok = False
            center_y = None
            if self.inner_handover_pending:
                if lane_ok:
                    signature = self._lane_signature()
                    if signature != self.inner_lane_candidate_signature:
                        self.inner_lane_candidate_signature = signature
                        self.inner_lane_candidate_since = now
                        lane_ok = False
                        lane_reason = "lane_handover_confirming"
                    elif self.inner_lane_candidate_since is None:
                        self.inner_lane_candidate_since = now
                        lane_ok = False
                        lane_reason = "lane_handover_confirming"
                    elif (now-self.inner_lane_candidate_since).to_sec() < self.inner_handover_confirm_s:
                        lane_ok = False
                        lane_reason = "lane_handover_confirming"
                    else:
                        self.inner_handover_pending = False
                        self.inner_lane_candidate_since = None
                        self.inner_lane_candidate_signature = None
                else:
                    self.inner_lane_candidate_since = None
                    self.inner_lane_candidate_signature = None
            if lane_ok:
                center_ok, center_reason, center_y = self._inner_center_sanity()
                if not center_ok:
                    lane_ok = False
                    lane_reason = center_reason

            if lane_ok:
                filtered, filter_reason = self._filtered_inner_path(now, dt)
                if filtered is None:
                    lane_ok = False
                    lane_reason = filter_reason
                else:
                    self.last_inner_path = filtered

            if lane_ok:
                self.lane_invalid_since = None
            elif self.lane_invalid_since is None:
                self.lane_invalid_since = now

            path = self.last_inner_path
            lane_grace = (
                not lane_ok
                and path is not None
                and self.lane_invalid_since is not None
                and (now-self.lane_invalid_since).to_sec() <= self.inner_lane_invalid_grace_s
            )
            handover_wait = self.inner_handover_pending and lane_grace
            lane_fallback = False
            if not lane_ok:
                fallback = self._rolling_inner_fallback(now)
                if fallback is not None:
                    path = fallback
                    self.last_inner_path = fallback
                    lane_fallback = True

            if obs_fresh and lane_ok:
                adaptive, emergency, follow = self._adaptive_speed(self.cruise_speed_mps)
            elif obs_fresh and handover_wait:
                adaptive, emergency, follow = min(self.cruise_speed_mps, self.committed_speed_mps), False, {}
            elif obs_fresh:
                adaptive, emergency, follow = self.inner_lane_grace_speed_mps, False, {}
            else:
                adaptive, emergency, follow = 0.0, False, {}

            if emergency:
                stop = True
                inner_reason = "lead_emergency"
            elif not obs_fresh:
                stop = True
                inner_reason = "obstacles_stale"
            elif lane_ok:
                stop = path is None
                inner_reason = "ok" if path is not None else "inner_path_missing"
            elif handover_wait:
                stop = False
                inner_reason = (lane_reason if lane_reason.startswith("lane_handover_")
                                else "lane_handover_" + lane_reason)
            elif lane_grace or lane_fallback:
                stop = False
                adaptive = min(adaptive, self.inner_lane_grace_speed_mps)
                inner_reason = (
                    "lane_grace_" if lane_grace else "lane_fallback_"
                ) + lane_reason
            else:
                stop = True
                inner_reason = lane_reason

            path_safe, path_reason = self._dynamic_path_safe(path, adaptive) if obs_fresh else (False, "obstacles_stale")
            if not stop and not path_safe:
                stop = True
                inner_reason = path_reason

            # Up to two changes. Follow and settle in each lane before starting
            # a NEW uninterrupted gap confirmation; never reuse the first one.
            lat = (self.lane_info or {}).get("lateral_error_m")
            heading = (self.lane_info or {}).get("heading_error_rad")
            settled_for_next = (
                lane_ok and not stop
                and self.inner_hold_travel_m >= self.min_lane_hold_before_next_change_m
                and lat is not None and abs(float(lat)) <= self.change_center_error_m
                and heading is not None and abs(float(heading)) <= self.change_heading_error_rad
                and (self.lane_info or {}).get("output_status", "FRESH") == "FRESH"
            )
            next_change_pending = False
            if settled_for_next and self.lane_changes_done < self.target_left_lane_changes:
                p, v, length, reason, diag = self._choose_lane_change(now)
                if p is not None:
                    next_change_pending = True
                    if self.ready_since is None:
                        self.ready_since = now
                    elif (now-self.ready_since).to_sec() >= self.ready_confirm_s:
                        self.committed_path = p
                        self.committed_speed_mps = float(v)
                        self.committed_change_length_m = float(length)
                        self.change_travel_m = 0.0
                        self.last_change_xy = (ex,ey)
                        self.complete_since = None
                        self.ready_since = None
                        self.state = self.LANE_CHANGE
                        self._publish(self.committed_path, False, self.committed_speed_mps, True, {"reason":"next_left_lane_change", "candidate_diag":diag}, now, dt)
                        return
                else:
                    self.ready_since = None
            else:
                self.ready_since = None

            global_d = self._global_signed_d()
            can_start_rejoin = (
                self.lane_changes_done > 0
                and not next_change_pending
                and self.inner_hold_travel_m >= self.min_inner_hold_after_change_m
                and global_d is not None
                and abs(global_d) <= self.rejoin_start_global_d_m
                and lane_ok
                and base_fresh
                and base_stop_fresh
                and not self.base_stop
                and not stop
            )
            rejoin_blocked = False
            if can_start_rejoin:
                rejoin = self._generate_rejoin_path(now)
                rejoin_safe, rejoin_reason = self._dynamic_path_safe(rejoin, adaptive)
                if rejoin_safe:
                    self.committed_rejoin_path = rejoin
                    self.rejoin_travel_m = 0.0
                    self.last_rejoin_xy = (ex, ey)
                    self.release_since = None
                    self.state = self.REJOIN
                    rospy.logwarn("HIGHWAY REJOIN COMMITTED global_d=%.2f length=%.1f", global_d, self.rejoin_length_m)
                    self._publish(rejoin, False, adaptive, True, {"reason":"rejoin_committed", "global_d":round(global_d,2), "follow":follow}, now, dt)
                    return
                # Keep the current lane and stop until the proposed merge is
                # clear. Do not fall through to the direct-release shortcut.
                stop = True
                inner_reason = "rejoin_" + rejoin_reason
                rejoin_blocked = True

            # Failsafe direct release only when the two paths are already almost
            # coincident. This also prevents a lane-info dropout at the physical
            # merge from stopping the car forever.
            can_direct_release = (
                self.lane_changes_done > 0
                and not next_change_pending
                and self.inner_hold_travel_m >= self.min_inner_hold_after_change_m
                and global_d is not None
                and abs(global_d) <= self.release_global_d_m
                and base_fresh
                and base_stop_fresh
                and not self.base_stop
                # A camera-only stop may release onto the coincident base path,
                # but a motion safety stop or rejected rejoin must never do so.
                and obs_fresh
                and not emergency
                and path_safe
                and not rejoin_blocked
            )
            if can_direct_release:
                release_safe, release_reason = self._dynamic_path_safe(self.latest_base_path, adaptive)
                if not release_safe:
                    self.release_since = None
                    stop = True
                    inner_reason = "release_" + release_reason
                elif self.release_since is None:
                    self.release_since = now
                elif (now-self.release_since).to_sec() >= self.release_confirm_s:
                    self.completed_once = True
                    self.state = self.DONE
                    self._publish(self.latest_base_path, (not base_stop_fresh) or self.base_stop, adaptive, False, {"reason":"direct_release_near_global", "global_d":global_d, "follow":follow}, now, dt)
                    rospy.logwarn("HIGHWAY strategy DONE: direct global release d=%.2f", global_d)
                    return
            else:
                self.release_since = None

            self._publish(path, stop, adaptive, True, {
                "reason": inner_reason,
                "global_d": None if global_d is None else round(global_d,2),
                "inner_hold_travel_m": round(self.inner_hold_travel_m,2),
                "center_y8_m": None if center_y is None else round(center_y,3),
                "lane_center_source": (self.lane_info or {}).get("center_source"),
                "lane_straddling": bool(((self.lane_info or {}).get("straddling_lane") or {}).get("detected", False)),
                "lane_grace": lane_grace,
                "lane_fallback": lane_fallback,
                "lane_handover_pending": self.inner_handover_pending,
                "follow": follow,
            }, now, dt)
            return

        if self.state == self.REJOIN:
            adaptive, emergency, follow = self._adaptive_speed(self.cruise_speed_mps) if obs_fresh and self.lane_info is not None else (self.cruise_speed_mps, False, {})
            global_d = self._global_signed_d()
            path_safe, path_reason = self._dynamic_path_safe(self.committed_rejoin_path, adaptive) if obs_fresh else (False, "obstacles_stale")
            stop = emergency or not path_safe or not base_stop_fresh or self.base_stop
            close_enough = global_d is not None and abs(global_d) <= self.rejoin_complete_global_d_m
            progressed = self.rejoin_travel_m >= 0.65*self.rejoin_length_m
            if (progressed or close_enough) and base_fresh and not stop:
                release_safe, release_reason = self._dynamic_path_safe(self.latest_base_path, adaptive)
                if not release_safe:
                    self.release_since = None
                    stop = True
                    path_reason = "release_" + release_reason
                elif self.release_since is None:
                    self.release_since = now
                elif (now-self.release_since).to_sec() >= self.release_confirm_s:
                    self.completed_once = True
                    self.state = self.DONE
                    self._publish(self.latest_base_path, (not base_stop_fresh) or self.base_stop, adaptive, False, {"reason":"rejoin_complete", "global_d":global_d, "travel_m":round(self.rejoin_travel_m,2), "follow":follow}, now, dt)
                    rospy.logwarn("HIGHWAY REJOIN COMPLETE global_d=%s travel=%.1f", "n/a" if global_d is None else "%.2f" % global_d, self.rejoin_travel_m)
                    return
            else:
                self.release_since = None
            reason = "lead_emergency" if emergency else ("base_stop" if self.base_stop else ("base_stop_stale" if not base_stop_fresh else (path_reason if stop else "rejoining")))
            self._publish(self.committed_rejoin_path, stop, adaptive, True, {"reason":reason, "global_d":None if global_d is None else round(global_d,2), "travel_m":round(self.rejoin_travel_m,2), "progressed":progressed, "follow":follow}, now, dt)
            return


if __name__ == "__main__":
    try:
        HighwayLaneStrategyNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
