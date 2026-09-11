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
  2) generates a committed quintic lateral shift into the inner lane;
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
        self.target_left_lane_changes = int(rospy.get_param("~target_left_lane_changes", 1))
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
        self.max_heading_error_rad = float(rospy.get_param("~max_heading_error_rad", math.radians(22.0)))
        self.require_left_dashed = bool(rospy.get_param("~require_left_dashed", True))

        self.vehicle_length_m = float(rospy.get_param("~vehicle_length_m", 4.635))
        self.vehicle_width_m = float(rospy.get_param("~vehicle_width_m", 1.892))
        self.vehicle_center_from_base_m = float(rospy.get_param("~vehicle_center_from_base_m", 1.50))

        self.change_start_m = float(rospy.get_param("~change_start_m", 3.0))
        self.change_min_length_m = float(rospy.get_param("~change_min_length_m", 22.0))
        self.change_max_length_m = float(rospy.get_param("~change_max_length_m", 26.0))
        self.change_time_s = float(rospy.get_param("~change_time_s", 5.5))
        self.change_post_hold_m = float(rospy.get_param("~change_post_hold_m", 10.0))
        self.change_complete_min_ratio = float(rospy.get_param("~change_complete_min_ratio", 0.72))
        self.change_center_error_m = float(rospy.get_param("~change_center_error_m", 0.45))
        self.change_heading_error_rad = float(rospy.get_param("~change_heading_error_rad", math.radians(10.0)))
        self.change_complete_confirm_s = float(rospy.get_param("~change_complete_confirm_s", 0.45))

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
        self.lead_overlap_tolerance_m = float(rospy.get_param("~lead_overlap_tolerance_m", 0.25))
        self.change_settle_m = float(rospy.get_param("~change_settle_m", 6.0))
        self.speed_rise_mps2 = float(rospy.get_param("~speed_rise_mps2", 0.8))
        self.speed_fall_mps2 = float(rospy.get_param("~speed_fall_mps2", 1.8))

        self.collision_long_margin_m = float(rospy.get_param("~collision_long_margin_m", 0.5))
        self.collision_lat_margin_m = float(rospy.get_param("~collision_lat_margin_m", 0.35))
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
            "Highway lane strategy: LEFT lane changes=%d cruise=%.2f m/s; camera files are read-only inputs",
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

    def _lane_valid(self, now: rospy.Time) -> Tuple[bool, str]:
        if self.lane_info is None or not self._fresh(self.lane_info_at, self.lane_info_timeout_s, now):
            return False, "lane_info_missing_or_stale"
        d = self.lane_info
        if not bool(d.get("lane_valid", False)):
            return False, "lane_invalid"
        if float(d.get("confidence", 0.0) or 0.0) < self.min_lane_confidence:
            return False, "lane_confidence"
        width = d.get("lane_width_m")
        if width is None or not (self.lane_width_min_m <= float(width) <= self.lane_width_max_m):
            return False, "lane_width"
        heading = d.get("heading_error_rad")
        if heading is None or abs(float(heading)) > self.max_heading_error_rad:
            return False, "lane_heading"
        pts = d.get("centerline_points") or []
        if len(pts) < 3:
            return False, "centerline_short"
        return True, "ok"

    def _left_dashed_ok(self) -> Tuple[bool, str]:
        if not self.require_left_dashed:
            return True, "disabled"
        left = (self.lane_info or {}).get("left_lane") or {}
        if not bool(left.get("detected", False)):
            return False, "left_not_detected"
        if left.get("dashed") is True:
            return True, "ok"
        return False, "left_not_dashed"

    def _centerline_local(self) -> List[Tuple[float, float]]:
        raw = (self.lane_info or {}).get("centerline_points") or []
        pts: List[Tuple[float, float]] = [(0.0, 0.0)]
        for p in raw:
            try:
                x, y = float(p[0]), float(p[1])
            except Exception:
                continue
            if math.isfinite(x) and math.isfinite(y) and x > 0.5:
                pts.append((x, y))
        pts.sort(key=lambda q: q[0])
        # Remove near-duplicates.
        out: List[Tuple[float, float]] = []
        for p in pts:
            if not out or math.hypot(p[0]-out[-1][0], p[1]-out[-1][1]) > 0.15:
                out.append(p)
        return out

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
        base = self._centerline_local()
        length = clamp(max(speed_mps, 1.0) * self.change_time_s, self.change_min_length_m, self.change_max_length_m)
        # Camera centerline ends at ~25 m. Extend only its final tangent so the
        # quintic lane change can be longer and still give Pure Pursuit a stable
        # post-change segment to follow.
        target_len = self.change_start_m + length + self.change_post_hold_m
        base = self._extend_local_polyline(base, target_len)
        arc = polyline_arclength(base)
        shifted: List[Tuple[float, float]] = []
        for i, (x, y) in enumerate(base):
            tx, ty = tangent_at(base, i)
            nx, ny = -ty, tx  # left normal; for straight +x this is +y
            u = (arc[i] - self.change_start_m) / max(length, 1e-6)
            w = smoothstep5(u)
            shifted.append((x + w * lane_width * nx, y + w * lane_width * ny))
        return shifted, length

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

    def _dynamic_path_safe(self, path: RosPath, candidate_speed: float, max_arc_m: Optional[float] = None) -> Tuple[bool, str]:
        if self.latest_obstacles is None or len(path.poses) < 3:
            return False, "no_obstacles_or_short_path"
        # Arc length / time along the ego base_link path.
        arc = [0.0]
        for i in range(1, len(path.poses)):
            a = path.poses[i-1].pose.position
            b = path.poses[i].pose.position
            arc.append(arc[-1] + math.hypot(b.x-a.x, b.y-a.y))

        obs = list(self.latest_obstacles.obstacles)
        for i, ps in enumerate(path.poses):
            if i == 0:
                continue
            if max_arc_m is not None and arc[i] > max_arc_m:
                break
            p = ps.pose.position
            p0 = path.poses[max(0, i-1)].pose.position
            p1 = path.poses[min(len(path.poses)-1, i+1)].pose.position
            yaw = math.atan2(p1.y-p0.y, p1.x-p0.x)
            c, s = math.cos(yaw), math.sin(yaw)
            t = arc[i] / max(candidate_speed, 0.5)
            ego_cx = p.x + self.vehicle_center_from_base_m*c
            ego_cy = p.y + self.vehicle_center_from_base_m*s
            for o in obs:
                ox = float(o.center_x_map) + float(o.velocity_x_map)*t
                oy = float(o.center_y_map) + float(o.velocity_y_map)*t
                dx, dy = ox-ego_cx, oy-ego_cy
                lon = c*dx + s*dy
                lat = -s*dx + c*dy
                lon_lim = 0.5*self.vehicle_length_m + 0.5*max(0.5,float(o.length)) + self.collision_long_margin_m
                lat_lim = 0.5*self.vehicle_width_m + 0.5*max(0.4,float(o.width)) + self.collision_lat_margin_m
                if abs(lon) <= lon_lim and abs(lat) <= lat_lim:
                    # Allow a very short prefix if the obstacle box already overlaps
                    # because LiDAR boxes can transiently touch the ego footprint.
                    if arc[i] > 2.0:
                        return False, "predicted_collision_id_%d" % int(o.id)
        return True, "ok"

    def _choose_lane_change(self, now: rospy.Time) -> Tuple[Optional[RosPath], Optional[float], Optional[float], str, dict]:
        ok, reason = self._lane_valid(now)
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
            curv_ok, max_k = self._path_curvature_ok(local, v)
            gap_ok, gap_reason, gap_diag = self._gap_safe_for_speed(v, width, length)
            path = self._local_to_map(local, now)
            dyn_ok, dyn_reason = self._dynamic_path_safe(
                path, v, self.change_start_m + length + 4.0
            )
            diagnostics[str(v)] = {"curvature": round(max_k,5), "gap": gap_diag, "gap_reason": gap_reason, "dyn": dyn_reason}
            if curv_ok and gap_ok and dyn_ok:
                return path, v, length, "ok", diagnostics
        return None, None, None, "no_safe_speed_path_pair", diagnostics

    def _current_lane_lead(self) -> Tuple[Optional[LocalObstacle], Optional[float], Optional[float]]:
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
            # Lead following is bumper-to-bumper, not center-x based.  A vehicle
            # whose rear bumper is still alongside/behind the ego front must not be
            # treated as a front lead merely because its CENTER transformed to x>0.
            ego_front_x = self.vehicle_center_from_base_m + 0.5*self.vehicle_length_m
            obstacle_rear_x = o.x - 0.5*o.length
            gap = obstacle_rear_x - ego_front_x
            if gap < -self.lead_overlap_tolerance_m:
                continue
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
            lane_ok_for_shape, _ = self._lane_valid(now)
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

            # Only front/imminent safety may stop a committed change. Rear-gap
            # changes after commitment must not make us brake into the approaching car.
            stop = emergency or not obs_fresh or self.committed_path is None

            lat = None if self.lane_info is None else self.lane_info.get("lateral_error_m")
            head = None if self.lane_info is None else self.lane_info.get("heading_error_rad")
            transition_done = self.change_travel_m >= (self.change_start_m + self.committed_change_length_m)
            settle_needed = self.change_start_m + self.committed_change_length_m + min(self.change_settle_m, self.change_post_hold_m)
            settled = self.change_travel_m >= settle_needed
            progressed = settled
            centered = lane_ok and lat is not None and head is not None and abs(float(lat)) <= self.change_center_error_m and abs(float(head)) <= self.change_heading_error_rad
            # Do not switch from the committed quintic to the camera centerline
            # immediately after lateral translation.  Keep several metres of the
            # post-change straight segment so Pure Pursuit can settle first.
            fallback_complete = (
                self.change_travel_m >= (self.change_start_m + self.committed_change_length_m + 0.9*self.change_post_hold_m)
                and lane_ok
                and head is not None
                and abs(float(head)) <= math.radians(15.0)
            )
            if (settled and centered) or fallback_complete:
                if self.complete_since is None:
                    self.complete_since = now
                elif (now-self.complete_since).to_sec() >= self.change_complete_confirm_s:
                    self.lane_changes_done += 1
                    self.state = self.INNER_HOLD
                    self.inner_hold_travel_m = 0.0
                    self.last_hold_xy = (ex,ey)
                    self.release_since = None
                    rospy.logwarn("HIGHWAY lane change COMPLETE count=%d", self.lane_changes_done)
            else:
                self.complete_since = None

            stop_reason = "lead_emergency" if emergency else ("obstacles_stale" if not obs_fresh else lane_reason)
            self._publish(self.committed_path, stop, speed, True, {"reason":stop_reason, "travel_m":round(self.change_travel_m,2), "transition_done":transition_done, "settled":settled, "settle_needed_m":round(settle_needed,2), "progressed":progressed, "centered":centered, "fallback_complete":fallback_complete, "follow":follow}, now, dt)
            return

        if self.state == self.INNER_HOLD:
            lane_ok, lane_reason = self._lane_valid(now)
            if lane_ok:
                local = self._extend_local_polyline(self._centerline_local(), 40.0)
                self.last_inner_path = self._local_to_map(local, now)
            path = self.last_inner_path
            adaptive, emergency, follow = self._adaptive_speed(self.cruise_speed_mps) if obs_fresh and lane_ok else (0.0, True, {})
            stop = emergency or not obs_fresh or not lane_ok or path is None
            if emergency:
                inner_reason = "lead_emergency"
            elif not obs_fresh:
                inner_reason = "obstacles_stale"
            elif not lane_ok:
                inner_reason = lane_reason
            elif path is None:
                inner_reason = "inner_path_missing"
            else:
                inner_reason = lane_reason

            # Optional repeated left changes for a >2-lane highway. Default is 1.
            if self.lane_changes_done < self.target_left_lane_changes:
                p, v, length, reason, diag = self._choose_lane_change(now)
                if p is not None:
                    if self.ready_since is None:
                        self.ready_since = now
                    elif (now-self.ready_since).to_sec() >= self.ready_confirm_s:
                        self.committed_path = p
                        self.committed_speed_mps = float(v)
                        self.committed_change_length_m = float(length)
                        self.change_travel_m = 0.0
                        self.last_change_xy = (ex,ey)
                        self.complete_since = None
                        self.state = self.LANE_CHANGE
                        self._publish(self.committed_path, False, self.committed_speed_mps, True, {"reason":"next_left_lane_change", "candidate_diag":diag}, now, dt)
                        return
                else:
                    self.ready_since = None

            global_d = self._global_signed_d()
            can_start_rejoin = (
                self.lane_changes_done >= self.target_left_lane_changes
                and self.inner_hold_travel_m >= self.min_inner_hold_after_change_m
                and global_d is not None
                and abs(global_d) <= self.rejoin_start_global_d_m
                and lane_ok
                and base_fresh
            )
            if can_start_rejoin:
                rejoin = self._generate_rejoin_path(now)
                if rejoin is not None:
                    self.committed_rejoin_path = rejoin
                    self.rejoin_travel_m = 0.0
                    self.last_rejoin_xy = (ex, ey)
                    self.release_since = None
                    self.state = self.REJOIN
                    rospy.logwarn("HIGHWAY REJOIN COMMITTED global_d=%.2f length=%.1f", global_d, self.rejoin_length_m)
                    self._publish(rejoin, False, adaptive, True, {"reason":"rejoin_committed", "global_d":round(global_d,2), "follow":follow}, now, dt)
                    return

            # Failsafe direct release only when the two paths are already almost
            # coincident. This also prevents a lane-info dropout at the physical
            # merge from stopping the car forever.
            can_direct_release = (
                self.lane_changes_done >= self.target_left_lane_changes
                and self.inner_hold_travel_m >= self.min_inner_hold_after_change_m
                and global_d is not None
                and abs(global_d) <= self.release_global_d_m
                and base_fresh
            )
            if can_direct_release:
                if self.release_since is None:
                    self.release_since = now
                elif (now-self.release_since).to_sec() >= self.release_confirm_s:
                    self.completed_once = True
                    self.state = self.DONE
                    self._publish(self.latest_base_path, (not base_stop_fresh) or self.base_stop, adaptive, False, {"reason":"direct_release_near_global", "global_d":global_d, "follow":follow}, now, dt)
                    rospy.logwarn("HIGHWAY strategy DONE: direct global release d=%.2f", global_d)
                    return
            else:
                self.release_since = None

            self._publish(path, stop, adaptive, True, {"reason":inner_reason, "global_d":None if global_d is None else round(global_d,2), "inner_hold_travel_m":round(self.inner_hold_travel_m,2), "follow":follow}, now, dt)
            return

        if self.state == self.REJOIN:
            adaptive, emergency, follow = self._adaptive_speed(self.cruise_speed_mps) if obs_fresh and self.lane_info is not None else (self.cruise_speed_mps, False, {})
            global_d = self._global_signed_d()
            stop = emergency or not obs_fresh or self.committed_rejoin_path is None
            close_enough = global_d is not None and abs(global_d) <= self.rejoin_complete_global_d_m
            progressed = self.rejoin_travel_m >= 0.65*self.rejoin_length_m
            if (progressed or close_enough) and base_fresh:
                if self.release_since is None:
                    self.release_since = now
                elif (now-self.release_since).to_sec() >= self.release_confirm_s:
                    self.completed_once = True
                    self.state = self.DONE
                    self._publish(self.latest_base_path, (not base_stop_fresh) or self.base_stop, adaptive, False, {"reason":"rejoin_complete", "global_d":global_d, "travel_m":round(self.rejoin_travel_m,2), "follow":follow}, now, dt)
                    rospy.logwarn("HIGHWAY REJOIN COMPLETE global_d=%s travel=%.1f", "n/a" if global_d is None else "%.2f" % global_d, self.rejoin_travel_m)
                    return
            else:
                self.release_since = None
            self._publish(self.committed_rejoin_path, stop, adaptive, True, {"reason":"rejoining", "global_d":None if global_d is None else round(global_d,2), "travel_m":round(self.rejoin_travel_m,2), "progressed":progressed, "follow":follow}, now, dt)
            return


if __name__ == "__main__":
    try:
        HighwayLaneStrategyNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
