#!/usr/bin/env python3
"""Committed active-path manager for the first controlled MORAI avoidance test.

Scope of this stage
-------------------
* NORMAL: publish a rolling local slice of the original global path.
* BYPASS commit: when the planner reports a safe ``bypass`` candidate, copy it
  once and keep following that same path until the vehicle has returned to the
  global route.  Planner replans do not replace the committed maneuver.
* NO SAFE PATH / unsupported lane-change-only situation: request a stop.
* While a bypass is committed, re-check the remaining committed path against
  the latest LiDAR obstacle OBBs.  If it becomes blocked or sensors go stale,
  request a stop while keeping the committed path.

Deliberate limitation: lane-change candidates are NOT executed in this stage.
They remain visualization/evaluation only until a lane hold + return state
machine is added.
"""

from __future__ import annotations

import bisect
import json
import math
from typing import List, Optional, Sequence, Tuple

import rospy
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from lidar_perception.msg import LidarObstacleArray
from purepursuit_mgeo.path import PathPoint, load_mgeo_path
from frenet_path import ReferencePath
from trajectory_safety import ObstacleBox, check_path_collision


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _points_from_ros_path(msg: RosPath) -> List[PathPoint]:
    return [
        PathPoint(
            float(ps.pose.position.x),
            float(ps.pose.position.y),
            float(ps.pose.position.z),
        )
        for ps in msg.poses
    ]


def _path_lengths(points: Sequence[PathPoint]) -> List[float]:
    result = [0.0]
    for i in range(len(points) - 1):
        result.append(
            result[-1]
            + math.hypot(
                points[i + 1].x - points[i].x,
                points[i + 1].y - points[i].y,
            )
        )
    return result


def _nearest_index(points: Sequence[PathPoint], x: float, y: float) -> int:
    return min(
        range(len(points)),
        key=lambda i: (points[i].x - x) ** 2 + (points[i].y - y) ** 2,
    )


def _path_heading(points: Sequence[PathPoint], index: int) -> float:
    if len(points) < 2:
        return 0.0
    if index <= 0:
        a, b = points[0], points[1]
    elif index >= len(points) - 1:
        a, b = points[-2], points[-1]
    else:
        a, b = points[index - 1], points[index + 1]
    return math.atan2(b.y - a.y, b.x - a.x)


class AvoidancePathManager:
    NORMAL = "NORMAL"
    AVOIDING = "AVOIDING_BYPASS"
    AVOIDING_BLOCKED = "AVOIDING_BLOCKED"
    AVOIDING_SENSOR_STOP = "AVOIDING_SENSOR_STOP"
    STOP_NO_SAFE = "STOP_NO_SAFE_PATH"
    STOP_UNSUPPORTED = "STOP_UNSUPPORTED_MANEUVER"
    STOP_PLANNER = "STOP_PLANNER_NOT_READY"

    def __init__(self) -> None:
        rospy.init_node("avoidance_path_manager", anonymous=False)

        path_file = rospy.get_param("~path_file")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odometry")
        self.obstacle_topic = rospy.get_param(
            "~obstacle_topic", "/perception/lidar/tracked_obstacles_map"
        )
        self.selected_path_topic = rospy.get_param(
            "~selected_path_topic", "/avoidance_frenet_debug/selected_path"
        )
        self.planner_ready_topic = rospy.get_param(
            "~planner_ready_topic", "/avoidance_frenet_debug/planner_ready"
        )
        self.avoidance_required_topic = rospy.get_param(
            "~avoidance_required_topic", "/avoidance_frenet_debug/avoidance_required"
        )
        self.safe_path_available_topic = rospy.get_param(
            "~safe_path_available_topic", "/avoidance_frenet_debug/safe_path_available"
        )
        self.selected_kind_topic = rospy.get_param(
            "~selected_kind_topic", "/avoidance_frenet_debug/selected_kind"
        )
        self.selected_side_topic = rospy.get_param(
            "~selected_side_topic", "/avoidance_frenet_debug/selected_side"
        )

        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.50))
        self.obstacle_timeout_s = float(rospy.get_param("~obstacle_timeout_s", 0.60))
        self.planner_signal_timeout_s = float(
            rospy.get_param("~planner_signal_timeout_s", 0.70)
        )
        self.selected_path_timeout_s = float(
            rospy.get_param("~selected_path_timeout_s", 0.70)
        )

        self.normal_back_m = float(rospy.get_param("~normal_back_m", 3.0))
        self.normal_forward_m = float(rospy.get_param("~normal_forward_m", 80.0))
        self.committed_back_points = int(rospy.get_param("~committed_back_points", 2))
        self.max_commit_start_distance_m = float(
            rospy.get_param("~max_commit_start_distance_m", 3.0)
        )
        self.max_commit_heading_error_deg = float(
            rospy.get_param("~max_commit_heading_error_deg", 35.0)
        )

        # First controlled stage intentionally executes bypass only.
        self.allow_lane_change_control = bool(
            rospy.get_param("~allow_lane_change_control", False)
        )

        self.minimum_commit_time_s = float(
            rospy.get_param("~minimum_commit_time_s", 1.0)
        )
        self.departure_detect_d_m = float(
            rospy.get_param("~departure_detect_d_m", 0.60)
        )
        self.return_d_tolerance_m = float(
            rospy.get_param("~return_d_tolerance_m", 0.35)
        )
        self.return_heading_tolerance_deg = float(
            rospy.get_param("~return_heading_tolerance_deg", 12.0)
        )
        self.completion_min_fraction = float(
            rospy.get_param("~completion_min_fraction", 0.50)
        )
        self.completion_end_distance_m = float(
            rospy.get_param("~completion_end_distance_m", 3.0)
        )

        self.vehicle_length_m = float(rospy.get_param("~vehicle_length_m", 4.635))
        self.vehicle_width_m = float(rospy.get_param("~vehicle_width_m", 1.892))
        self.vehicle_center_from_base_m = float(
            rospy.get_param("~vehicle_center_from_base_m", 1.50)
        )
        self.collision_longitudinal_margin_m = float(
            rospy.get_param("~collision_longitudinal_margin_m", 0.25)
        )
        self.collision_lateral_margin_m = float(
            rospy.get_param("~collision_lateral_margin_m", 0.20)
        )
        self.collision_sample_stride = int(
            rospy.get_param("~collision_sample_stride", 1)
        )

        global_points = load_mgeo_path(path_file)
        self.reference = ReferencePath(global_points)

        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_at: Optional[rospy.Time] = None
        self.latest_obstacles: Optional[LidarObstacleArray] = None
        self.latest_obstacles_at: Optional[rospy.Time] = None

        self.selected_points: List[PathPoint] = []
        self.selected_path_at: Optional[rospy.Time] = None
        self.planner_ready = False
        self.avoidance_required = False
        self.safe_path_available = False
        self.selected_kind = ""
        self.selected_side = ""
        self.planner_status_at: Optional[rospy.Time] = None

        self.state = self.STOP_PLANNER
        self.committed_points: List[PathPoint] = []
        self.committed_lengths: List[float] = []
        self.commit_started_at: Optional[rospy.Time] = None
        self.commit_side = ""
        self.has_departed_global = False
        self.max_abs_global_d = 0.0
        self.last_guard_collision_id: Optional[int] = None

        self.active_path_pub = rospy.Publisher("~active_path", RosPath, queue_size=1)
        self.committed_path_pub = rospy.Publisher(
            "~committed_path", RosPath, queue_size=1, latch=True
        )
        self.stop_required_pub = rospy.Publisher(
            "~stop_required", Bool, queue_size=1
        )
        self.state_pub = rospy.Publisher("~state", String, queue_size=1)

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber(
            self.obstacle_topic, LidarObstacleArray, self._obstacle_cb, queue_size=1
        )
        rospy.Subscriber(
            self.selected_path_topic, RosPath, self._selected_path_cb, queue_size=1
        )
        rospy.Subscriber(
            self.planner_ready_topic, Bool, self._planner_ready_cb, queue_size=1
        )
        rospy.Subscriber(
            self.avoidance_required_topic,
            Bool,
            self._avoidance_required_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            self.safe_path_available_topic,
            Bool,
            self._safe_path_available_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            self.selected_kind_topic, String, self._selected_kind_cb, queue_size=1
        )
        rospy.Subscriber(
            self.selected_side_topic, String, self._selected_side_cb, queue_size=1
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self._timer_cb
        )

        rospy.logwarn(
            "Avoidance Path Manager started: BYPASS control only, lane_change_control=%s. "
            "active_path=%s/active_path stop=%s/stop_required",
            self.allow_lane_change_control,
            rospy.get_name(),
            rospy.get_name(),
        )

    def _mark_planner_status(self) -> None:
        self.planner_status_at = rospy.Time.now()

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_odom_at = rospy.Time.now()

    def _obstacle_cb(self, msg: LidarObstacleArray) -> None:
        self.latest_obstacles = msg
        self.latest_obstacles_at = rospy.Time.now()

    def _selected_path_cb(self, msg: RosPath) -> None:
        self.selected_points = _points_from_ros_path(msg)
        self.selected_path_at = rospy.Time.now()

    def _planner_ready_cb(self, msg: Bool) -> None:
        self.planner_ready = bool(msg.data)
        self._mark_planner_status()

    def _avoidance_required_cb(self, msg: Bool) -> None:
        self.avoidance_required = bool(msg.data)
        self._mark_planner_status()

    def _safe_path_available_cb(self, msg: Bool) -> None:
        self.safe_path_available = bool(msg.data)
        self._mark_planner_status()

    def _selected_kind_cb(self, msg: String) -> None:
        self.selected_kind = str(msg.data or "")
        self._mark_planner_status()

    def _selected_side_cb(self, msg: String) -> None:
        self.selected_side = str(msg.data or "")
        self._mark_planner_status()

    def _ros_path(self, points: Sequence[PathPoint]) -> RosPath:
        msg = RosPath()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        for p in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(p.x)
            ps.pose.position.y = float(p.y)
            ps.pose.position.z = float(p.z)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def _normal_path(self, ego_projection) -> List[PathPoint]:
        start_s = max(0.0, ego_projection.s - self.normal_back_m)
        end_s = min(
            self.reference.total_length_m,
            ego_projection.s + self.normal_forward_m,
        )
        start_i = max(
            0,
            bisect.bisect_left(self.reference.cumulative_s, start_s) - 1,
        )
        end_i = min(
            len(self.reference.points),
            bisect.bisect_right(self.reference.cumulative_s, end_s) + 1,
        )
        result = list(self.reference.points[start_i:end_i])
        if len(result) < 2:
            return list(self.reference.points[-2:])
        return result

    def _remaining_committed(self, x: float, y: float) -> Tuple[List[PathPoint], int, float, float]:
        if len(self.committed_points) < 2:
            return [], 0, 0.0, 0.0
        nearest = _nearest_index(self.committed_points, x, y)
        start = max(0, nearest - max(0, self.committed_back_points))
        remaining = list(self.committed_points[start:])
        progress = self.committed_lengths[nearest]
        total = max(self.committed_lengths[-1], 1.0e-6)
        fraction = max(0.0, min(1.0, progress / total))
        remaining_distance = max(0.0, total - progress)
        return remaining, nearest, fraction, remaining_distance

    def _sensors_fresh(self, now: rospy.Time) -> Tuple[bool, dict]:
        odom_age = (
            (now - self.latest_odom_at).to_sec()
            if self.latest_odom_at is not None
            else float("inf")
        )
        obstacle_age = (
            (now - self.latest_obstacles_at).to_sec()
            if self.latest_obstacles_at is not None
            else float("inf")
        )
        planner_age = (
            (now - self.planner_status_at).to_sec()
            if self.planner_status_at is not None
            else float("inf")
        )
        fresh = (
            odom_age <= self.odom_timeout_s
            and obstacle_age <= self.obstacle_timeout_s
            and planner_age <= self.planner_signal_timeout_s
            and self.planner_ready
        )
        return fresh, {
            "odom_age_s": None if not math.isfinite(odom_age) else round(odom_age, 3),
            "obstacle_age_s": None if not math.isfinite(obstacle_age) else round(obstacle_age, 3),
            "planner_age_s": None if not math.isfinite(planner_age) else round(planner_age, 3),
            "planner_ready": self.planner_ready,
        }

    def _selected_path_is_fresh(self, now: rospy.Time) -> bool:
        if self.selected_path_at is None:
            return False
        return (now - self.selected_path_at).to_sec() <= self.selected_path_timeout_s

    def _kind_is_allowed(self) -> bool:
        if self.selected_kind == "bypass":
            return True
        if self.selected_kind == "lane_change" and self.allow_lane_change_control:
            return True
        return False

    def _commit_selected(self, now: rospy.Time, pose) -> bool:
        if not self._selected_path_is_fresh(now) or len(self.selected_points) < 3:
            return False
        if not self._kind_is_allowed():
            return False

        x = float(pose.position.x)
        y = float(pose.position.y)
        start_distance = math.hypot(
            self.selected_points[0].x - x,
            self.selected_points[0].y - y,
        )
        if start_distance > self.max_commit_start_distance_m:
            rospy.logwarn_throttle(
                1.0,
                "Selected path rejected: start is %.2fm from ego (limit %.2fm)",
                start_distance,
                self.max_commit_start_distance_m,
            )
            return False

        ego_yaw = _yaw_from_quaternion(pose.orientation)
        path_yaw = _path_heading(self.selected_points, 0)
        heading_error_deg = abs(math.degrees(_normalize_angle(path_yaw - ego_yaw)))
        if heading_error_deg > self.max_commit_heading_error_deg:
            rospy.logwarn_throttle(
                1.0,
                "Selected path rejected: initial heading error %.1fdeg (limit %.1fdeg)",
                heading_error_deg,
                self.max_commit_heading_error_deg,
            )
            return False

        self.committed_points = list(self.selected_points)
        self.committed_lengths = _path_lengths(self.committed_points)
        self.commit_started_at = now
        self.commit_side = self.selected_side
        self.has_departed_global = False
        self.max_abs_global_d = 0.0
        self.last_guard_collision_id = None
        self.state = self.AVOIDING
        self.committed_path_pub.publish(self._ros_path(self.committed_points))
        rospy.logwarn(
            "BYPASS COMMITTED side=%s points=%d length=%.1fm",
            self.commit_side,
            len(self.committed_points),
            self.committed_lengths[-1],
        )
        return True

    def _clear_commit(self) -> None:
        self.committed_points = []
        self.committed_lengths = []
        self.commit_started_at = None
        self.commit_side = ""
        self.has_departed_global = False
        self.max_abs_global_d = 0.0
        self.last_guard_collision_id = None
        self.committed_path_pub.publish(self._ros_path([]))

    def _obstacle_boxes(self) -> List[ObstacleBox]:
        result: List[ObstacleBox] = []
        if self.latest_obstacles is None:
            return result
        for obs in self.latest_obstacles.obstacles:
            result.append(
                ObstacleBox(
                    obstacle_id=int(obs.id),
                    center_x=float(obs.center_x_map),
                    center_y=float(obs.center_y_map),
                    yaw=float(obs.yaw),
                    length=max(0.10, float(obs.length)),
                    width=max(0.10, float(obs.width)),
                )
            )
        return result

    def _committed_guard(self, remaining: Sequence[PathPoint]):
        return check_path_collision(
            path=remaining,
            obstacles=self._obstacle_boxes(),
            vehicle_length_m=self.vehicle_length_m,
            vehicle_width_m=self.vehicle_width_m,
            vehicle_center_from_base_m=self.vehicle_center_from_base_m,
            collision_longitudinal_margin_m=self.collision_longitudinal_margin_m,
            collision_lateral_margin_m=self.collision_lateral_margin_m,
            collision_sample_stride=self.collision_sample_stride,
        )

    def _timer_cb(self, _event) -> None:
        now = rospy.Time.now()
        if self.latest_odom is None:
            self.stop_required_pub.publish(Bool(data=True))
            return

        pose = self.latest_odom.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        ego_yaw = _yaw_from_quaternion(pose.orientation)
        ego_projection = self.reference.project(x, y)
        global_heading = math.atan2(
            ego_projection.tangent_y, ego_projection.tangent_x
        )
        global_heading_error_deg = abs(
            math.degrees(_normalize_angle(ego_yaw - global_heading))
        )

        sensors_fresh, freshness = self._sensors_fresh(now)
        stop_required = False
        active_source = "global"
        remaining_fraction = None
        remaining_distance = None
        guard_collision_id = None

        if self.committed_points:
            remaining, _nearest, fraction, rem_dist = self._remaining_committed(x, y)
            remaining_fraction = fraction
            remaining_distance = rem_dist
            active_source = "committed_bypass"

            abs_d = abs(ego_projection.d)
            self.max_abs_global_d = max(self.max_abs_global_d, abs_d)
            if abs_d >= self.departure_detect_d_m:
                self.has_departed_global = True

            if not sensors_fresh:
                self.state = self.AVOIDING_SENSOR_STOP
                stop_required = True
            else:
                collision_id, _collision_index = self._committed_guard(remaining)
                guard_collision_id = collision_id
                self.last_guard_collision_id = collision_id
                if collision_id is not None:
                    self.state = self.AVOIDING_BLOCKED
                    stop_required = True
                else:
                    self.state = self.AVOIDING

            commit_age = (
                (now - self.commit_started_at).to_sec()
                if self.commit_started_at is not None
                else 0.0
            )
            returned_to_global = (
                self.has_departed_global
                and abs(ego_projection.d) <= self.return_d_tolerance_m
                and global_heading_error_deg <= self.return_heading_tolerance_deg
                and fraction >= self.completion_min_fraction
                and commit_age >= self.minimum_commit_time_s
            )
            near_committed_end = (
                rem_dist <= self.completion_end_distance_m
                and abs(ego_projection.d) <= max(self.return_d_tolerance_m, 0.60)
                and global_heading_error_deg <= max(
                    self.return_heading_tolerance_deg, 18.0
                )
            )

            if (returned_to_global or near_committed_end) and not stop_required:
                rospy.logwarn(
                    "BYPASS COMPLETE: returned to global (d=%+.2f heading_err=%.1fdeg progress=%.0f%%)",
                    ego_projection.d,
                    global_heading_error_deg,
                    100.0 * fraction,
                )
                self._clear_commit()
                self.state = self.NORMAL
                active_source = "global"
                remaining = self._normal_path(ego_projection)

            self.active_path_pub.publish(self._ros_path(remaining))

        else:
            normal_points = self._normal_path(ego_projection)
            self.active_path_pub.publish(self._ros_path(normal_points))

            if not sensors_fresh:
                self.state = self.STOP_PLANNER
                stop_required = True
            elif self.avoidance_required:
                if self.safe_path_available and self._kind_is_allowed():
                    if self._commit_selected(now, pose):
                        remaining, _nearest, fraction, rem_dist = self._remaining_committed(x, y)
                        remaining_fraction = fraction
                        remaining_distance = rem_dist
                        active_source = "committed_bypass"
                        self.active_path_pub.publish(self._ros_path(remaining))
                        stop_required = False
                    else:
                        self.state = self.STOP_NO_SAFE
                        stop_required = True
                elif self.safe_path_available and not self._kind_is_allowed():
                    self.state = self.STOP_UNSUPPORTED
                    stop_required = True
                else:
                    self.state = self.STOP_NO_SAFE
                    stop_required = True
            else:
                self.state = self.NORMAL
                stop_required = False

        self.stop_required_pub.publish(Bool(data=stop_required))

        payload = {
            "state": self.state,
            "stop_required": stop_required,
            "active_source": active_source,
            "planner": {
                "avoidance_required": self.avoidance_required,
                "safe_path_available": self.safe_path_available,
                "selected_kind": self.selected_kind,
                "selected_side": self.selected_side,
            },
            "freshness": freshness,
            "ego": {
                "global_d_m": round(ego_projection.d, 3),
                "global_heading_error_deg": round(global_heading_error_deg, 2),
            },
            "commit": {
                "active": bool(self.committed_points),
                "side": self.commit_side,
                "departed": self.has_departed_global,
                "max_abs_global_d_m": round(self.max_abs_global_d, 3),
                "progress_fraction": (
                    round(remaining_fraction, 3)
                    if remaining_fraction is not None
                    else None
                ),
                "remaining_distance_m": (
                    round(remaining_distance, 2)
                    if remaining_distance is not None
                    else None
                ),
                "guard_collision_id": guard_collision_id,
            },
        }
        self.state_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

        rospy.loginfo_throttle(
            1.0,
            "PathManager state=%s stop=%s source=%s d=%+.2f planner(need=%s safe=%s kind=%s)",
            self.state,
            stop_required,
            active_source,
            ego_projection.d,
            self.avoidance_required,
            self.safe_path_available,
            self.selected_kind or "-",
        )


def main() -> None:
    try:
        AvoidancePathManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
