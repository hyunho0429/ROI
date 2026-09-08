#!/usr/bin/env python3
"""MORAI ROS1 Pure Pursuit + Frenet obstacle-avoidance FSM.

Only this node may publish the final ``/ctrl_cmd`` while control is enabled.
Run the existing ``purepursuit_mgeo`` node with ``enable_control:=false``.
"""

from __future__ import annotations

import json
import math
import threading
from typing import List, Optional, Sequence, Tuple

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float64, String

from purepursuit_mgeo.frenet_planner import (
    FrenetCandidate,
    FrenetPlanner,
    ReferencePath,
    TrackedObstacle,
)
from purepursuit_mgeo.path import MgeoPurePursuit, PathPoint, load_mgeo_path


NORMAL = "NORMAL"
AVOIDANCE = "AVOIDANCE"
RETURN = "RETURN"
STOP = "STOP"


def quaternion_to_yaw(x_value: float, y_value: float, z_value: float, w_value: float) -> float:
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


def yaw_to_quaternion_z_w(yaw: float) -> Tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def float_list_parameter(name: str, default: Sequence[float]) -> List[float]:
    value = rospy.get_param("~" + name, list(default))
    if isinstance(value, str):
        value = value.strip().strip("[]")
        value = [] if not value else [part.strip() for part in value.split(",")]
    result = [float(item) for item in value]
    if not result:
        raise ValueError("{} must contain at least one number".format(name))
    return result


class FrenetFsmNode:
    def __init__(self) -> None:
        rospy.init_node("frenet_fsm_controller", anonymous=False)
        self.lock = threading.RLock()

        path_file = rospy.get_param("~path_file")
        self.global_points = load_mgeo_path(path_file)
        self.reference = ReferencePath(self.global_points)

        self.enable_control = bool(rospy.get_param("~enable_control", False))
        self.command_topic = rospy.get_param("~command_topic", "/ctrl_cmd")
        self.odom_topic = rospy.get_param("~odometry_topic", "/localization/odometry")
        self.obstacle_topic = rospy.get_param(
            "~obstacle_topic", "/detection/obstacle_states"
        )
        self.pedestrian_stop_topic = rospy.get_param(
            "~pedestrian_stop_topic",
            "/perception/pedestrian_crossing/stop_required",
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.longitudinal_command_type = int(rospy.get_param("~longl_cmd_type", 2))

        self.normal_speed = float(rospy.get_param("~normal_speed_mps", 6.0))
        self.avoidance_speed = float(rospy.get_param("~avoidance_speed_mps", 3.0))
        self.return_speed = float(rospy.get_param("~return_speed_mps", 3.0))
        self.control_rate = float(rospy.get_param("~control_rate_hz", 20.0))
        self.planning_rate = float(rospy.get_param("~planning_rate_hz", 5.0))
        self.max_steering = float(
            rospy.get_param("~max_steering_rad", math.radians(40.0))
        )
        self.steering_sign = float(rospy.get_param("~steering_sign", 1.0))
        self.wheelbase = float(rospy.get_param("~wheelbase_m", 3.0))
        self.lookahead_min = float(rospy.get_param("~lookahead_min_m", 4.0))
        self.lookahead_gain = float(rospy.get_param("~lookahead_gain", 0.35))
        self.return_lookahead_min = float(
            rospy.get_param("~return_lookahead_min_m", 8.0)
        )
        self.return_lookahead_gain = float(
            rospy.get_param("~return_lookahead_gain", 0.50)
        )
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance_m", 1.5))

        self.vehicle_length = float(rospy.get_param("~vehicle_length_m", 4.635))
        self.vehicle_width = float(rospy.get_param("~vehicle_width_m", 1.892))
        self.rear_axle_to_center = float(
            rospy.get_param("~rear_axle_to_center_m", 1.35)
        )
        self.lane_width = float(rospy.get_param("~lane_width_m", 3.5))
        self.safety_margin = float(rospy.get_param("~safety_margin_m", 0.45))
        self.trigger_distance = float(
            rospy.get_param("~obstacle_trigger_distance_m", 30.0)
        )
        self.rear_ignore_distance = float(
            rospy.get_param("~obstacle_rear_distance_m", 3.0)
        )
        self.return_tolerance = float(rospy.get_param("~return_tolerance_m", 0.30))
        self.clear_hold_seconds = float(rospy.get_param("~clear_hold_s", 1.5))
        self.odom_stale_timeout = float(rospy.get_param("~odom_stale_timeout_s", 0.5))
        self.obstacle_stale_timeout = float(
            rospy.get_param("~obstacle_stale_timeout_s", 0.6)
        )
        self.require_obstacle_topic = bool(
            rospy.get_param("~require_obstacle_topic", True)
        )
        self.preferred_side = str(
            rospy.get_param("~preferred_avoidance_side", "left")
        )

        self.avoidance_offsets = float_list_parameter(
            "avoidance_offsets_m",
            [
                1.0, -1.0,
                2.0, -2.0,
                3.0, -3.0,
                4.0, -4.0,
                5.0, -5.0,
                6.0, -6.0,
                7.0, -7.0,
            ],
        )
        self.planning_times = float_list_parameter(
            "planning_times_s", [2.0, 3.0, 4.0]
        )

        self.global_controller = self._new_controller(self.global_points)
        # Reference reacquisition uses Pure Pursuit directly.  Its longer
        # lookahead prevents the sharp steering that would result from aiming
        # at the geometrically nearest point itself.
        self.return_controller = self._new_controller(
            self.global_points,
            self.return_lookahead_min,
            self.return_lookahead_gain,
        )
        self.planner = FrenetPlanner(
            self.reference,
            vehicle_length=self.vehicle_length,
            vehicle_width=self.vehicle_width,
            rear_axle_to_center=self.rear_axle_to_center,
            safety_margin=self.safety_margin,
            preferred_side=self.preferred_side,
        )

        self.latest_odom: Optional[Odometry] = None
        self.odom_received_at = 0.0
        self.obstacles: List[TrackedObstacle] = []
        self.obstacles_received_at = 0.0
        self.obstacles_source_timestamp = 0.0
        self.pedestrian_stop_required = False

        self.state = NORMAL
        self.state_reason = "startup"
        self.active_candidate: Optional[FrenetCandidate] = None
        self.local_controller: Optional[MgeoPurePursuit] = None
        self.active_avoidance_offset: Optional[float] = None
        # RETURN is meaningful only after the vehicle actually committed to
        # an avoidance path.  A startup/stale-input STOP must resume NORMAL
        # even when GPS/reference-path error is larger than the tight return
        # tolerance.
        self.return_required = False
        self.last_plan_at = 0.0
        self.clear_since: Optional[float] = None

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber(
            self.obstacle_topic, String, self._obstacle_callback, queue_size=1
        )
        rospy.Subscriber(
            self.pedestrian_stop_topic,
            Bool,
            self._pedestrian_stop_callback,
            queue_size=1,
        )
        self.command_publisher = rospy.Publisher(
            self.command_topic, CtrlCmd, queue_size=1
        )
        self.state_publisher = rospy.Publisher(
            "/control/frenet_fsm/state", String, queue_size=1, latch=True
        )
        self.path_publisher = rospy.Publisher(
            "/control/frenet_fsm/local_path", Path, queue_size=1, latch=True
        )
        self.lookahead_publisher = rospy.Publisher(
            "/control/frenet_fsm/lookahead_point", PointStamped, queue_size=1
        )
        self.steering_publisher = rospy.Publisher(
            "/control/frenet_fsm/steering_preview", Float64, queue_size=1
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.control_rate, 1.0)),
            self._control_callback,
        )

        rospy.logwarn(
            "Frenet FSM ready: control=%s path_points=%d obstacle=%s cmd=%s "
            "offsets=%s. The old Pure Pursuit publisher MUST use enable_control=false.",
            self.enable_control,
            len(self.global_points),
            self.obstacle_topic,
            self.command_topic,
            self.avoidance_offsets,
        )

    def _new_controller(
        self,
        points: Sequence[PathPoint],
        lookahead_min: Optional[float] = None,
        lookahead_gain: Optional[float] = None,
    ) -> MgeoPurePursuit:
        return MgeoPurePursuit(
            points,
            self.wheelbase,
            self.lookahead_min if lookahead_min is None else lookahead_min,
            self.lookahead_gain if lookahead_gain is None else lookahead_gain,
            self.goal_tolerance,
            self.steering_sign,
        )

    def _odom_callback(self, message: Odometry) -> None:
        with self.lock:
            self.latest_odom = message
            self.odom_received_at = rospy.Time.now().to_sec()

    def _obstacle_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                raw_obstacles = payload.get("obstacles", [])
                source_timestamp = float(payload.get("timestamp", 0.0))
            elif isinstance(payload, list):
                raw_obstacles = payload
                source_timestamp = 0.0
            else:
                raise ValueError("top-level JSON must be an object or list")
            parsed = [TrackedObstacle.from_dict(item) for item in raw_obstacles]
        except (TypeError, ValueError, KeyError, json.JSONDecodeError) as error:
            rospy.logerr_throttle(1.0, "Invalid obstacle state JSON: %s", error)
            return
        with self.lock:
            self.obstacles = parsed
            self.obstacles_received_at = rospy.Time.now().to_sec()
            self.obstacles_source_timestamp = source_timestamp

    def _pedestrian_stop_callback(self, message: Bool) -> None:
        with self.lock:
            self.pedestrian_stop_required = bool(message.data)

    def _snapshot(self):
        with self.lock:
            return (
                self.latest_odom,
                self.odom_received_at,
                list(self.obstacles),
                self.obstacles_received_at,
                self.obstacles_source_timestamp,
                self.pedestrian_stop_required,
            )

    @staticmethod
    def _vehicle_state(message: Odometry) -> Tuple[float, float, float, float]:
        pose = message.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        speed = math.hypot(
            message.twist.twist.linear.x, message.twist.twist.linear.y
        )
        return pose.position.x, pose.position.y, yaw, speed

    def _transition(self, state: str, reason: str) -> None:
        if state != self.state:
            rospy.logwarn("Frenet FSM: %s -> %s (%s)", self.state, state, reason)
        self.state = state
        self.state_reason = reason
        if state == NORMAL:
            self.active_candidate = None
            self.local_controller = None
            self.active_avoidance_offset = None
            self.return_required = False
            self.clear_since = None

    def _obstacle_age(self, now: float, source_timestamp: float) -> float:
        if source_timestamp <= 0.0:
            return 0.0
        return max(0.0, min(now - source_timestamp, self.obstacle_stale_timeout))

    def _planning_due(self, now: float) -> bool:
        return now - self.last_plan_at >= 1.0 / max(self.planning_rate, 0.1)

    def _plan(
        self,
        now: float,
        vehicle: Tuple[float, float, float, float],
        obstacles: Sequence[TrackedObstacle],
        obstacle_age: float,
        target_offsets: Sequence[float],
        target_speed: float,
        preserve_on_failure: bool = False,
    ) -> bool:
        x_value, y_value, yaw, speed = vehicle
        self.last_plan_at = now
        candidate = self.planner.plan(
            x_value,
            y_value,
            yaw,
            speed,
            target_speed,
            obstacles,
            obstacle_age,
            target_offsets,
            self.planning_times,
        )
        if candidate is None:
            if not preserve_on_failure:
                self.active_candidate = None
                self.local_controller = None
            return False
        self.active_candidate = candidate
        self.local_controller = self._new_controller(candidate.points)
        self._publish_candidate_path(candidate)
        rospy.loginfo(
            "Frenet selected: state=%s d=%.2f T=%.2f cost=%.3f clearance=%.2f",
            self.state,
            candidate.target_offset,
            candidate.planning_time,
            candidate.total_cost,
            candidate.minimum_clearance,
        )
        return True

    def _publish_candidate_path(self, candidate: FrenetCandidate) -> None:
        message = Path()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.map_frame
        for point, yaw in zip(candidate.points, candidate.yaws):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z
            pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion_z_w(yaw)
            message.poses.append(pose)
        self.path_publisher.publish(message)

    def _publish_stop(self, reason: str, blocker_count: int = 0) -> None:
        if self.enable_control:
            self.command_publisher.publish(self._make_command(0.0, 0.0, True))
        self._publish_state(reason, blocker_count, 0.0)

    def _follow(
        self,
        controller: MgeoPurePursuit,
        vehicle: Tuple[float, float, float, float],
        desired_speed: float,
        blocker_count: int,
    ) -> bool:
        x_value, y_value, yaw, speed = vehicle
        steering, path_stop, target, target_index, lookahead = controller.compute(
            x_value, y_value, yaw, speed
        )
        steering = max(-self.max_steering, min(self.max_steering, steering))
        target_message = PointStamped()
        target_message.header.stamp = rospy.Time.now()
        target_message.header.frame_id = self.map_frame
        target_message.point.x = target.x
        target_message.point.y = target.y
        target_message.point.z = target.z
        self.lookahead_publisher.publish(target_message)
        self.steering_publisher.publish(Float64(steering))
        if self.enable_control:
            self.command_publisher.publish(
                self._make_command(steering, desired_speed, path_stop)
            )
        self._publish_state(
            "local path ended" if path_stop else self.state_reason,
            blocker_count,
            steering,
            target_index,
            lookahead,
        )
        return path_stop

    def _make_command(self, steering: float, speed: float, stop: bool) -> CtrlCmd:
        message = CtrlCmd()
        if hasattr(message, "longlCmdType"):
            message.longlCmdType = self.longitudinal_command_type
        if hasattr(message, "steering"):
            message.steering = 0.0 if stop else steering
        if hasattr(message, "brake"):
            message.brake = 1.0 if stop else 0.0
        if hasattr(message, "accel"):
            message.accel = 0.0
        if hasattr(message, "acceleration"):
            message.acceleration = 0.0
        if hasattr(message, "velocity"):
            message.velocity = 0.0 if stop else max(0.0, speed)
        return message

    def _publish_state(
        self,
        reason: str,
        blocker_count: int,
        steering: float,
        target_index: int = -1,
        lookahead: float = 0.0,
    ) -> None:
        candidate = self.active_candidate
        payload = {
            "state": self.state,
            "reason": reason,
            "control_enabled": self.enable_control,
            "blocker_count": blocker_count,
            "steering_rad": steering,
            "target_index": target_index,
            "lookahead_m": lookahead,
            "avoidance_offset_m": self.active_avoidance_offset,
            "candidate_cost": None if candidate is None else candidate.total_cost,
            "candidate_clearance_m": (
                None
                if candidate is None or not math.isfinite(candidate.minimum_clearance)
                else candidate.minimum_clearance
            ),
        }
        self.state_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

    def _enter_avoidance(
        self,
        now: float,
        vehicle: Tuple[float, float, float, float],
        obstacles: Sequence[TrackedObstacle],
        blocking_obstacles: Sequence[TrackedObstacle],
        obstacle_age: float,
    ) -> bool:
        self._transition(AVOIDANCE, "global-lane obstacle detected")
        target_offsets = self.planner.clearance_target_offsets(
            blocking_obstacles,
            obstacle_age,
            self.avoidance_offsets,
        )
        if not target_offsets:
            self._transition(STOP, "no configured offset clears obstacle BBox")
            return False
        if not self._plan(
            now,
            vehicle,
            obstacles,
            obstacle_age,
            target_offsets,
            self.avoidance_speed,
        ):
            self._transition(STOP, "no collision-free avoidance candidate")
            return False
        self.active_avoidance_offset = self.active_candidate.target_offset
        self.return_required = True
        self.clear_since = None
        return True

    def _control_callback(self, _event: rospy.timer.TimerEvent) -> None:
        now = rospy.Time.now().to_sec()
        (
            odometry,
            odometry_received_at,
            obstacles,
            obstacles_received_at,
            source_timestamp,
            pedestrian_stop,
        ) = self._snapshot()

        if odometry is None or now - odometry_received_at > self.odom_stale_timeout:
            self._transition(STOP, "odometry missing or stale")
            self._publish_stop(self.state_reason)
            return
        if self.require_obstacle_topic and (
            obstacles_received_at <= 0.0
            or now - obstacles_received_at > self.obstacle_stale_timeout
        ):
            self._transition(STOP, "obstacle state topic missing or stale")
            self._publish_stop(self.state_reason)
            return
        if pedestrian_stop:
            self._publish_stop("pedestrian stop requested")
            return

        vehicle = self._vehicle_state(odometry)
        x_value, y_value, _yaw, speed = vehicle
        ego_projection = self.reference.project(x_value, y_value)
        obstacle_age = self._obstacle_age(now, source_timestamp)
        blockers = self.planner.relevant_obstacles(
            obstacles,
            ego_projection.s,
            speed,
            self.vehicle_width,
            self.trigger_distance,
            self.rear_ignore_distance,
            obstacle_age,
        )

        if self.state == NORMAL:
            if blockers:
                if not self._enter_avoidance(
                    now, vehicle, obstacles, blockers, obstacle_age
                ):
                    self._publish_stop(self.state_reason, len(blockers))
                    return
                self._follow(
                    self.local_controller, vehicle, self.avoidance_speed, len(blockers)
                )
                return
            self.state_reason = "global path tracking"
            self._follow(
                self.global_controller, vehicle, self.normal_speed, len(blockers)
            )
            return

        if self.state == AVOIDANCE:
            if blockers:
                self.clear_since = None
            elif self.clear_since is None:
                self.clear_since = now
            elif now - self.clear_since >= self.clear_hold_seconds:
                # The obstacle is completely behind the configured rear
                # distance.  Reacquire the MGeo reference with a long-lookahead
                # Pure Pursuit controller instead of generating a second
                # Frenet trajectory.
                self.active_candidate = None
                self.local_controller = None
                self.active_avoidance_offset = None
                self._transition(RETURN, "obstacle passed; Pure Pursuit return")

            if self.state == AVOIDANCE:
                if self._planning_due(now):
                    preferred = (
                        [self.active_avoidance_offset]
                        if self.active_avoidance_offset is not None
                        else self.avoidance_offsets
                    )
                    if not self._plan(
                        now,
                        vehicle,
                        obstacles,
                        obstacle_age,
                        preferred,
                        self.avoidance_speed,
                    ):
                        fallback_offsets = self.planner.clearance_target_offsets(
                            blockers,
                            obstacle_age,
                            self.avoidance_offsets,
                        )
                        if not fallback_offsets or not self._plan(
                            now,
                            vehicle,
                            obstacles,
                            obstacle_age,
                            fallback_offsets,
                            self.avoidance_speed,
                        ):
                            self._transition(STOP, "avoidance replanning failed")
                            self._publish_stop(self.state_reason, len(blockers))
                            return
                        self.active_avoidance_offset = self.active_candidate.target_offset
                if self.local_controller is None:
                    self._transition(STOP, "avoidance controller unavailable")
                    self._publish_stop(self.state_reason, len(blockers))
                    return
                self.state_reason = "following collision-free avoidance path"
                self._follow(
                    self.local_controller, vehicle, self.avoidance_speed, len(blockers)
                )
                return

        if self.state == RETURN:
            if blockers:
                if not self._enter_avoidance(
                    now, vehicle, obstacles, blockers, obstacle_age
                ):
                    self._publish_stop(self.state_reason, len(blockers))
                    return
                self._follow(
                    self.local_controller, vehicle, self.avoidance_speed, len(blockers)
                )
                return
            if abs(ego_projection.d) <= self.return_tolerance:
                self._transition(NORMAL, "reference path reacquired")
                self._follow(
                    self.global_controller, vehicle, self.normal_speed, len(blockers)
                )
                return
            self.state_reason = "Pure Pursuit return to global reference"
            self._follow(
                self.return_controller, vehicle, self.return_speed, len(blockers)
            )
            return

        # STOP retries planning whenever both required inputs are healthy.
        if blockers:
            if self._planning_due(now) and self._enter_avoidance(
                now, vehicle, obstacles, blockers, obstacle_age
            ):
                self._follow(
                    self.local_controller, vehicle, self.avoidance_speed, len(blockers)
                )
                return
        elif self.return_required and abs(ego_projection.d) > self.return_tolerance:
            self._transition(RETURN, "inputs recovered; Pure Pursuit return")
            self._follow(
                self.return_controller, vehicle, self.return_speed, len(blockers)
            )
            return
        else:
            self._transition(NORMAL, "inputs recovered; resume global path")
            self._follow(
                self.global_controller, vehicle, self.normal_speed, len(blockers)
            )
            return
        self._publish_stop(self.state_reason, len(blockers))


if __name__ == "__main__":
    try:
        FrenetFsmNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
