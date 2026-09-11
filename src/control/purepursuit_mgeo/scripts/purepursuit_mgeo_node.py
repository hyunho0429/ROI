#!/usr/bin/env python3
"""Sensor-team Pure Pursuit + managed path + merge-gate integration.

Preserves the sensor team's steering geometry and longlCmdType=1 pedal control.
Adds only:
* Path Manager active-path input with stale fail-safe;
* Path Manager avoidance stop OR;
* roundabout/generic merge stop OR;
* optional scenario-specific suppression of the generic intersection stop only
  after the merge gate has explicitly committed GO.
"""

from __future__ import annotations

import math
import threading
from typing import List, Optional

import rospy
from geometry_msgs.msg import PointStamped
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry, Path as RosPath
from path_planning.longitudinal_controller import PedalSpeedController
from std_msgs.msg import Bool, Float64

from purepursuit_mgeo.path import MgeoPurePursuit, PathPoint, load_mgeo_path


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class PurePursuitNode:
    def __init__(self) -> None:
        rospy.init_node("purepursuit_mgeo", anonymous=False)

        path_file = rospy.get_param("~path_file")
        self.points = load_mgeo_path(path_file)
        self.target_speed = float(rospy.get_param("~target_speed_mps", 6.0))
        if self.target_speed < 0.0:
            raise ValueError("target_speed_mps must be zero or positive")
        self.max_steering = float(rospy.get_param("~max_steering_rad", math.radians(40.0)))
        self.rate_hz = float(rospy.get_param("~control_rate_hz", 20.0))
        self.enable_control = bool(rospy.get_param("~enable_control", False))
        self.longl_cmd_type = int(rospy.get_param("~longl_cmd_type", 1))
        self.steering_sign = float(rospy.get_param("~steering_sign", 1.0))
        if self.longl_cmd_type != 1:
            raise ValueError("competition rules require longl_cmd_type=1")

        self.speed_controller = PedalSpeedController(
            kp=float(rospy.get_param("~speed_kp", 0.075)),
            ki=float(rospy.get_param("~speed_ki", 0.0001)),
            kd=float(rospy.get_param("~speed_kd", 0.025)),
            nominal_dt=1.0 / max(self.rate_hz, 1.0),
            max_accel=float(rospy.get_param("~max_accel_pedal", 1.0)),
            max_brake=float(rospy.get_param("~max_brake_pedal", 1.0)),
        )

        wheelbase = float(rospy.get_param("~wheelbase_m", 3.0))
        lookahead_min = float(rospy.get_param("~lookahead_min_m", 4.0))
        lookahead_gain = float(rospy.get_param("~lookahead_gain", 0.35))
        goal_tolerance = float(rospy.get_param("~goal_tolerance_m", 1.5))
        self.controller = MgeoPurePursuit(
            self.points,
            wheelbase,
            lookahead_min,
            lookahead_gain,
            goal_tolerance,
            self.steering_sign,
        )
        self.controller_lock = threading.RLock()

        self.pose_topic = rospy.get_param("~pose_topic", "/localization/odometry")
        self.command_topic = rospy.get_param("~command_topic", "/ctrl_cmd")
        self.lookahead_topic = rospy.get_param("~lookahead_topic", "/control/lookahead_point")
        self.pedestrian_stop_topic = rospy.get_param(
            "~pedestrian_stop_topic", "/perception/pedestrian_crossing/stop_required"
        )
        self.traffic_light_stop_topic = rospy.get_param(
            "~traffic_light_stop_topic", "/perception/traffic_light/stop_required"
        )
        self.intersection_stop_topic = rospy.get_param(
            "~intersection_stop_topic", "/perception/intersection/driving_unavailable"
        )
        self.map_frame = rospy.get_param("~map_frame", "map")

        # Avoidance Path Manager integration.
        self.use_active_path = bool(rospy.get_param("~use_active_path", False))
        self.active_path_topic = rospy.get_param(
            "~active_path_topic", "/avoidance_path_manager/active_path"
        )
        self.require_path_manager_status = bool(
            rospy.get_param("~require_path_manager_status", False)
        )
        self.stop_required_topic = rospy.get_param(
            "~stop_required_topic", "/avoidance_path_manager/stop_required"
        )
        self.managed_timeout_s = float(rospy.get_param("~managed_timeout_s", 2.0))

        # Optional dynamic target-speed from the final driving supervisor.
        self.use_target_speed_override = bool(rospy.get_param("~use_target_speed_override", False))
        self.target_speed_override_topic = rospy.get_param(
            "~target_speed_override_topic", "/highway_lane_strategy/target_speed_mps"
        )
        self.target_speed_override_timeout_s = float(
            rospy.get_param("~target_speed_override_timeout_s", 0.8)
        )
        self.target_speed_override = self.target_speed
        self.target_speed_override_at: Optional[rospy.Time] = None

        # Roundabout / generic yield merge gate.
        self.enable_merge_gate = bool(rospy.get_param("~enable_merge_gate", False))
        self.merge_stop_topic = rospy.get_param(
            "~merge_stop_topic", "/roundabout_merge_gate/stop_required"
        )
        self.merge_allowed_topic = rospy.get_param(
            "~merge_allowed_topic", "/roundabout_merge_gate/allowed"
        )
        self.merge_request_topic = rospy.get_param(
            "~merge_request_topic", "/planning/merge_request"
        )
        self.merge_gate_timeout_s = float(rospy.get_param("~merge_gate_timeout_s", 0.8))
        self.roundabout_override_intersection_stop = bool(
            rospy.get_param("~roundabout_override_intersection_stop", False)
        )

        self.latest_odom: Optional[Odometry] = None
        self.pedestrian_stop_required = False
        self.traffic_light_stop_required = False
        self.intersection_stop_required = False

        self.active_path_received = False
        self.active_path_at: Optional[rospy.Time] = None
        self.path_manager_stop = True if self.require_path_manager_status else False
        self.path_manager_status_at: Optional[rospy.Time] = None

        self.merge_requested = False
        self.merge_stop_required = False
        self.merge_allowed = False
        self.merge_gate_at: Optional[rospy.Time] = None
        self.merge_request_at: Optional[rospy.Time] = None

        rospy.Subscriber(self.pose_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(self.pedestrian_stop_topic, Bool, self.pedestrian_stop_callback, queue_size=1)
        rospy.Subscriber(self.traffic_light_stop_topic, Bool, self.traffic_light_stop_callback, queue_size=1)
        rospy.Subscriber(self.intersection_stop_topic, Bool, self.intersection_stop_callback, queue_size=1)
        if self.use_active_path:
            rospy.Subscriber(self.active_path_topic, RosPath, self.active_path_callback, queue_size=1)
        if self.require_path_manager_status:
            rospy.Subscriber(self.stop_required_topic, Bool, self.stop_required_callback, queue_size=1)
        if self.use_target_speed_override:
            rospy.Subscriber(
                self.target_speed_override_topic, Float64, self.target_speed_override_callback, queue_size=1
            )
        if self.enable_merge_gate:
            rospy.Subscriber(self.merge_stop_topic, Bool, self.merge_stop_callback, queue_size=1)
            rospy.Subscriber(self.merge_allowed_topic, Bool, self.merge_allowed_callback, queue_size=1)
            rospy.Subscriber(self.merge_request_topic, Bool, self.merge_request_callback, queue_size=1)

        self.command_pub = rospy.Publisher(self.command_topic, CtrlCmd, queue_size=1)
        self.lookahead_pub = rospy.Publisher(self.lookahead_topic, PointStamped, queue_size=1)
        self.steering_preview_pub = rospy.Publisher("/control/steering_preview", Float64, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.control_callback
        )

        rospy.logwarn(
            "Pure Pursuit FINAL control=%s speed=%.2fm/s path_points=%d managed_path=%s managed_stop=%s "
            "merge_gate=%s roundabout_intersection_override=%s",
            self.enable_control,
            self.target_speed,
            len(self.points),
            self.use_active_path,
            self.require_path_manager_status,
            self.enable_merge_gate,
            self.roundabout_override_intersection_stop,
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def pedestrian_stop_callback(self, msg: Bool) -> None:
        self.pedestrian_stop_required = bool(msg.data)

    def traffic_light_stop_callback(self, msg: Bool) -> None:
        self.traffic_light_stop_required = bool(msg.data)

    def intersection_stop_callback(self, msg: Bool) -> None:
        self.intersection_stop_required = bool(msg.data)

    def active_path_callback(self, msg: RosPath) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn_throttle(
                2.0,
                "active_path frame=%s ignored (expected %s)",
                msg.header.frame_id,
                self.map_frame,
            )
            return
        points: List[PathPoint] = [
            PathPoint(
                float(ps.pose.position.x),
                float(ps.pose.position.y),
                float(ps.pose.position.z),
            )
            for ps in msg.poses
        ]
        if len(points) < 2:
            rospy.logwarn_throttle(2.0, "active_path has fewer than 2 points")
            return
        with self.controller_lock:
            self.points = points
            self.controller.points = points
        self.active_path_received = True
        self.active_path_at = rospy.Time.now()

    def stop_required_callback(self, msg: Bool) -> None:
        self.path_manager_stop = bool(msg.data)
        self.path_manager_status_at = rospy.Time.now()

    def target_speed_override_callback(self, msg: Float64) -> None:
        value = max(0.0, float(msg.data))
        self.target_speed_override = value
        self.target_speed_override_at = rospy.Time.now()

    def merge_stop_callback(self, msg: Bool) -> None:
        self.merge_stop_required = bool(msg.data)
        self.merge_gate_at = rospy.Time.now()

    def merge_allowed_callback(self, msg: Bool) -> None:
        self.merge_allowed = bool(msg.data)
        self.merge_gate_at = rospy.Time.now()

    def merge_request_callback(self, msg: Bool) -> None:
        self.merge_requested = bool(msg.data)
        self.merge_request_at = rospy.Time.now()

    def _managed_fault_reason(self, now: rospy.Time) -> Optional[str]:
        if self.use_active_path:
            if not self.active_path_received or self.active_path_at is None:
                return "active_path_missing"
            if (now - self.active_path_at).to_sec() > self.managed_timeout_s:
                return "active_path_stale"
        if self.require_path_manager_status:
            if self.path_manager_status_at is None:
                return "path_manager_status_missing"
            if (now - self.path_manager_status_at).to_sec() > self.managed_timeout_s:
                return "path_manager_status_stale"
        if self.use_target_speed_override:
            if self.target_speed_override_at is None:
                return "target_speed_override_missing"
            if (now - self.target_speed_override_at).to_sec() > self.target_speed_override_timeout_s:
                return "target_speed_override_stale"
        return None

    def _merge_gate_fresh(self, now: rospy.Time) -> bool:
        if not self.enable_merge_gate or not self.merge_requested:
            return True
        if self.merge_gate_at is None:
            return False
        return (now - self.merge_gate_at).to_sec() <= self.merge_gate_timeout_s

    def control_callback(self, _event: rospy.timer.TimerEvent) -> None:
        if self.latest_odom is None:
            rospy.logwarn_throttle(5.0, "Pure Pursuit가 /localization/odometry를 기다리는 중이다.")
            return

        now = rospy.Time.now()
        managed_fault = self._managed_fault_reason(now)
        merge_fresh = self._merge_gate_fresh(now)
        if managed_fault is not None or not merge_fresh:
            if self.enable_control:
                self.speed_controller.reset()
                self.command_pub.publish(self.make_command(0.0, True, 0.0, 1.0))
            self.steering_preview_pub.publish(Float64(0.0))
            rospy.logwarn_throttle(
                1.0,
                "Pure Pursuit FAIL-SAFE STOP: %s",
                managed_fault if managed_fault is not None else "merge_gate_stale",
            )
            return

        pose = self.latest_odom.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        speed = math.hypot(
            self.latest_odom.twist.twist.linear.x,
            self.latest_odom.twist.twist.linear.y,
        )
        with self.controller_lock:
            steering, path_stop, target, target_index, lookahead = self.controller.compute(
                pose.position.x,
                pose.position.y,
                yaw,
                speed,
            )
            active_count = len(self.controller.points)
        steering = max(-self.max_steering, min(self.max_steering, steering))

        avoidance_stop = self.path_manager_stop if self.require_path_manager_status else False
        merge_stop = self.merge_stop_required if (self.enable_merge_gate and self.merge_requested) else False

        # The sensor team's generic intersection detector may also fire at a
        # roundabout. Never suppress it by default. When the mission layer has
        # explicitly asserted merge_request AND the merge gate has latched GO,
        # this optional override lets the dedicated roundabout gap logic own the
        # entry decision. Pedestrian, traffic-light, avoidance and path stops are
        # never overridden.
        intersection_effective_stop = self.intersection_stop_required
        if (
            self.roundabout_override_intersection_stop
            and self.enable_merge_gate
            and self.merge_requested
            and self.merge_allowed
        ):
            intersection_effective_stop = False

        stop = (
            path_stop
            or self.pedestrian_stop_required
            or self.traffic_light_stop_required
            or intersection_effective_stop
            or avoidance_stop
            or merge_stop
        )

        target_msg = PointStamped()
        target_msg.header.stamp = now
        target_msg.header.frame_id = self.map_frame
        target_msg.point.x = target.x
        target_msg.point.y = target.y
        target_msg.point.z = target.z
        self.lookahead_pub.publish(target_msg)
        self.steering_preview_pub.publish(Float64(steering))

        effective_target_speed = (
            self.target_speed_override if self.use_target_speed_override else self.target_speed
        )
        now_sec = now.to_sec()
        if self.enable_control:
            if stop:
                self.speed_controller.reset()
                accel, brake = 0.0, 1.0
            else:
                accel, brake = self.speed_controller.compute(effective_target_speed, speed, now_sec)
            self.command_pub.publish(self.make_command(steering, stop, accel, brake))

        rospy.loginfo_throttle(
            1.0,
            "PP idx=%d lookahead=%.2f steer=%.4f stop=%s path=%s ped=%s tl=%s int=%s->%s "
            "avoid=%s merge(request=%s stop=%s allowed=%s) source=%s points=%d target_v=%.2f",
            target_index,
            lookahead,
            steering,
            stop,
            path_stop,
            self.pedestrian_stop_required,
            self.traffic_light_stop_required,
            self.intersection_stop_required,
            intersection_effective_stop,
            avoidance_stop,
            self.merge_requested,
            merge_stop,
            self.merge_allowed,
            "active_path" if self.use_active_path else "path_file",
            active_count,
            effective_target_speed,
        )

    def make_command(
        self,
        steering: float,
        stop: bool,
        accel: float = 0.0,
        brake: float = 0.0,
    ) -> CtrlCmd:
        command = CtrlCmd()
        if hasattr(command, "longlCmdType"):
            command.longlCmdType = 1
        if hasattr(command, "steering"):
            command.steering = 0.0 if stop else steering
        if hasattr(command, "brake"):
            command.brake = 1.0 if stop else brake
        if hasattr(command, "accel"):
            command.accel = 0.0 if stop else accel
        if hasattr(command, "acceleration"):
            command.acceleration = 0.0
        if hasattr(command, "velocity"):
            command.velocity = 0.0
        return command


if __name__ == "__main__":
    try:
        PurePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
