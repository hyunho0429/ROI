#!/usr/bin/env python3
"""Existing MGeo Pure Pursuit with an optional ROS active-path input.

The steering/lookahead geometry is unchanged: it still calls the existing
``MgeoPurePursuit.compute`` implementation.  This node only adds a switchable
path source and fail-safe stop inputs for the avoidance integration stage.
"""

from __future__ import annotations

import math
import threading
from typing import List, Optional

import rospy
from geometry_msgs.msg import PointStamped
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry, Path as RosPath
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
        self.global_points = load_mgeo_path(path_file)
        self.target_speed = float(rospy.get_param("~target_speed_mps", 2.0))
        self.max_steering = float(
            rospy.get_param("~max_steering_rad", math.radians(40.0))
        )
        self.rate_hz = float(rospy.get_param("~control_rate_hz", 20.0))
        self.enable_control = bool(rospy.get_param("~enable_control", False))
        self.longl_cmd_type = int(rospy.get_param("~longl_cmd_type", 2))
        self.steering_sign = float(rospy.get_param("~steering_sign", 1.0))

        wheelbase = float(rospy.get_param("~wheelbase_m", 3.0))
        lookahead_min = float(rospy.get_param("~lookahead_min_m", 4.0))
        lookahead_gain = float(rospy.get_param("~lookahead_gain", 0.35))
        goal_tolerance = float(rospy.get_param("~goal_tolerance_m", 1.5))
        self.controller = MgeoPurePursuit(
            self.global_points,
            wheelbase,
            lookahead_min,
            lookahead_gain,
            goal_tolerance,
            self.steering_sign,
        )
        self.controller_lock = threading.RLock()

        self.pose_topic = rospy.get_param("~pose_topic", "/localization/odometry")
        self.command_topic = rospy.get_param("~command_topic", "/ctrl_cmd")
        self.lookahead_topic = rospy.get_param(
            "~lookahead_topic", "/control/lookahead_point"
        )
        self.steering_preview_topic = rospy.get_param(
            "~steering_preview_topic", "/control/steering_preview"
        )
        self.map_frame = rospy.get_param("~map_frame", "map")

        # Optional active-path mode.  Defaults keep the historical behavior.
        self.use_active_path = bool(rospy.get_param("~use_active_path", False))
        self.require_active_path = bool(
            rospy.get_param("~require_active_path", False)
        )
        self.active_path_topic = rospy.get_param(
            "~active_path_topic", "/avoidance_path_manager/active_path"
        )
        self.active_path_timeout_s = float(
            rospy.get_param("~active_path_timeout_s", 0.60)
        )

        self.external_stop_topic = rospy.get_param(
            "~external_stop_topic", "/avoidance_path_manager/stop_required"
        )
        self.require_external_stop_status = bool(
            rospy.get_param("~require_external_stop_status", False)
        )
        self.external_stop_timeout_s = float(
            rospy.get_param("~external_stop_timeout_s", 0.60)
        )

        # Existing camera/pedestrian fusion can participate as an independent
        # stop condition without being merged into the planner code.
        self.pedestrian_stop_topic = rospy.get_param(
            "~pedestrian_stop_topic", "/perception/pedestrian_crossing/stop_required"
        )
        self.require_pedestrian_status = bool(
            rospy.get_param("~require_pedestrian_status", False)
        )
        self.pedestrian_stop_timeout_s = float(
            rospy.get_param("~pedestrian_stop_timeout_s", 0.75)
        )

        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.50))

        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_at: Optional[rospy.Time] = None
        self.latest_active_path_at: Optional[rospy.Time] = None
        self.active_path_received = False
        self.external_stop = True if self.require_external_stop_status else False
        self.external_stop_at: Optional[rospy.Time] = None
        self.pedestrian_stop = False
        self.pedestrian_stop_at: Optional[rospy.Time] = None

        rospy.Subscriber(self.pose_topic, Odometry, self.odom_callback, queue_size=10)
        if self.use_active_path:
            rospy.Subscriber(
                self.active_path_topic, RosPath, self.active_path_callback, queue_size=1
            )
        if self.external_stop_topic:
            rospy.Subscriber(
                self.external_stop_topic, Bool, self.external_stop_callback, queue_size=1
            )
        if self.pedestrian_stop_topic:
            rospy.Subscriber(
                self.pedestrian_stop_topic,
                Bool,
                self.pedestrian_stop_callback,
                queue_size=1,
            )

        self.command_pub = rospy.Publisher(self.command_topic, CtrlCmd, queue_size=1)
        self.lookahead_pub = rospy.Publisher(
            self.lookahead_topic, PointStamped, queue_size=1
        )
        self.steering_preview_pub = rospy.Publisher(
            self.steering_preview_topic, Float64, queue_size=1
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.control_callback
        )

        rospy.logwarn(
            "Pure Pursuit control=%s path_source=%s speed=%.2fm/s global_points=%d "
            "active_topic=%s",
            self.enable_control,
            "ACTIVE_PATH" if self.use_active_path else "GLOBAL_FILE",
            self.target_speed,
            len(self.global_points),
            self.active_path_topic,
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_odom_at = rospy.Time.now()

    def active_path_callback(self, msg: RosPath) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn_throttle(
                1.0,
                "Active path frame=%s rejected; expected %s",
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
            # Empty/short path is not installed.  Freshness will force a stop.
            return

        with self.controller_lock:
            self.controller.points = points
        self.latest_active_path_at = rospy.Time.now()
        self.active_path_received = True

    def external_stop_callback(self, msg: Bool) -> None:
        self.external_stop = bool(msg.data)
        self.external_stop_at = rospy.Time.now()

    def pedestrian_stop_callback(self, msg: Bool) -> None:
        self.pedestrian_stop = bool(msg.data)
        self.pedestrian_stop_at = rospy.Time.now()

    def _publish_stop(self, reason: str) -> None:
        rospy.logwarn_throttle(1.0, "Pure Pursuit FAIL-SAFE STOP: %s", reason)
        if self.enable_control:
            self.command_pub.publish(self.make_command(0.0, True))
        self.steering_preview_pub.publish(Float64(0.0))

    def _failsafe_reason(self, now: rospy.Time) -> Optional[str]:
        if self.latest_odom is None or self.latest_odom_at is None:
            return "odometry_missing"
        if (now - self.latest_odom_at).to_sec() > self.odom_timeout_s:
            return "odometry_stale"

        if self.use_active_path and self.require_active_path:
            if not self.active_path_received or self.latest_active_path_at is None:
                return "active_path_missing"
            if (now - self.latest_active_path_at).to_sec() > self.active_path_timeout_s:
                return "active_path_stale"

        if self.require_external_stop_status:
            if self.external_stop_at is None:
                return "path_manager_stop_status_missing"
            if (now - self.external_stop_at).to_sec() > self.external_stop_timeout_s:
                return "path_manager_stop_status_stale"

        if self.require_pedestrian_status:
            if self.pedestrian_stop_at is None:
                return "pedestrian_stop_status_missing"
            if (now - self.pedestrian_stop_at).to_sec() > self.pedestrian_stop_timeout_s:
                return "pedestrian_stop_status_stale"

        if self.external_stop:
            return "path_manager_stop_required"
        if self.pedestrian_stop:
            return "pedestrian_stop_required"
        return None

    def control_callback(self, _event: rospy.timer.TimerEvent) -> None:
        now = rospy.Time.now()
        fail_reason = self._failsafe_reason(now)
        if fail_reason is not None:
            self._publish_stop(fail_reason)
            return

        assert self.latest_odom is not None
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
            if len(self.controller.points) < 2:
                self._publish_stop("path_too_short")
                return
            steering, stop, target, target_index, lookahead = self.controller.compute(
                pose.position.x,
                pose.position.y,
                yaw,
                speed,
            )

        steering = max(-self.max_steering, min(self.max_steering, steering))

        target_msg = PointStamped()
        target_msg.header.stamp = now
        target_msg.header.frame_id = self.map_frame
        target_msg.point.x = target.x
        target_msg.point.y = target.y
        target_msg.point.z = target.z
        self.lookahead_pub.publish(target_msg)
        self.steering_preview_pub.publish(Float64(steering))

        if self.enable_control:
            command = self.make_command(steering, stop)
            self.command_pub.publish(command)

        rospy.loginfo_throttle(
            1.0,
            "Pure Pursuit ACTIVE=%s points=%d index=%d lookahead=%.2f steering=%.4f stop=%s speed=%.2f",
            self.use_active_path,
            len(self.controller.points),
            target_index,
            lookahead,
            steering,
            stop,
            self.target_speed,
        )

    def make_command(self, steering: float, stop: bool) -> CtrlCmd:
        command = CtrlCmd()
        if hasattr(command, "longlCmdType"):
            command.longlCmdType = self.longl_cmd_type
        if hasattr(command, "steering"):
            command.steering = 0.0 if stop else steering
        if hasattr(command, "brake"):
            command.brake = 1.0 if stop else 0.0
        if hasattr(command, "accel"):
            command.accel = 0.0
        if hasattr(command, "acceleration"):
            command.acceleration = 0.0
        if hasattr(command, "velocity"):
            command.velocity = 0.0 if stop else self.target_speed
        return command


if __name__ == "__main__":
    try:
        PurePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
