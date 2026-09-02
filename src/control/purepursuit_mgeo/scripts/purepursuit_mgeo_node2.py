#!/usr/bin/env python3
"""MGeo local ENU pose를 이용해 MORAI CtrlCmd Pure Pursuit를 실행한다.

This version keeps the team's verified Pure Pursuit/command generation intact
and only adds an optional managed-path input for the avoidance Path Manager.
Legacy behavior is preserved when ``~use_active_path`` is false.
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
        self.points = load_mgeo_path(path_file)
        self.target_speed = float(rospy.get_param("~target_speed_mps", 2.0))
        if self.target_speed < 0.0:
            raise ValueError("target_speed_mps must be zero or positive")
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
        self.lookahead_topic = rospy.get_param(
            "~lookahead_topic", "/control/lookahead_point"
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.latest_odom: Optional[Odometry] = None

        # Optional Path Manager integration.  Defaults are intentionally OFF so
        # the original camera-team launch behaves exactly as before.
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

        self.active_path_received = False
        self.active_path_at: Optional[rospy.Time] = None
        self.path_manager_stop = True if self.require_path_manager_status else False
        self.path_manager_status_at: Optional[rospy.Time] = None

        rospy.Subscriber(self.pose_topic, Odometry, self.odom_callback, queue_size=10)
        if self.use_active_path:
            rospy.Subscriber(
                self.active_path_topic,
                RosPath,
                self.active_path_callback,
                queue_size=1,
            )
        if self.require_path_manager_status:
            rospy.Subscriber(
                self.stop_required_topic,
                Bool,
                self.stop_required_callback,
                queue_size=1,
            )

        self.command_pub = rospy.Publisher(self.command_topic, CtrlCmd, queue_size=1)
        self.lookahead_pub = rospy.Publisher(
            self.lookahead_topic, PointStamped, queue_size=1
        )
        self.steering_preview_pub = rospy.Publisher(
            "/control/steering_preview", Float64, queue_size=1
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.control_callback
        )

        rospy.logwarn(
            "Pure Pursuit 제어=%s path=%s points=%d speed=%.2fm/s (%.1fkm/h) "
            "wheelbase=%.3f lookahead_min=%.3f managed_path=%s",
            self.enable_control,
            path_file,
            len(self.points),
            self.target_speed,
            self.target_speed * 3.6,
            wheelbase,
            lookahead_min,
            self.use_active_path,
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

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

    def _managed_stop_reason(self, now: rospy.Time) -> Optional[str]:
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
            if self.path_manager_stop:
                return "path_manager_stop_required"

        return None

    def control_callback(self, _event: rospy.timer.TimerEvent) -> None:
        if self.latest_odom is None:
            rospy.logwarn_throttle(
                5.0, "Pure Pursuit가 /localization/odometry를 기다리는 중이다."
            )
            return

        now = rospy.Time.now()
        managed_stop_reason = self._managed_stop_reason(now)
        if managed_stop_reason is not None:
            if self.enable_control:
                self.command_pub.publish(self.make_command(0.0, True))
            self.steering_preview_pub.publish(Float64(0.0))
            rospy.logwarn_throttle(
                1.0, "Pure Pursuit managed STOP: %s", managed_stop_reason
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
            2.0,
            "Pure Pursuit index=%d lookahead=%.2f steering=%.4f stop=%s source=%s points=%d",
            target_index,
            lookahead,
            steering,
            stop,
            "active_path" if self.use_active_path else "path_file",
            len(self.controller.points),
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
            command.accel = 0.0 if stop else 0.0
        if hasattr(command, "acceleration"):
            command.acceleration = 0.0 if stop else 0.0
        if hasattr(command, "velocity"):
            command.velocity = 0.0 if stop else self.target_speed
        return command


if __name__ == "__main__":
    try:
        PurePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
