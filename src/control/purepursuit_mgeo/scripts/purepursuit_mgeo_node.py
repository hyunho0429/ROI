#!/usr/bin/env python3
"""MGeo local ENU pose를 이용해 MORAI CtrlCmd Pure Pursuit를 실행한다."""

from __future__ import annotations

import math
from typing import Optional

import rospy
from geometry_msgs.msg import PointStamped
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry
from path_planning.longitudinal_controller import PedalSpeedController
from std_msgs.msg import Bool, Float64

from purepursuit_mgeo.path import MgeoPurePursuit, PathPoint, load_mgeo_path
from purepursuit_mgeo.stopline_approach import StoplineApproachController


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
        self.max_steering = float(
            rospy.get_param("~max_steering_rad", math.radians(40.0))
        )
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
        self.stopline_approach = StoplineApproachController(
            target_distance_m=float(
                rospy.get_param("~stopline_target_distance_m", 2.0)
            ),
            comfortable_decel_mps2=float(
                rospy.get_param("~stopline_comfort_decel_mps2", 1.5)
            ),
            stale_timeout_s=float(
                rospy.get_param("~stopline_stale_timeout_s", 0.5)
            ),
            maximum_detection_distance_m=float(
                rospy.get_param("~stopline_maximum_distance_m", 40.0)
            ),
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

        self.pose_topic = rospy.get_param("~pose_topic", "/localization/odometry")
        self.command_topic = rospy.get_param("~command_topic", "/ctrl_cmd")
        self.lookahead_topic = rospy.get_param("~lookahead_topic", "/control/lookahead_point")
        self.pedestrian_stop_topic = rospy.get_param(
            "~pedestrian_stop_topic",
            "/perception/pedestrian_crossing/stop_required",
        )
        self.traffic_light_stop_topic = rospy.get_param(
            "~traffic_light_stop_topic",
            "/perception/traffic_light/stop_required",
        )
        self.intersection_stop_topic = rospy.get_param(
            "~intersection_stop_topic",
            "/perception/intersection/driving_unavailable",
        )
        self.stopline_distance_topic = rospy.get_param(
            "~stopline_distance_topic",
            "/perception/camera/stopline_distance_m",
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.latest_odom: Optional[Odometry] = None
        self.pedestrian_stop_required = False
        self.traffic_light_stop_required = False
        self.intersection_stop_required = False

        rospy.Subscriber(self.pose_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(
            self.pedestrian_stop_topic,
            Bool,
            self.pedestrian_stop_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.traffic_light_stop_topic,
            Bool,
            self.traffic_light_stop_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.intersection_stop_topic,
            Bool,
            self.intersection_stop_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.stopline_distance_topic,
            Float64,
            self.stopline_distance_callback,
            queue_size=1,
        )
        self.command_pub = rospy.Publisher(self.command_topic, CtrlCmd, queue_size=1)
        self.lookahead_pub = rospy.Publisher(self.lookahead_topic, PointStamped, queue_size=1)
        self.steering_preview_pub = rospy.Publisher("/control/steering_preview", Float64, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.control_callback)

        rospy.logwarn(
            "Pure Pursuit 제어=%s path=%s points=%d speed=%.2fm/s (%.1fkm/h) "
            "wheelbase=%.3f lookahead_min=%.3f",
            self.enable_control,
            path_file,
            len(self.points),
            self.target_speed,
            self.target_speed * 3.6,
            wheelbase,
            lookahead_min,
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def pedestrian_stop_callback(self, msg: Bool) -> None:
        previous = self.pedestrian_stop_required
        self.pedestrian_stop_required = bool(msg.data)
        if self.pedestrian_stop_required != previous:
            rospy.logwarn(
                "Pedestrian crossing control: stop_required=%s",
                self.pedestrian_stop_required,
            )

    def traffic_light_stop_callback(self, msg: Bool) -> None:
        previous = self.traffic_light_stop_required
        self.traffic_light_stop_required = bool(msg.data)
        if self.traffic_light_stop_required != previous:
            rospy.logwarn(
                "Traffic-light control: stop_required=%s",
                self.traffic_light_stop_required,
            )

    def intersection_stop_callback(self, msg: Bool) -> None:
        previous = self.intersection_stop_required
        self.intersection_stop_required = bool(msg.data)
        if self.intersection_stop_required != previous:
            rospy.logwarn(
                "Intersection control: stop_required=%s",
                self.intersection_stop_required,
            )

    def stopline_distance_callback(self, msg: Float64) -> None:
        regulatory_trigger = (
            self.traffic_light_stop_required or self.intersection_stop_required
        )
        if regulatory_trigger:
            self.stopline_approach.observe_distance(
                msg.data,
                rospy.Time.now().to_sec(),
            )

    def control_callback(self, _event: rospy.timer.TimerEvent) -> None:
        if self.latest_odom is None:
            rospy.logwarn_throttle(5.0, "Pure Pursuit가 /localization/odometry를 기다리는 중이다.")
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
        steering, stop, target, target_index, lookahead = self.controller.compute(
            pose.position.x,
            pose.position.y,
            yaw,
            speed,
        )
        steering = max(-self.max_steering, min(self.max_steering, steering))
        now_sec = rospy.Time.now().to_sec()
        path_stop = stop
        regulatory_trigger = (
            self.traffic_light_stop_required or self.intersection_stop_required
        )
        stopline_decision = self.stopline_approach.update(
            regulatory_trigger,
            self.target_speed,
            now_sec,
        )
        stop = (
            path_stop
            or self.pedestrian_stop_required
            or stopline_decision.full_stop
        )

        target_msg = PointStamped()
        target_msg.header.stamp = rospy.Time.now()
        target_msg.header.frame_id = self.map_frame
        target_msg.point.x = target.x
        target_msg.point.y = target.y
        target_msg.point.z = target.z
        self.lookahead_pub.publish(target_msg)
        self.steering_preview_pub.publish(Float64(steering))

        if self.enable_control:
            if stop:
                self.speed_controller.reset()
                accel, brake = 0.0, 1.0
            else:
                accel, brake = self.speed_controller.compute(
                    stopline_decision.target_speed_mps,
                    speed,
                    now_sec,
                )
            command = self.make_command(steering, stop, accel, brake)
            self.command_pub.publish(command)

        rospy.loginfo_throttle(
            2.0,
            "Pure Pursuit index=%d lookahead=%.2f steering=%.4f "
            "stop=%s path_stop=%s pedestrian_stop=%s traffic_light_stop=%s "
            "intersection_stop=%s stopline=%s distance=%.2f target_speed=%.2f",
            target_index,
            lookahead,
            steering,
            stop,
            path_stop,
            self.pedestrian_stop_required,
            self.traffic_light_stop_required,
            self.intersection_stop_required,
            stopline_decision.reason,
            stopline_decision.distance_m,
            stopline_decision.target_speed_mps,
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
            command.acceleration = 0.0 if stop else 0.0
        if hasattr(command, "velocity"):
            # longlCmdType=1에서는 velocity 필드가 비활성이다.
            command.velocity = 0.0
        return command


if __name__ == "__main__":
    try:
        PurePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
