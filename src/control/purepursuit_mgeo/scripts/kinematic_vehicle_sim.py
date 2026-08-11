#!/usr/bin/env python3
"""MORAI CtrlCmd를 받는 2D kinematic bicycle 차량 테스트 노드.

실제 MORAI 차량 대신 Pure Pursuit의 조향·속도 명령을 받아
MGeo map 좌표의 가상 차량 pose를 적분한다. 컨트롤러 폐루프 검증용이며
MORAI의 물리 모델이나 UDP 통신을 대체하지 않는다.
"""

from __future__ import annotations

import math
import os
from typing import Optional, Tuple

import rospy
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry

from purepursuit_mgeo.path import PathPoint, load_mgeo_path


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def path_yaw(points, index: int) -> float:
    current = points[index]
    for offset in range(1, len(points)):
        candidate = points[(index + offset) % len(points)]
        dx = candidate.x - current.x
        dy = candidate.y - current.y
        if math.hypot(dx, dy) > 1e-6:
            return math.atan2(dy, dx)
    return 0.0


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


class KinematicVehicleSim:
    def __init__(self) -> None:
        rospy.init_node("kinematic_vehicle_sim", anonymous=False)

        default_path = os.path.join(
            os.environ.get("HOME", "/home"),
            "morai_ws",
            "data",
            "routes",
            "2026_molit_comp_global_path.txt",
        )
        path_file = rospy.get_param("~path_file", default_path)
        points = load_mgeo_path(path_file)
        start_index = int(rospy.get_param("~start_index", 0))
        start_index = max(0, min(start_index, len(points) - 1))
        start = points[start_index]

        self.wheelbase_m = float(rospy.get_param("~wheelbase_m", 3.0))
        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))
        self.max_steering_rad = float(rospy.get_param("~max_steering_rad", math.radians(40.0)))
        self.max_accel_mps2 = float(rospy.get_param("~max_accel_mps2", 2.0))
        self.max_decel_mps2 = float(rospy.get_param("~max_decel_mps2", 4.0))
        self.speed_time_constant = float(rospy.get_param("~speed_time_constant", 0.5))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 0.5))
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odometry")
        self.command_topic = rospy.get_param("~command_topic", "/ctrl_cmd")

        self.x = start.x
        self.y = start.y
        self.z = start.z
        self.yaw = path_yaw(points, start_index)
        self.speed_mps = float(rospy.get_param("~initial_speed_mps", 0.0))
        self.target_speed_mps = 0.0
        self.steering_rad = 0.0
        self.last_command_time: Optional[float] = None
        self.last_update_time = rospy.Time.now().to_sec()

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        rospy.Subscriber(self.command_topic, CtrlCmd, self.command_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.update)

        rospy.logwarn(
            "Kinematic vehicle simulation is active: wheelbase=%.3f, command=%s, odom=%s",
            self.wheelbase_m,
            self.command_topic,
            self.odom_topic,
        )

    def command_callback(self, message: CtrlCmd) -> None:
        self.last_command_time = rospy.Time.now().to_sec()
        self.steering_rad = max(
            -self.max_steering_rad,
            min(self.max_steering_rad, float(getattr(message, "steering", 0.0))),
        )

        brake = max(0.0, float(getattr(message, "brake", 0.0)))
        velocity = max(0.0, float(getattr(message, "velocity", 0.0)))
        acceleration = float(getattr(message, "acceleration", 0.0))
        accel = float(getattr(message, "accel", 0.0))

        # 현재 Pure Pursuit는 longlCmdType=2와 velocity를 사용한다.
        if int(getattr(message, "longlCmdType", 2)) == 2:
            self.target_speed_mps = velocity
        else:
            self.target_speed_mps = max(0.0, self.speed_mps + max(acceleration, accel))
        if brake > 0.0:
            self.target_speed_mps = 0.0

    def update(self, _event: rospy.timer.TimerEvent) -> None:
        now = rospy.Time.now().to_sec()
        dt = now - self.last_update_time
        self.last_update_time = now
        if not 0.0 < dt <= 0.2:
            return

        if self.last_command_time is None or now - self.last_command_time > self.command_timeout_sec:
            target_speed = 0.0
            self.steering_rad = 0.0
        else:
            target_speed = self.target_speed_mps

        speed_error = target_speed - self.speed_mps
        max_delta = self.max_accel_mps2 * dt if speed_error >= 0.0 else self.max_decel_mps2 * dt
        speed_delta = max(-max_delta, min(max_delta, speed_error))
        self.speed_mps = max(0.0, self.speed_mps + speed_delta)

        self.x += self.speed_mps * math.cos(self.yaw) * dt
        self.y += self.speed_mps * math.sin(self.yaw) * dt
        self.yaw = wrap_angle(
            self.yaw + self.speed_mps / max(self.wheelbase_m, 1e-6) * math.tan(self.steering_rad) * dt
        )
        self.publish_odom(now)

    def publish_odom(self, stamp_sec: float) -> None:
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        message = Odometry()
        message.header.stamp = rospy.Time.from_sec(stamp_sec)
        message.header.frame_id = self.map_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.position.z = self.z
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = self.speed_mps * math.cos(self.yaw)
        message.twist.twist.linear.y = self.speed_mps * math.sin(self.yaw)
        self.odom_pub.publish(message)


if __name__ == "__main__":
    try:
        KinematicVehicleSim()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
