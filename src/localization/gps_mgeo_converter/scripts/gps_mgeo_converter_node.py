#!/usr/bin/env python3
"""MORAI GPSMessage를 MGeo map 좌표의 GPS/base 위치로 변환한다."""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import PointStamped
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from gps_mgeo_converter.coordinate import gps_to_mgeo


def stamp_seconds(stamp: rospy.Time) -> float:
    value = stamp.to_sec()
    return value if value > 0.0 else rospy.Time.now().to_sec()


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> Optional[float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class GpsMgeoConverter:
    def __init__(self) -> None:
        rospy.init_node("gps_mgeo_converter", anonymous=False)

        self.origin = rospy.get_param(
            "~local_origin_utm", [302595.0, 4124145.0, 0.0]
        )
        self.gps_xyz = rospy.get_param("~gps_xyz_in_base_link", [0.0, 0.0, 1.2])
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.gps_topic = rospy.get_param("~gps_topic", "/gps")
        self.imu_topic = rospy.get_param("~imu_topic", "/Imu")
        self.point_topic = rospy.get_param("~point_topic", "/gps_mgeo")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/gps")
        self.require_valid_status = bool(rospy.get_param("~require_valid_status", True))
        self.position_variance = float(rospy.get_param("~position_variance_m2", 1.0))
        self.velocity_alpha = float(rospy.get_param("~velocity_alpha", 0.25))

        self.last_yaw: Optional[float] = None
        self.last_position: Optional[Tuple[float, float, float]] = None
        self.last_stamp: Optional[float] = None
        self.velocity_xy = [0.0, 0.0]

        self.point_pub = rospy.Publisher(self.point_topic, PointStamped, queue_size=20)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=20)
        rospy.Subscriber(self.gps_topic, GPSMessage, self.gps_callback, queue_size=50)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=50)

        rospy.loginfo(
            "GPS-MGeo converter: origin=%s gps_xyz_in_base=%s output=%s",
            self.origin,
            self.gps_xyz,
            self.odom_topic,
        )

    def imu_callback(self, msg: Imu) -> None:
        self.last_yaw = quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

    def _base_position(self, gps_position: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """GPS 안테나 위치를 base_link 위치로 보정한다."""

        yaw = self.last_yaw if self.last_yaw is not None else 0.0
        dx, dy, dz = [float(value) for value in self.gps_xyz]
        rotated_dx = math.cos(yaw) * dx - math.sin(yaw) * dy
        rotated_dy = math.sin(yaw) * dx + math.cos(yaw) * dy
        return (
            gps_position[0] - rotated_dx,
            gps_position[1] - rotated_dy,
            gps_position[2] - dz,
        )

    def gps_callback(self, msg: GPSMessage) -> None:
        if self.require_valid_status and int(msg.status) <= 0:
            rospy.logwarn_throttle(5.0, "유효하지 않은 GPS status=%d를 무시한다.", msg.status)
            return

        stamp = stamp_seconds(msg.header.stamp)
        gps_position = gps_to_mgeo(
            msg.latitude,
            msg.longitude,
            msg.altitude,
            self.origin,
        )
        base_position = self._base_position(gps_position)

        if self.last_position is not None and self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if 0.001 < dt < 2.0:
                raw_vx = (base_position[0] - self.last_position[0]) / dt
                raw_vy = (base_position[1] - self.last_position[1]) / dt
                alpha = max(0.0, min(1.0, self.velocity_alpha))
                self.velocity_xy[0] = (1.0 - alpha) * self.velocity_xy[0] + alpha * raw_vx
                self.velocity_xy[1] = (1.0 - alpha) * self.velocity_xy[1] + alpha * raw_vy

        self.last_position = base_position
        self.last_stamp = stamp

        header = msg.header
        header.stamp = rospy.Time.from_sec(stamp)
        header.frame_id = self.map_frame

        point = PointStamped()
        point.header = header
        point.point.x, point.point.y, point.point.z = base_position
        self.point_pub.publish(point)

        odom = Odometry()
        odom.header = header
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = base_position[0]
        odom.pose.pose.position.y = base_position[1]
        odom.pose.pose.position.z = base_position[2]
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = self.velocity_xy[0]
        odom.twist.twist.linear.y = self.velocity_xy[1]
        odom.pose.covariance[0] = self.position_variance
        odom.pose.covariance[7] = self.position_variance
        odom.pose.covariance[14] = max(self.position_variance, 4.0)
        self.odom_pub.publish(odom)

        rospy.loginfo_throttle(
            2.0,
            "GPS map=(%.3f, %.3f, %.3f) v=(%.3f, %.3f) status=%d",
            base_position[0],
            base_position[1],
            base_position[2],
            self.velocity_xy[0],
            self.velocity_xy[1],
            msg.status,
        )


if __name__ == "__main__":
    try:
        GpsMgeoConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
