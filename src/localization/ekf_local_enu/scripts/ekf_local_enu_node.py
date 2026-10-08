#!/usr/bin/env python3
"""MGeo local ENU GPS·IMU 2D EKF 노드.

상태:
  [x_map, y_map, yaw_map, forward_velocity, gyro_bias_z, accel_bias_x]

GPS는 /localization/gps(nav_msgs/Odometry), IMU는 /Imu(sensor_msgs/Imu)를
입력으로 사용한다. map->base_link는 이 노드가 발행한다.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from ekf_local_enu.math_utils import quaternion_to_yaw, wrap_angle, yaw_to_quaternion


def stamp_seconds(stamp: rospy.Time) -> float:
    value = stamp.to_sec()
    return value if value > 0.0 else rospy.Time.now().to_sec()


class LocalEnuEkf:
    def __init__(self) -> None:
        rospy.init_node("ekf_local_enu", anonymous=False)

        self.gps_topic = rospy.get_param("~gps_topic", "/localization/gps")
        self.imu_topic = rospy.get_param("~imu_topic", "/Imu")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odometry")
        self.pose_topic = rospy.get_param("~pose_topic", "/localization/pose")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.use_imu_orientation = bool(rospy.get_param("~use_imu_orientation", True))
        self.yaw_offset_rad = math.radians(float(rospy.get_param("~yaw_offset_deg", 0.0)))
        self.default_gps_variance = float(rospy.get_param("~default_gps_variance_m2", 1.0))
        self.default_yaw_variance = math.radians(
            float(rospy.get_param("~default_yaw_std_deg", 8.0))
        ) ** 2
        self.gyro_noise_std = float(rospy.get_param("~gyro_noise_std_rad_s", 0.08))
        self.accel_noise_std = float(rospy.get_param("~accel_noise_std_m_s2", 0.8))
        self.bias_random_walk = float(rospy.get_param("~bias_random_walk", 0.001))
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        # [x, y, yaw, v_forward, gyro_bias_z, accel_bias_x]
        self.state = np.zeros((6, 1), dtype=float)
        self.covariance = np.diag([4.0, 4.0, 0.5, 4.0, 0.05, 0.5]).astype(float)
        self.initialized = False
        self.last_imu_stamp: Optional[float] = None
        self.last_gps_stamp: Optional[float] = None
        self.latest_z = 0.0
        self.latest_imu_yaw: Optional[float] = None

        self.gps_sub = rospy.Subscriber(self.gps_topic, Odometry, self.gps_callback, queue_size=20)
        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=50)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=20)
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseWithCovarianceStamped, queue_size=20)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.loginfo(
            "ENU EKF: gps=%s imu=%s output=%s frame=%s->%s imu_orientation=%s",
            self.gps_topic,
            self.imu_topic,
            self.odom_topic,
            self.map_frame,
            self.base_frame,
            self.use_imu_orientation,
        )

    def imu_callback(self, msg: Imu) -> None:
        stamp = stamp_seconds(msg.header.stamp)
        measured_yaw = quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        if measured_yaw is not None:
            self.latest_imu_yaw = wrap_angle(measured_yaw + self.yaw_offset_rad)

        if not self.initialized:
            return

        if self.last_imu_stamp is None:
            self.last_imu_stamp = stamp
            return

        dt = stamp - self.last_imu_stamp
        self.last_imu_stamp = stamp
        if not 0.0001 < dt <= 0.25:
            rospy.logwarn_throttle(5.0, "IMU dt가 비정상이라 예측을 건너뛴다: %.6f", dt)
            return

        self.predict(
            gyro_z=float(msg.angular_velocity.z),
            accel_x=float(msg.linear_acceleration.x),
            dt=dt,
        )

        if self.use_imu_orientation and self.latest_imu_yaw is not None:
            self.update_yaw(self.latest_imu_yaw)

        self.publish(stamp)

    def gps_callback(self, msg: Odometry) -> None:
        stamp = stamp_seconds(msg.header.stamp)
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.latest_z = float(msg.pose.pose.position.z)

        gps_vx = float(msg.twist.twist.linear.x)
        gps_vy = float(msg.twist.twist.linear.y)
        gps_speed = math.hypot(gps_vx, gps_vy)

        if not self.initialized:
            self.state[0, 0] = x
            self.state[1, 0] = y
            if self.latest_imu_yaw is not None:
                self.state[2, 0] = self.latest_imu_yaw
            elif gps_speed > 0.3:
                self.state[2, 0] = math.atan2(gps_vy, gps_vx)
            else:
                self.state[2, 0] = 0.0
            self.state[3, 0] = gps_vx * math.cos(self.state[2, 0]) + gps_vy * math.sin(self.state[2, 0])
            self.initialized = True
            self.last_gps_stamp = stamp
            if self.last_imu_stamp is None:
                self.last_imu_stamp = stamp
            rospy.loginfo(
                "ENU EKF 초기화: x=%.3f y=%.3f yaw=%.2f deg v=%.3f",
                self.state[0, 0],
                self.state[1, 0],
                math.degrees(self.state[2, 0]),
                self.state[3, 0],
            )
            self.publish(stamp)
            return

        if self.last_gps_stamp is not None and stamp <= self.last_gps_stamp:
            return
        self.last_gps_stamp = stamp

        variance_x = float(msg.pose.covariance[0])
        variance_y = float(msg.pose.covariance[7])
        if variance_x <= 0.0:
            variance_x = self.default_gps_variance
        if variance_y <= 0.0:
            variance_y = self.default_gps_variance
        self.update_position(x, y, variance_x, variance_y)
        self.publish(stamp)

    def predict(self, gyro_z: float, accel_x: float, dt: float) -> None:
        x, y, yaw, velocity, gyro_bias, accel_bias = self.state[:, 0]
        yaw_rate = gyro_z - gyro_bias
        forward_accel = accel_x - accel_bias

        self.state[0, 0] = x + velocity * math.cos(yaw) * dt + 0.5 * forward_accel * math.cos(yaw) * dt * dt
        self.state[1, 0] = y + velocity * math.sin(yaw) * dt + 0.5 * forward_accel * math.sin(yaw) * dt * dt
        self.state[2, 0] = wrap_angle(yaw + yaw_rate * dt)
        self.state[3, 0] = velocity + forward_accel * dt

        f = np.eye(6, dtype=float)
        f[0, 2] = -velocity * math.sin(yaw) * dt
        f[0, 3] = math.cos(yaw) * dt
        f[1, 2] = velocity * math.cos(yaw) * dt
        f[1, 3] = math.sin(yaw) * dt
        f[2, 4] = -dt
        f[3, 5] = -dt

        q = np.diag([
            0.02 * dt,
            0.02 * dt,
            self.gyro_noise_std**2 * dt * dt,
            self.accel_noise_std**2 * dt * dt,
            self.bias_random_walk * dt,
            self.bias_random_walk * dt,
        ])
        self.covariance = f @ self.covariance @ f.T + q

    def update_position(self, x: float, y: float, variance_x: float, variance_y: float) -> None:
        h = np.zeros((2, 6), dtype=float)
        h[0, 0] = 1.0
        h[1, 1] = 1.0
        measurement = np.array([[x], [y]], dtype=float)
        expected = h @ self.state
        innovation = measurement - expected
        r = np.diag([variance_x, variance_y])
        s = h @ self.covariance @ h.T + r
        gain = self.covariance @ h.T @ np.linalg.inv(s)
        self.state = self.state + gain @ innovation
        self.state[2, 0] = wrap_angle(self.state[2, 0])
        identity = np.eye(6, dtype=float)
        self.covariance = (identity - gain @ h) @ self.covariance

    def update_yaw(self, yaw_measurement: float) -> None:
        h = np.zeros((1, 6), dtype=float)
        h[0, 2] = 1.0
        innovation = np.array([[wrap_angle(yaw_measurement - self.state[2, 0])]], dtype=float)
        s = h @ self.covariance @ h.T + self.default_yaw_variance
        gain = self.covariance @ h.T / float(s[0, 0])
        self.state = self.state + gain @ innovation
        self.state[2, 0] = wrap_angle(self.state[2, 0])
        self.covariance = (np.eye(6) - gain @ h) @ self.covariance

    def publish(self, stamp_seconds_value: float) -> None:
        stamp = rospy.Time.from_sec(stamp_seconds_value)
        x, y, yaw, velocity, _, _ = self.state[:, 0]
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = self.latest_z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = velocity * math.cos(yaw)
        odom.twist.twist.linear.y = velocity * math.sin(yaw)
        odom.pose.covariance[0] = self.covariance[0, 0]
        odom.pose.covariance[7] = self.covariance[1, 1]
        odom.pose.covariance[35] = self.covariance[2, 2]
        self.odom_pub.publish(odom)

        pose = PoseWithCovarianceStamped()
        pose.header = odom.header
        pose.pose.pose = odom.pose.pose
        pose.pose.covariance = odom.pose.covariance
        self.pose_pub.publish(pose)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = self.latest_z
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

        rospy.loginfo_throttle(
            2.0,
            "EKF pose map=(%.3f, %.3f) yaw=%.2f deg v=%.3f",
            x,
            y,
            math.degrees(yaw),
            velocity,
        )


if __name__ == "__main__":
    try:
        LocalEnuEkf()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
