#!/usr/bin/env python3
"""Detect crossing traffic from YOLO vehicle state and map-frame LiDAR motion."""

import json
import math
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from camera_perception.intersection import (
    IntersectionStateMachine,
    perpendicular_dynamic_obstacles,
)


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _quaternion_to_yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


class IntersectionEnvironmentNode:
    def __init__(self):
        self.car_topic = _param("car_topic", "/perception/camera/car_detected")
        self.dynamic_obstacle_topic = _param(
            "dynamic_obstacle_topic", "/detection/dynamic_obstacles"
        )
        self.odometry_topic = _param("odometry_topic", "/localization/odometry")
        self.detected_topic = _param(
            "detected_topic", "/perception/intersection/detected"
        )
        self.driving_allowed_topic = _param(
            "driving_allowed_topic", "/perception/intersection/driving_allowed"
        )
        self.driving_unavailable_topic = _param(
            "driving_unavailable_topic",
            "/perception/intersection/driving_unavailable",
        )
        self.status_topic = _param(
            "status_topic", "/perception/intersection/status"
        )
        self.minimum_speed_mps = float(_param("minimum_speed_mps", 1.0))
        self.maximum_range_m = float(_param("maximum_range_m", 40.0))
        self.maximum_perpendicular_error_deg = float(
            _param("maximum_perpendicular_error_deg", 20.0)
        )
        self.input_stale_timeout_s = float(_param("input_stale_timeout_s", 0.5))
        self.publish_rate_hz = float(_param("publish_rate_hz", 20.0))
        self.state_machine = IntersectionStateMachine(
            camera_clear_confirmation_s=float(
                _param("camera_clear_confirmation_s", 0.5)
            ),
            clear_hold_s=float(_param("clear_hold_s", 2.0)),
        )

        self.camera_vehicle_detected = False
        self.camera_updated_at = None
        self.perpendicular_count = 0
        self.lidar_updated_at = None
        self.latest_odom = None
        self.last_state = None

        self.detected_publisher = rospy.Publisher(
            self.detected_topic, Bool, queue_size=1, latch=True
        )
        self.allowed_publisher = rospy.Publisher(
            self.driving_allowed_topic, Bool, queue_size=1, latch=True
        )
        self.unavailable_publisher = rospy.Publisher(
            self.driving_unavailable_topic, Bool, queue_size=1, latch=True
        )
        self.status_publisher = rospy.Publisher(
            self.status_topic, String, queue_size=1, latch=True
        )
        rospy.Subscriber(self.car_topic, Bool, self._car_callback, queue_size=1)
        rospy.Subscriber(
            self.dynamic_obstacle_topic,
            String,
            self._dynamic_obstacle_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.odometry_topic, Odometry, self._odometry_callback, queue_size=1
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_rate_hz, 1.0)),
            self._timer_callback,
        )
        rospy.on_shutdown(self._shutdown)
        rospy.logwarn(
            "Intersection detector: YOLO=%s AND perpendicular dynamic LiDAR=%s; "
            "angle=90+/-%.1fdeg speed>=%.2fm/s range<=%.1fm outputs=%s,%s,%s",
            self.car_topic,
            self.dynamic_obstacle_topic,
            self.maximum_perpendicular_error_deg,
            self.minimum_speed_mps,
            self.maximum_range_m,
            self.detected_topic,
            self.driving_allowed_topic,
            self.driving_unavailable_topic,
        )

    def _car_callback(self, message):
        self.camera_vehicle_detected = bool(message.data)
        self.camera_updated_at = time.monotonic()

    def _odometry_callback(self, message):
        self.latest_odom = message

    def _dynamic_obstacle_callback(self, message):
        now = time.monotonic()
        self.lidar_updated_at = now
        self.perpendicular_count = 0
        if self.latest_odom is None:
            return
        try:
            payload = json.loads(message.data)
            obstacles = payload.get("obstacles", [])
        except (TypeError, ValueError, AttributeError):
            rospy.logwarn_throttle(2.0, "Invalid dynamic obstacle JSON")
            return
        pose = self.latest_odom.pose.pose
        selected = perpendicular_dynamic_obstacles(
            obstacles,
            ego_x_map=float(pose.position.x),
            ego_y_map=float(pose.position.y),
            ego_yaw=_quaternion_to_yaw(pose.orientation),
            minimum_speed_mps=self.minimum_speed_mps,
            maximum_range_m=self.maximum_range_m,
            maximum_perpendicular_error_deg=self.maximum_perpendicular_error_deg,
        )
        self.perpendicular_count = len(selected)

    @staticmethod
    def _fresh(updated_at, now, timeout):
        return updated_at is not None and now - updated_at <= timeout

    def _timer_callback(self, _event):
        now = time.monotonic()
        camera_fresh = self._fresh(
            self.camera_updated_at, now, self.input_stale_timeout_s
        )
        lidar_fresh = self._fresh(
            self.lidar_updated_at, now, self.input_stale_timeout_s
        )
        perpendicular_detected = lidar_fresh and self.perpendicular_count > 0
        decision = self.state_machine.update(
            camera_vehicle_detected=self.camera_vehicle_detected,
            perpendicular_dynamic_detected=perpendicular_detected,
            now=now,
            camera_fresh=camera_fresh,
        )
        self.detected_publisher.publish(Bool(data=decision.detected))
        self.allowed_publisher.publish(Bool(data=decision.driving_allowed))
        self.unavailable_publisher.publish(
            Bool(data=decision.driving_unavailable)
        )
        status = {
            "state": decision.state,
            "intersection_detected": decision.detected,
            "driving_allowed": decision.driving_allowed,
            "driving_unavailable": decision.driving_unavailable,
            "camera_vehicle_detected": bool(
                camera_fresh and self.camera_vehicle_detected
            ),
            "perpendicular_dynamic_detected": bool(perpendicular_detected),
            "perpendicular_object_count": int(
                self.perpendicular_count if lidar_fresh else 0
            ),
        }
        self.status_publisher.publish(
            String(data=json.dumps(status, separators=(",", ":")))
        )
        if decision.state != self.last_state:
            rospy.logwarn("Intersection state changed: %s", status)
            self.last_state = decision.state

    def _shutdown(self):
        self.detected_publisher.publish(Bool(data=False))
        self.allowed_publisher.publish(Bool(data=False))
        self.unavailable_publisher.publish(Bool(data=False))


def main():
    rospy.init_node("intersection_environment")
    IntersectionEnvironmentNode()
    rospy.spin()


if __name__ == "__main__":
    main()
