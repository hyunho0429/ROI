#!/usr/bin/env python3
"""Detect moving vehicles parallel to the ego in the left adjacent lane."""

import json
import math
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from lidar_perception.lidar_merge_gap import (
    select_parallel_dynamic_obstacles_in_adjacent_lane,
)


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class LeftParallelDynamicNode:
    def __init__(self):
        self.obstacle_topic = _param(
            "obstacle_topic", "/detection/obstacle_states"
        )
        self.odometry_topic = _param(
            "odometry_topic", "/localization/odometry"
        )
        self.output_topic = _param(
            "output_topic",
            "/perception/lidar/left_lane_parallel_dynamic_detected",
        )
        self.lane_width_m = float(_param("lane_width_m", 3.5))
        self.vehicle_width_m = float(_param("vehicle_width_m", 1.892))
        self.lane_lateral_allowance_m = float(
            _param("lane_lateral_allowance_m", 0.4)
        )
        self.detection_range_m = float(_param("detection_range_m", 40.0))
        self.minimum_speed_mps = float(_param("minimum_speed_mps", 1.0))
        self.maximum_heading_error_rad = math.radians(
            float(_param("maximum_heading_error_deg", 30.0))
        )
        self.maximum_lateral_speed_mps = float(
            _param("maximum_lateral_speed_mps", 1.5)
        )
        self.confirmation_scans = int(_param("confirmation_scans", 3))
        self.input_stale_timeout_s = float(
            _param("input_stale_timeout_s", 0.5)
        )
        self.publish_rate_hz = float(_param("publish_rate_hz", 20.0))
        if self.confirmation_scans < 1:
            raise ValueError("confirmation_scans must be at least 1")
        if self.input_stale_timeout_s <= 0.0 or self.publish_rate_hz <= 0.0:
            raise ValueError("timeouts and publish rate must be positive")

        self.obstacles = []
        self.ego_pose = None
        self.last_obstacle_at = None
        self.last_odometry_at = None
        self.positive_scan_count = 0
        self.last_output = None
        self.last_matching_ids = []

        self.publisher = rospy.Publisher(self.output_topic, Bool, queue_size=1)
        self.obstacle_subscriber = rospy.Subscriber(
            self.obstacle_topic, String, self._obstacle_callback, queue_size=1
        )
        self.odometry_subscriber = rospy.Subscriber(
            self.odometry_topic,
            Odometry,
            self._odometry_callback,
            queue_size=1,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self._timer_callback
        )
        rospy.on_shutdown(self._shutdown)
        rospy.logwarn(
            "Left parallel dynamic gate: obstacles=%s odometry=%s output=%s "
            "range=+/-%.1fm speed>=%.2fm/s heading<=%.1fdeg",
            self.obstacle_topic,
            self.odometry_topic,
            self.output_topic,
            self.detection_range_m,
            self.minimum_speed_mps,
            math.degrees(self.maximum_heading_error_rad),
        )

    def _obstacle_callback(self, message):
        try:
            payload = json.loads(message.data)
            obstacles = payload.get("obstacles", [])
            if not isinstance(obstacles, list):
                raise ValueError("obstacles must be a list")
            self.obstacles = obstacles
            self.last_obstacle_at = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            rospy.logwarn_throttle(
                1.0, "Invalid obstacle-state payload: %s", error
            )

    def _odometry_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.ego_pose = (
            float(position.x),
            float(position.y),
            _yaw_from_quaternion(orientation),
        )
        self.last_odometry_at = time.monotonic()

    def _inputs_are_fresh(self, now):
        return (
            self.ego_pose is not None
            and self.last_obstacle_at is not None
            and self.last_odometry_at is not None
            and now - self.last_obstacle_at <= self.input_stale_timeout_s
            and now - self.last_odometry_at <= self.input_stale_timeout_s
        )

    def _matching_obstacles(self):
        ego_x, ego_y, ego_yaw = self.ego_pose
        return select_parallel_dynamic_obstacles_in_adjacent_lane(
            obstacles=self.obstacles,
            ego_x_map=ego_x,
            ego_y_map=ego_y,
            ego_yaw=ego_yaw,
            side="left",
            lane_width_m=self.lane_width_m,
            vehicle_width_m=self.vehicle_width_m,
            lane_lateral_allowance_m=self.lane_lateral_allowance_m,
            detection_range_m=self.detection_range_m,
            minimum_speed_mps=self.minimum_speed_mps,
            maximum_heading_error_rad=self.maximum_heading_error_rad,
            maximum_lateral_speed_mps=self.maximum_lateral_speed_mps,
        )

    def _timer_callback(self, _event):
        now = time.monotonic()
        matches = self._matching_obstacles() if self._inputs_are_fresh(now) else []
        if matches:
            self.positive_scan_count = min(
                self.confirmation_scans, self.positive_scan_count + 1
            )
        else:
            self.positive_scan_count = 0
        active = self.positive_scan_count >= self.confirmation_scans
        matching_ids = [int(obstacle.get("id", -1)) for obstacle in matches]
        self.publisher.publish(Bool(data=active))
        if active != self.last_output or matching_ids != self.last_matching_ids:
            rospy.logwarn(
                "Left parallel dynamic changed: active=%s ids=%s "
                "confirmation=%d/%d",
                active,
                matching_ids,
                self.positive_scan_count,
                self.confirmation_scans,
            )
            self.last_output = active
            self.last_matching_ids = matching_ids

    def _shutdown(self):
        self.publisher.publish(Bool(data=False))


def main():
    rospy.init_node("lidar_left_parallel_dynamic")
    LeftParallelDynamicNode()
    rospy.spin()


if __name__ == "__main__":
    main()
