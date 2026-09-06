#!/usr/bin/env python3
"""Detect intersections and release after left-to-right traffic passes."""

import json
import math
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from camera_perception.intersection import (
    FrontCrossingVehicleLatch,
    IntersectionStateMachine,
    left_to_right_crossing_obstacles,
)


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


class IntersectionEnvironmentNode:
    def __init__(self):
        self.car_topic = _param("car_topic", "/perception/camera/car_detected")
        self.left_yellow_solid_lane_topic = _param(
            "left_yellow_solid_lane_topic",
            "/perception/camera/left_yellow_solid_lane_detected",
        )
        self.right_solid_lane_topic = _param(
            "right_solid_lane_topic",
            "/perception/camera/right_solid_lane_detected",
        )
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
        self.dynamic_obstacle_topic = _param(
            "dynamic_obstacle_topic", "/detection/dynamic_obstacles"
        )
        self.odometry_topic = _param(
            "odometry_topic", "/localization/odometry"
        )
        self.input_stale_timeout_s = float(_param("input_stale_timeout_s", 0.5))
        self.publish_rate_hz = float(_param("publish_rate_hz", 20.0))
        self.minimum_crossing_speed_mps = float(
            _param("minimum_crossing_speed_mps", 1.0)
        )
        self.minimum_rightward_speed_mps = float(
            _param("minimum_rightward_speed_mps", 0.5)
        )
        self.maximum_forward_distance_m = float(
            _param("maximum_forward_distance_m", 40.0)
        )
        self.maximum_abs_lateral_distance_m = float(
            _param("maximum_abs_lateral_distance_m", 20.0)
        )
        self.state_machine = IntersectionStateMachine(
            camera_clear_confirmation_s=float(
                _param("camera_clear_confirmation_s", 0.5)
            ),
            clear_hold_s=float(_param("clear_hold_s", 2.0)),
        )
        self.crossing_tracker = FrontCrossingVehicleLatch()

        self.camera_vehicle_detected = False
        self.camera_updated_at = None
        self.left_yellow_solid_lane_detected = False
        self.right_solid_lane_detected = False
        self.left_lane_updated_at = None
        self.right_lane_updated_at = None
        self.dynamic_obstacles = []
        self.dynamic_obstacles_updated_at = None
        self.ego_pose = None
        self.odometry_updated_at = None
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
            self.left_yellow_solid_lane_topic,
            Bool,
            self._left_yellow_solid_lane_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.dynamic_obstacle_topic,
            String,
            self._dynamic_obstacle_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.odometry_topic,
            Odometry,
            self._odometry_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.right_solid_lane_topic,
            Bool,
            self._right_solid_lane_callback,
            queue_size=1,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_rate_hz, 1.0)),
            self._timer_callback,
        )
        rospy.on_shutdown(self._shutdown)
        rospy.logwarn(
            "Intersection detector: Car=%s AND left_yellow_solid=%s AND "
            "right_solid=%s; crossing=%s odometry=%s outputs=%s,%s,%s",
            self.car_topic,
            self.left_yellow_solid_lane_topic,
            self.right_solid_lane_topic,
            self.dynamic_obstacle_topic,
            self.odometry_topic,
            self.detected_topic,
            self.driving_allowed_topic,
            self.driving_unavailable_topic,
        )

    def _car_callback(self, message):
        self.camera_vehicle_detected = bool(message.data)
        self.camera_updated_at = time.monotonic()

    def _left_yellow_solid_lane_callback(self, message):
        self.left_yellow_solid_lane_detected = bool(message.data)
        self.left_lane_updated_at = time.monotonic()

    def _right_solid_lane_callback(self, message):
        self.right_solid_lane_detected = bool(message.data)
        self.right_lane_updated_at = time.monotonic()

    def _dynamic_obstacle_callback(self, message):
        try:
            payload = json.loads(message.data)
            obstacles = payload.get("obstacles", [])
            if not isinstance(obstacles, list):
                raise ValueError("obstacles must be a list")
            self.dynamic_obstacles = obstacles
            self.dynamic_obstacles_updated_at = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            rospy.logwarn_throttle(
                1.0, "Invalid intersection obstacle payload: %s", error
            )

    def _odometry_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.ego_pose = (
            float(position.x),
            float(position.y),
            _yaw_from_quaternion(orientation),
        )
        self.odometry_updated_at = time.monotonic()

    @staticmethod
    def _fresh(updated_at, now, timeout):
        return updated_at is not None and now - updated_at <= timeout

    def _timer_callback(self, _event):
        now = time.monotonic()
        camera_fresh = self._fresh(
            self.camera_updated_at, now, self.input_stale_timeout_s
        )
        left_lane_fresh = self._fresh(
            self.left_lane_updated_at, now, self.input_stale_timeout_s
        )
        right_lane_fresh = self._fresh(
            self.right_lane_updated_at, now, self.input_stale_timeout_s
        )
        lane_fresh = left_lane_fresh and right_lane_fresh
        recognition_conditions_met = bool(
            camera_fresh
            and lane_fresh
            and self.camera_vehicle_detected
            and self.left_yellow_solid_lane_detected
            and self.right_solid_lane_detected
        )
        lidar_fresh = self._fresh(
            self.dynamic_obstacles_updated_at,
            now,
            self.input_stale_timeout_s,
        )
        odometry_fresh = self._fresh(
            self.odometry_updated_at, now, self.input_stale_timeout_s
        )
        crossing_observations = []
        if lidar_fresh and odometry_fresh and self.ego_pose is not None:
            ego_x, ego_y, ego_yaw = self.ego_pose
            crossing_observations = left_to_right_crossing_obstacles(
                self.dynamic_obstacles,
                ego_x_map=ego_x,
                ego_y_map=ego_y,
                ego_yaw=ego_yaw,
                minimum_speed_mps=self.minimum_crossing_speed_mps,
                minimum_rightward_speed_mps=self.minimum_rightward_speed_mps,
                maximum_forward_distance_m=self.maximum_forward_distance_m,
                maximum_abs_lateral_distance_m=(
                    self.maximum_abs_lateral_distance_m
                ),
            )
        tracking_active = bool(
            self.state_machine.state != "IDLE" or recognition_conditions_met
        )
        crossing_seen, crossing_ids = self.crossing_tracker.update(
            crossing_observations,
            active=tracking_active,
        )
        decision = self.state_machine.update(
            camera_vehicle_detected=self.camera_vehicle_detected,
            left_yellow_solid_lane_detected=(
                self.left_yellow_solid_lane_detected
            ),
            right_solid_lane_detected=self.right_solid_lane_detected,
            now=now,
            camera_fresh=camera_fresh,
            lane_fresh=lane_fresh,
            crossing_vehicle_seen_in_front=crossing_seen,
        )
        if decision.state == "IDLE":
            self.crossing_tracker.reset()
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
            "recognition_rule": "CAR_AND_LEFT_YELLOW_SOLID_AND_RIGHT_SOLID",
            "camera_car_detected": bool(
                camera_fresh and self.camera_vehicle_detected
            ),
            "left_yellow_solid_lane_detected": bool(
                lane_fresh and self.left_yellow_solid_lane_detected
            ),
            "right_solid_lane_detected": bool(
                lane_fresh and self.right_solid_lane_detected
            ),
            "crossing_lidar_fresh": bool(lidar_fresh),
            "crossing_odometry_fresh": bool(odometry_fresh),
            "left_to_right_observation_ids": [
                int(observation["id"])
                for observation in crossing_observations
            ],
            "crossing_vehicle_ids": list(crossing_ids),
            "crossing_vehicle_seen_in_front": bool(crossing_seen),
        }
        self.status_publisher.publish(
            String(data=json.dumps(status, separators=(",", ":")))
        )
        if decision.state != self.last_state:
            if decision.state == "BLOCKED":
                driving_notice = "[INTERSECTION] 주행 불가능 (STOP)"
            elif decision.state == "CLEAR":
                driving_notice = "[INTERSECTION] 주행 가능 (GO)"
            else:
                driving_notice = "[INTERSECTION] 교차로 상황 해제 (IDLE)"
            rospy.logwarn(
                "\n============================================================\n"
                "%s\n"
                "camera_car=%s | left_yellow_solid=%s | right_solid=%s\n"
                "front_left_to_right_vehicle_ids=%s | crossing_seen=%s\n"
                "============================================================",
                driving_notice,
                status["camera_car_detected"],
                status["left_yellow_solid_lane_detected"],
                status["right_solid_lane_detected"],
                status["crossing_vehicle_ids"],
                status["crossing_vehicle_seen_in_front"],
            )
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
