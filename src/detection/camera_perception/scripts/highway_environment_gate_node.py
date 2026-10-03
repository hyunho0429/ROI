#!/usr/bin/env python3
"""Build a stable highway-environment gate from camera and LiDAR states."""

import json
import math
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from camera_perception.highway_environment import (
    AdjacentDashedHold,
    HighwayEnvironmentLatch,
    ConsecutiveLanePattern,
    exclusive_highway_active,
    multilane_highway_pattern,
)


def _param(name, default):
    return rospy.get_param("~" + name, default)


class HighwayEnvironmentGateNode:
    def __init__(self):
        self.car_detected_topic = _param(
            "car_detected_topic", "/perception/camera/car_detected"
        )
        self.dashed_lane_topic = _param(
            "dashed_lane_topic", "/perception/camera/dashed_lane_detected"
        )
        self.left_solid_lane_topic = _param(
            "left_solid_lane_topic",
            "/perception/camera/left_solid_lane_detected",
        )
        self.left_parallel_dynamic_topic = _param(
            "left_parallel_dynamic_topic",
            "/perception/lidar/left_lane_parallel_dynamic_detected",
        )
        self.output_topic = _param(
            "output_topic", "/perception/camera/highway_environment"
        )
        self.intersection_detected_topic = _param(
            "intersection_detected_topic", "/perception/intersection/detected"
        )
        self.require_dashed_lane = bool(_param("require_dashed_lane", False))
        self.require_left_parallel_dynamic = bool(
            _param("require_left_parallel_dynamic", False)
        )
        self.latch_once = bool(_param("latch_once", True))
        self.car_hold_s = float(_param("car_hold_s", 2.0))
        self.dashed_lane_hold_s = float(_param("dashed_lane_hold_s", 2.0))
        self.left_parallel_dynamic_hold_s = float(
            _param("left_parallel_dynamic_hold_s", 0.5)
        )
        self.publish_rate_hz = float(_param("publish_rate_hz", 10.0))
        self.lane_info_topic = _param("lane_info_topic", "/perception/camera/lane_info")
        self.odom_topic = _param("odom_topic", "/localization/odometry")
        self.lane_pattern_enabled = bool(_param("lane_pattern_enabled", False))
        self.lane_pattern_min_speed_mps = float(_param("lane_pattern_min_speed_mps", 10.0))
        self.lane_pattern_timeout_s = float(_param("lane_pattern_timeout_s", 0.6))
        self.lane_pattern_tracker = ConsecutiveLanePattern(
            _param("lane_pattern_confirm_frames", 3)
        )
        if min(
            self.car_hold_s,
            self.dashed_lane_hold_s,
            self.left_parallel_dynamic_hold_s,
        ) <= 0.0:
            raise ValueError("camera condition hold times must be positive")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.last_car_detected_at = None
        self.dashed_hold = AdjacentDashedHold(self.dashed_lane_hold_s)
        self.last_left_parallel_dynamic_at = None
        self.last_output = None
        self.intersection_active = False
        self.last_lane_pattern = None
        self.last_lane_pattern_at = None
        self.last_ego_speed_mps = None
        self.last_odom_at = None
        self.output_lock = threading.Lock()
        self.state_latch = HighwayEnvironmentLatch(self.latch_once)

        self.publisher = rospy.Publisher(
            self.output_topic, Bool, queue_size=1
        )
        self.car_subscriber = rospy.Subscriber(
            self.car_detected_topic,
            Bool,
            self._car_callback,
            queue_size=1,
        )
        self.dashed_lane_subscriber = rospy.Subscriber(
            self.dashed_lane_topic,
            Bool,
            self._dashed_lane_callback,
            queue_size=1,
        )
        self.left_solid_lane_subscriber = rospy.Subscriber(
            self.left_solid_lane_topic,
            Bool,
            self._left_solid_lane_callback,
            queue_size=1,
        )
        self.left_parallel_dynamic_subscriber = rospy.Subscriber(
            self.left_parallel_dynamic_topic,
            Bool,
            self._left_parallel_dynamic_callback,
            queue_size=1,
        )
        self.intersection_subscriber = rospy.Subscriber(
            self.intersection_detected_topic,
            Bool,
            self._intersection_callback,
            queue_size=1,
        )
        if self.lane_pattern_enabled:
            self.lane_info_subscriber = rospy.Subscriber(
                self.lane_info_topic, String, self._lane_info_callback, queue_size=1
            )
            self.odom_subscriber = rospy.Subscriber(
                self.odom_topic, Odometry, self._odom_callback, queue_size=1
            )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self._timer_callback
        )
        rospy.on_shutdown(self._shutdown)

        rospy.logwarn(
            "Highway gate: car=%s dashed=%s left_solid=%s required_dashed=%s "
            "left_parallel_dynamic=%s required_left_parallel_dynamic=%s "
            "intersection_override=%s latch_once=%s output=%s",
            self.car_detected_topic,
            self.dashed_lane_topic,
            self.left_solid_lane_topic,
            self.require_dashed_lane,
            self.left_parallel_dynamic_topic,
            self.require_left_parallel_dynamic,
            self.intersection_detected_topic,
            self.latch_once,
            self.output_topic,
        )

    def _car_callback(self, message):
        if message.data:
            self.last_car_detected_at = time.monotonic()

    def _dashed_lane_callback(self, message):
        self.dashed_hold.observe_dashed(message.data, time.monotonic())

    def _left_solid_lane_callback(self, message):
        # A positively identified nearest solid revokes the dashed condition
        # immediately. A false dashed Bool alone is ambiguous: it can mean one
        # dropped lane frame, so let the configured hold bridge it instead.
        self.dashed_hold.observe_solid(message.data)

    def _left_parallel_dynamic_callback(self, message):
        if message.data:
            self.last_left_parallel_dynamic_at = time.monotonic()

    def _intersection_callback(self, message):
        self.intersection_active = bool(message.data)
        if self.intersection_active:
            # Publish the exclusion immediately instead of waiting for the
            # next periodic gate update.
            with self.output_lock:
                self.publisher.publish(Bool(data=False))
                self.last_output = False

    def _lane_info_callback(self, message):
        try:
            info = json.loads(message.data)
            if not isinstance(info, dict):
                return
            stamp = float(info.get("timestamp"))
            if not math.isfinite(stamp):
                return
            if info.get("observation_time_source") == "camera_receive_wall":
                age = time.time()-stamp
                if age < -0.1 or age > self.lane_pattern_timeout_s:
                    self.lane_pattern_tracker.observe(stamp, None)
                    self.last_lane_pattern = None
                    return
            pattern = multilane_highway_pattern(info)
            self.lane_pattern_tracker.observe(stamp, pattern)
            self.last_lane_pattern = pattern
            self.last_lane_pattern_at = time.monotonic()
        except (TypeError, ValueError, KeyError):
            rospy.logwarn_throttle(2.0, "Highway gate: invalid lane_info JSON")

    def _odom_callback(self, message):
        velocity = message.twist.twist.linear
        self.last_ego_speed_mps = math.hypot(velocity.x, velocity.y)
        self.last_odom_at = time.monotonic()

    @staticmethod
    def _recent(timestamp, hold_s, now):
        return timestamp is not None and now - timestamp <= hold_s

    def _timer_callback(self, _event):
        now = time.monotonic()
        car_active = self._recent(
            self.last_car_detected_at, self.car_hold_s, now
        )
        dashed_active = self.dashed_hold.active(now)
        left_parallel_dynamic_active = self._recent(
            self.last_left_parallel_dynamic_at,
            self.left_parallel_dynamic_hold_s,
            now,
        )
        lane_pattern_active = (
            self.lane_pattern_enabled
            and self._recent(self.last_lane_pattern_at, self.lane_pattern_timeout_s, now)
            and self.lane_pattern_tracker.ready(self.last_lane_pattern)
            and (
                self.last_lane_pattern == "paired_dashed_solid"
                or (
                    self.last_lane_pattern == "double_dashed"
                    and self._recent(self.last_odom_at, 0.5, now)
                    and self.last_ego_speed_mps >= self.lane_pattern_min_speed_mps
                )
            )
        )
        conditions_met = (
            (car_active and (dashed_active if self.require_dashed_lane else True)
             or lane_pattern_active)
            and (
                left_parallel_dynamic_active
                if self.require_left_parallel_dynamic
                else True
            )
        )
        highway_candidate = self.state_latch.update(conditions_met)
        with self.output_lock:
            active = exclusive_highway_active(
                highway_candidate, self.intersection_active
            )
            self.publisher.publish(Bool(data=active))

        if active != self.last_output:
            rospy.logwarn(
                "Highway environment gate changed: active=%s car=%s lane_pattern=%s "
                "dashed=%s dashed_required=%s left_parallel_dynamic=%s "
                "left_parallel_dynamic_required=%s intersection=%s latched=%s",
                active,
                car_active,
                self.last_lane_pattern if lane_pattern_active else "inactive",
                dashed_active,
                self.require_dashed_lane,
                left_parallel_dynamic_active,
                self.require_left_parallel_dynamic,
                self.intersection_active,
                self.state_latch.latched,
            )
            self.last_output = active

    def _shutdown(self):
        self.publisher.publish(Bool(data=False))


if __name__ == "__main__":
    try:
        rospy.init_node("highway_environment_gate")
        HighwayEnvironmentGateNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
