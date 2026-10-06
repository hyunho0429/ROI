#!/usr/bin/env python3
"""Activate the highway mission only from a fresh adjacent-left double marking."""

import json
import math
import threading
import time

import rospy
from std_msgs.msg import Bool, String

from camera_perception.highway_environment import (
    ConsecutiveLanePattern,
    HighwayEnvironmentLatch,
    adjacent_left_composite_detected,
    exclusive_highway_active,
)


PAIRED_LEFT_MARKING = "paired_dashed_solid"


def _param(name, default):
    return rospy.get_param("~" + name, default)


class HighwayEnvironmentGateNode:
    def __init__(self):
        self.lane_info_topic = _param("lane_info_topic", "/perception/camera/lane_info")
        self.output_topic = _param("output_topic", "/perception/camera/highway_environment")
        self.intersection_detected_topic = _param(
            "intersection_detected_topic", "/perception/intersection/detected"
        )
        self.latch_once = bool(_param("latch_once", True))
        self.lane_pattern_timeout_s = float(_param("lane_pattern_timeout_s", 0.6))
        self.publish_rate_hz = float(_param("publish_rate_hz", 10.0))
        self.lane_pattern_tracker = ConsecutiveLanePattern(
            _param("lane_pattern_confirm_frames", 3)
        )
        if self.lane_pattern_timeout_s <= 0.0 or self.publish_rate_hz <= 0.0:
            raise ValueError("highway gate timeouts and rate must be positive")

        self.last_lane_pattern_at = None
        self.last_output = None
        self.intersection_active = False
        self.output_lock = threading.Lock()
        self.state_latch = HighwayEnvironmentLatch(self.latch_once)

        self.publisher = rospy.Publisher(self.output_topic, Bool, queue_size=1)
        self.lane_info_subscriber = rospy.Subscriber(
            self.lane_info_topic, String, self._lane_info_callback, queue_size=1
        )
        self.intersection_subscriber = rospy.Subscriber(
            self.intersection_detected_topic,
            Bool,
            self._intersection_callback,
            queue_size=1,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self._timer_callback
        )
        rospy.on_shutdown(self._shutdown)

        rospy.logwarn(
            "Highway gate: adjacent-left dashed+solid pair from %s, "
            "intersection_override=%s latch_once=%s output=%s",
            self.lane_info_topic,
            self.intersection_detected_topic,
            self.latch_once,
            self.output_topic,
        )

    def _intersection_callback(self, message):
        with self.output_lock:
            self.intersection_active = bool(message.data)
            if self.intersection_active:
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
            if info.get("observation_time_source") != "camera_receive_wall":
                return
            age = time.time() - stamp
            if age < -0.1 or age > self.lane_pattern_timeout_s:
                with self.output_lock:
                    self.lane_pattern_tracker.observe(stamp, None)
                return
            # Other multilane patterns and YOLO vehicles cannot activate this gate.
            paired = (PAIRED_LEFT_MARKING
                      if adjacent_left_composite_detected(info) else None)
            with self.output_lock:
                self.lane_pattern_tracker.observe(stamp, paired)
                self.last_lane_pattern_at = time.monotonic()
        except (TypeError, ValueError, KeyError):
            rospy.logwarn_throttle(2.0, "Highway gate: invalid lane_info JSON")

    def _timer_callback(self, _event):
        now = time.monotonic()
        with self.output_lock:
            conditions_met = (
                self.last_lane_pattern_at is not None
                and now - self.last_lane_pattern_at <= self.lane_pattern_timeout_s
                and self.lane_pattern_tracker.ready(PAIRED_LEFT_MARKING)
            )
            highway_candidate = self.state_latch.update(conditions_met)
            active = exclusive_highway_active(
                highway_candidate, self.intersection_active
            )
            self.publisher.publish(Bool(data=active))
            if active != self.last_output:
                rospy.logwarn(
                    "Highway environment gate changed: active=%s paired_left=%s "
                    "intersection=%s latched=%s",
                    active,
                    conditions_met,
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
