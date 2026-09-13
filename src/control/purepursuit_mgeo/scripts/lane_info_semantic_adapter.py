#!/usr/bin/env python3
"""Republish lane semantics from /perception/camera/lane_info JSON.

This avoids running a second LaneDetector / UDP camera consumer.  It preserves
live_overlay.py's semantic topic definitions:
  - dashed_lane_detected: ego-left lane is dashed
  - left_solid_lane_detected: ego-left lane type is white_solid or yellow
  - left_yellow_solid_lane_detected: ego-left lane type is yellow
  - right_solid_lane_detected: ego-right lane type is white_solid or yellow
  - stopline_detected / stopline_distance_m

Camera-team source files are not modified.
"""
import json
import math
import threading

import rospy
from std_msgs.msg import Bool, Float64, String


class LaneInfoSemanticAdapter:
    def __init__(self):
        self.input_topic = rospy.get_param("~lane_info_topic", "/perception/camera/lane_info")
        self.dashed_topic = rospy.get_param("~dashed_lane_topic", "/perception/camera/dashed_lane_detected")
        self.left_solid_topic = rospy.get_param("~left_solid_lane_topic", "/perception/camera/left_solid_lane_detected")
        self.left_yellow_topic = rospy.get_param("~left_yellow_solid_lane_topic", "/perception/camera/left_yellow_solid_lane_detected")
        self.right_solid_topic = rospy.get_param("~right_solid_lane_topic", "/perception/camera/right_solid_lane_detected")
        self.stopline_detected_topic = rospy.get_param("~stopline_detected_topic", "/perception/camera/stopline_detected")
        self.stopline_distance_topic = rospy.get_param("~stopline_distance_topic", "/perception/camera/stopline_distance_m")
        self.stale_timeout_s = float(rospy.get_param("~stale_timeout_s", 0.75))

        self.pub_dashed = rospy.Publisher(self.dashed_topic, Bool, queue_size=1)
        self.pub_left_solid = rospy.Publisher(self.left_solid_topic, Bool, queue_size=1)
        self.pub_left_yellow = rospy.Publisher(self.left_yellow_topic, Bool, queue_size=1)
        self.pub_right_solid = rospy.Publisher(self.right_solid_topic, Bool, queue_size=1)
        self.pub_stopline_detected = rospy.Publisher(self.stopline_detected_topic, Bool, queue_size=1)
        self.pub_stopline_distance = rospy.Publisher(self.stopline_distance_topic, Float64, queue_size=1)

        self.lock = threading.Lock()
        self.last_msg_time = None
        self.stale_published = False

        rospy.Subscriber(self.input_topic, String, self._cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self._stale_timer)

        rospy.logwarn(
            "Lane-info semantic adapter: input=%s -> dashed=%s left_solid=%s left_yellow=%s right_solid=%s stopline=%s",
            self.input_topic,
            self.dashed_topic,
            self.left_solid_topic,
            self.left_yellow_topic,
            self.right_solid_topic,
            self.stopline_detected_topic,
        )

    @staticmethod
    def _lane(info, key):
        lane = info.get(key)
        return lane if isinstance(lane, dict) else {}

    def _publish(self, dashed, left_solid, left_yellow, right_solid, stopline_detected, stopline_distance):
        self.pub_dashed.publish(Bool(data=bool(dashed)))
        self.pub_left_solid.publish(Bool(data=bool(left_solid)))
        self.pub_left_yellow.publish(Bool(data=bool(left_yellow)))
        self.pub_right_solid.publish(Bool(data=bool(right_solid)))
        self.pub_stopline_detected.publish(Bool(data=bool(stopline_detected)))
        self.pub_stopline_distance.publish(Float64(data=float(stopline_distance)))

    def _publish_false(self):
        self._publish(False, False, False, False, False, float("nan"))

    def _cb(self, msg):
        try:
            info = json.loads(msg.data)
            if not isinstance(info, dict):
                raise ValueError("lane_info root is not an object")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "lane_info semantic adapter JSON parse failed: %s", exc)
            return

        left = self._lane(info, "left_lane")
        right = self._lane(info, "right_lane")

        left_detected = bool(left.get("detected", False))
        right_detected = bool(right.get("detected", False))
        left_type = left.get("type")
        right_type = right.get("type")

        # Match live_overlay.py exactly.
        dashed = left_detected and bool(left.get("dashed", False))
        left_solid = left_detected and left_type in ("white_solid", "yellow")
        left_yellow = left_detected and left_type == "yellow"
        right_solid = right_detected and right_type in ("white_solid", "yellow")

        stopline_detected = bool(info.get("stopline_detected", False))
        raw_distance = info.get("stopline_distance_m")
        try:
            stopline_distance = float(raw_distance) if stopline_detected and raw_distance is not None else float("nan")
            if stopline_detected and not math.isfinite(stopline_distance):
                stopline_distance = float("nan")
        except (TypeError, ValueError):
            stopline_distance = float("nan")

        self._publish(
            dashed,
            left_solid,
            left_yellow,
            right_solid,
            stopline_detected,
            stopline_distance,
        )

        with self.lock:
            self.last_msg_time = rospy.Time.now()
            self.stale_published = False

    def _stale_timer(self, _event):
        with self.lock:
            last = self.last_msg_time
            already = self.stale_published

        if last is None:
            return

        age = (rospy.Time.now() - last).to_sec()
        if age > self.stale_timeout_s and not already:
            self._publish_false()
            with self.lock:
                self.stale_published = True
            rospy.logwarn("lane_info stale %.2fs -> semantic topics forced false", age)


if __name__ == "__main__":
    rospy.init_node("lane_info_semantic_adapter", anonymous=False)
    LaneInfoSemanticAdapter()
    rospy.spin()
