#!/usr/bin/env python3
"""Build a stable highway-environment gate from camera and LiDAR states."""

import time

import rospy
from std_msgs.msg import Bool


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
        self.left_parallel_dynamic_topic = _param(
            "left_parallel_dynamic_topic",
            "/perception/lidar/left_lane_parallel_dynamic_detected",
        )
        self.output_topic = _param(
            "output_topic", "/perception/camera/highway_environment"
        )
        self.require_dashed_lane = bool(_param("require_dashed_lane", False))
        self.require_left_parallel_dynamic = bool(
            _param("require_left_parallel_dynamic", False)
        )
        self.car_hold_s = float(_param("car_hold_s", 2.0))
        self.dashed_lane_hold_s = float(_param("dashed_lane_hold_s", 2.0))
        self.left_parallel_dynamic_hold_s = float(
            _param("left_parallel_dynamic_hold_s", 0.5)
        )
        self.publish_rate_hz = float(_param("publish_rate_hz", 10.0))
        if min(
            self.car_hold_s,
            self.dashed_lane_hold_s,
            self.left_parallel_dynamic_hold_s,
        ) <= 0.0:
            raise ValueError("camera condition hold times must be positive")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.last_car_detected_at = None
        self.last_dashed_lane_detected_at = None
        self.last_left_parallel_dynamic_at = None
        self.last_output = None

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
        self.left_parallel_dynamic_subscriber = rospy.Subscriber(
            self.left_parallel_dynamic_topic,
            Bool,
            self._left_parallel_dynamic_callback,
            queue_size=1,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self._timer_callback
        )
        rospy.on_shutdown(self._shutdown)

        rospy.logwarn(
            "Highway gate: car=%s dashed=%s required_dashed=%s "
            "left_parallel_dynamic=%s required_left_parallel_dynamic=%s "
            "output=%s",
            self.car_detected_topic,
            self.dashed_lane_topic,
            self.require_dashed_lane,
            self.left_parallel_dynamic_topic,
            self.require_left_parallel_dynamic,
            self.output_topic,
        )

    def _car_callback(self, message):
        if message.data:
            self.last_car_detected_at = time.monotonic()

    def _dashed_lane_callback(self, message):
        if message.data:
            self.last_dashed_lane_detected_at = time.monotonic()

    def _left_parallel_dynamic_callback(self, message):
        if message.data:
            self.last_left_parallel_dynamic_at = time.monotonic()

    @staticmethod
    def _recent(timestamp, hold_s, now):
        return timestamp is not None and now - timestamp <= hold_s

    def _timer_callback(self, _event):
        now = time.monotonic()
        car_active = self._recent(
            self.last_car_detected_at, self.car_hold_s, now
        )
        dashed_active = self._recent(
            self.last_dashed_lane_detected_at,
            self.dashed_lane_hold_s,
            now,
        )
        left_parallel_dynamic_active = self._recent(
            self.last_left_parallel_dynamic_at,
            self.left_parallel_dynamic_hold_s,
            now,
        )
        active = (
            car_active
            and (dashed_active if self.require_dashed_lane else True)
            and (
                left_parallel_dynamic_active
                if self.require_left_parallel_dynamic
                else True
            )
        )
        self.publisher.publish(Bool(data=active))

        if active != self.last_output:
            rospy.logwarn(
                "Highway environment gate changed: active=%s car=%s "
                "dashed=%s dashed_required=%s left_parallel_dynamic=%s "
                "left_parallel_dynamic_required=%s",
                active,
                car_active,
                dashed_active,
                self.require_dashed_lane,
                left_parallel_dynamic_active,
                self.require_left_parallel_dynamic,
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
