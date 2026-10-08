#!/usr/bin/env python3
"""Activate the highway mission on the first YOLO road-vehicle detection."""

import threading
import time

import rospy
from std_msgs.msg import Bool

from camera_perception.highway_environment import (
    HighwayEnvironmentLatch,
    exclusive_highway_active,
)


def _param(name, default):
    return rospy.get_param("~" + name, default)


class HighwayEnvironmentGateNode:
    def __init__(self):
        self.car_detected_topic = _param(
            "car_detected_topic", "/perception/camera/car_detected"
        )
        self.output_topic = _param(
            "output_topic", "/perception/camera/highway_environment"
        )
        self.intersection_detected_topic = _param(
            "intersection_detected_topic", "/perception/intersection/detected"
        )
        self.latch_once = bool(_param("latch_once", True))
        self.car_timeout_s = float(_param("car_timeout_s", 1.0))
        self.publish_rate_hz = float(_param("publish_rate_hz", 10.0))
        if self.car_timeout_s <= 0.0 or self.publish_rate_hz <= 0.0:
            raise ValueError("highway gate timeout and rate must be positive")

        self.output_lock = threading.Lock()
        self.state_latch = HighwayEnvironmentLatch(self.latch_once)
        self.car_detected = False
        self.last_car_msg_at = None
        self.intersection_active = False
        self.last_output = None

        self.publisher = rospy.Publisher(self.output_topic, Bool, queue_size=1)
        self.car_subscriber = rospy.Subscriber(
            self.car_detected_topic, Bool, self._car_callback, queue_size=1
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
            "Highway gate: YOLO vehicle=%s intersection_override=%s "
            "latch_once=%s output=%s",
            self.car_detected_topic,
            self.intersection_detected_topic,
            self.latch_once,
            self.output_topic,
        )

    def _publish_locked(self):
        active = exclusive_highway_active(
            self.state_latch.latched if self.latch_once else self.car_detected,
            self.intersection_active,
        )
        self.publisher.publish(Bool(data=active))
        if active != self.last_output:
            rospy.logwarn(
                "Highway environment gate changed: active=%s yolo_vehicle=%s "
                "intersection=%s latched=%s",
                active,
                self.car_detected,
                self.intersection_active,
                self.state_latch.latched,
            )
            self.last_output = active

    def _car_callback(self, message):
        with self.output_lock:
            self.car_detected = bool(message.data)
            self.last_car_msg_at = time.monotonic()
            self.state_latch.update(self.car_detected)
            # Publish here: the first positive YOLO result activates immediately.
            self._publish_locked()

    def _intersection_callback(self, message):
        with self.output_lock:
            self.intersection_active = bool(message.data)
            self._publish_locked()

    def _timer_callback(self, _event):
        with self.output_lock:
            if (not self.latch_once and self.last_car_msg_at is not None
                    and time.monotonic() - self.last_car_msg_at > self.car_timeout_s):
                self.car_detected = False
                self.state_latch.update(False)
            self._publish_locked()

    def _shutdown(self):
        self.publisher.publish(Bool(data=False))


if __name__ == "__main__":
    try:
        rospy.init_node("highway_environment_gate")
        HighwayEnvironmentGateNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
