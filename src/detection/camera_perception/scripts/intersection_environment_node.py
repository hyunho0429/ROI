#!/usr/bin/env python3
"""Detect intersections from unified Car and both solid lane boundaries."""

import json
import time

import rospy
from std_msgs.msg import Bool, String

from camera_perception.intersection import IntersectionStateMachine


def _param(name, default):
    return rospy.get_param("~" + name, default)


class IntersectionEnvironmentNode:
    def __init__(self):
        self.car_topic = _param("car_topic", "/perception/camera/car_detected")
        self.left_solid_lane_topic = _param(
            "left_solid_lane_topic",
            "/perception/camera/left_solid_lane_detected",
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
        self.left_solid_lane_detected = False
        self.right_solid_lane_detected = False
        self.left_lane_updated_at = None
        self.right_lane_updated_at = None
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
            self.left_solid_lane_topic,
            Bool,
            self._left_solid_lane_callback,
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
            "Intersection detector: Car=%s AND left_solid=%s AND "
            "right_solid=%s; outputs=%s,%s,%s",
            self.car_topic,
            self.left_solid_lane_topic,
            self.right_solid_lane_topic,
            self.detected_topic,
            self.driving_allowed_topic,
            self.driving_unavailable_topic,
        )

    def _car_callback(self, message):
        self.camera_vehicle_detected = bool(message.data)
        self.camera_updated_at = time.monotonic()

    def _left_solid_lane_callback(self, message):
        self.left_solid_lane_detected = bool(message.data)
        self.left_lane_updated_at = time.monotonic()

    def _right_solid_lane_callback(self, message):
        self.right_solid_lane_detected = bool(message.data)
        self.right_lane_updated_at = time.monotonic()

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
        decision = self.state_machine.update(
            camera_vehicle_detected=self.camera_vehicle_detected,
            left_solid_lane_detected=self.left_solid_lane_detected,
            right_solid_lane_detected=self.right_solid_lane_detected,
            now=now,
            camera_fresh=camera_fresh,
            lane_fresh=lane_fresh,
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
            "recognition_rule": "CAR_AND_LEFT_SOLID_AND_RIGHT_SOLID",
            "camera_car_detected": bool(
                camera_fresh and self.camera_vehicle_detected
            ),
            "left_solid_lane_detected": bool(
                lane_fresh and self.left_solid_lane_detected
            ),
            "right_solid_lane_detected": bool(
                lane_fresh and self.right_solid_lane_detected
            ),
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
                "camera_car=%s | left_solid=%s | right_solid=%s\n"
                "============================================================",
                driving_notice,
                status["camera_car_detected"],
                status["left_solid_lane_detected"],
                status["right_solid_lane_detected"],
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
