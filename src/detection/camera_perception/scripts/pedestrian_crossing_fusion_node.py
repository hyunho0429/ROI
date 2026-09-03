#!/usr/bin/env python3
"""Publish pedestrian stop/resume from the YOLO person Bool state only.

The filename and ROS node name are retained for launch compatibility. LiDAR
obstacle topics, distance thresholds, and odometry are intentionally not part
of this decision path.
"""

import json
import threading
import time

import rospy
from std_msgs.msg import Bool, String

from camera_perception.pedestrian_crossing import PedestrianStopStateMachine


def _param(name, default):
    return rospy.get_param("~" + name, default)


class PedestrianCrossingCameraNode:
    def __init__(self):
        self.person_detected_topic = _param(
            "person_detected_topic", "/perception/camera/person_detected"
        )
        self.stop_topic = _param(
            "stop_topic", "/perception/pedestrian_crossing/stop_required"
        )
        self.resume_topic = _param(
            "resume_topic", "/perception/pedestrian_crossing/resume_allowed"
        )
        self.status_topic = _param(
            "status_topic", "/perception/pedestrian_crossing/status"
        )

        self.input_stale_timeout_s = float(
            _param("input_stale_timeout_s", 0.5)
        )
        self.publish_rate_hz = float(_param("publish_rate_hz", 20.0))
        if self.input_stale_timeout_s <= 0.0:
            raise ValueError("input_stale_timeout_s must be positive")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        # A positive camera result stops immediately. A short continuous clear
        # period prevents one missed YOLO frame from restarting the car.
        self.state_machine = PedestrianStopStateMachine(
            stop_confirmation_s=0.0,
            clear_confirmation_s=float(
                _param("person_clear_confirmation_s", 0.5)
            ),
        )
        self.person_detected = False
        self.last_person_message_at = None
        self.lock = threading.Lock()
        self.evaluation_lock = threading.Lock()

        self.stop_publisher = rospy.Publisher(
            self.stop_topic, Bool, queue_size=1
        )
        self.resume_publisher = rospy.Publisher(
            self.resume_topic, Bool, queue_size=1
        )
        self.status_publisher = rospy.Publisher(
            self.status_topic, String, queue_size=1
        )
        self.person_subscriber = rospy.Subscriber(
            self.person_detected_topic,
            Bool,
            self._person_callback,
            queue_size=1,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self._timer_callback
        )
        rospy.on_shutdown(self._shutdown)

        rospy.logwarn(
            "Pedestrian camera stop: person=%s outputs stop=%s resume=%s "
            "(LiDAR condition disabled)",
            self.person_detected_topic,
            self.stop_topic,
            self.resume_topic,
        )

    def _person_callback(self, message):
        now = time.monotonic()
        with self.lock:
            self.person_detected = bool(message.data)
            self.last_person_message_at = now
        # Evaluate in the subscriber callback so person=true is not delayed by
        # the periodic status timer.
        self._evaluate_and_publish(now)

    def _evaluate_and_publish(self, now):
        # ROS subscriber and Timer callbacks may run concurrently. Serialize
        # state transitions and their publications so an older DRIVE result
        # can never be published after a newer STOP result.
        with self.evaluation_lock:
            self._evaluate_and_publish_locked(now)

    def _evaluate_and_publish_locked(self, now):
        with self.lock:
            input_age_s = (
                now - self.last_person_message_at
                if self.last_person_message_at is not None
                else None
            )
            input_fresh = (
                input_age_s is not None
                and input_age_s <= self.input_stale_timeout_s
            )
            person_detected = input_fresh and self.person_detected
            decision = self.state_machine.update(
                now=now,
                inputs_ready=input_fresh,
                trigger_hazard=person_detected,
                clear_for_resume=input_fresh and not person_detected,
            )

        self.stop_publisher.publish(Bool(data=decision.stop_required))
        self.resume_publisher.publish(Bool(data=decision.resume_allowed))
        status = {
            "mode": "CAMERA_ONLY",
            "state": "STOP" if decision.stop_required else "DRIVE",
            "inputs_ready": input_fresh,
            "person_detected": person_detected,
            "person_input_age_s": input_age_s,
            "transition": decision.transition,
        }
        self.status_publisher.publish(
            String(
                data=json.dumps(
                    status, ensure_ascii=False, separators=(",", ":")
                )
            )
        )

        if decision.transition == "STOP":
            rospy.logerr("PEDESTRIAN STOP: YOLO person detected")
        elif decision.transition == "RESUME":
            rospy.logwarn(
                "PEDESTRIAN CLEAR: YOLO person absent; resume global path"
            )

    def _timer_callback(self, _event):
        self._evaluate_and_publish(time.monotonic())

    def _shutdown(self):
        # If this safety node disappears while the controller remains alive,
        # leave the controller with a fail-safe stop request.
        self.stop_publisher.publish(Bool(data=True))
        self.resume_publisher.publish(Bool(data=False))


if __name__ == "__main__":
    try:
        rospy.init_node("pedestrian_crossing_fusion")
        PedestrianCrossingCameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
