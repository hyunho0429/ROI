#!/usr/bin/env python3
"""Fuse YOLO person state and tracked LiDAR obstacles for crosswalk stops."""

import json
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from camera_perception.pedestrian_crossing import (
    PedestrianStopStateMachine,
    pedestrian_lidar_candidates,
    quaternion_to_yaw,
)


def _param(name, default):
    return rospy.get_param("~" + name, default)


class PedestrianCrossingFusionNode:
    def __init__(self):
        self.person_detected_topic = _param(
            "person_detected_topic", "/perception/camera/person_detected"
        )
        self.all_obstacles_topic = _param(
            "all_obstacles_topic", "/detection/obstacle_states"
        )
        self.dynamic_obstacles_topic = _param(
            "dynamic_obstacles_topic", "/detection/dynamic_obstacles"
        )
        self.odometry_topic = _param(
            "odometry_topic", "/localization/odometry"
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

        self.detection_distance_m = float(
            _param("detection_distance_m", 1.5)
        )
        self.rear_allowance_m = float(_param("rear_allowance_m", 0.5))
        self.max_object_length_m = float(
            _param("max_object_length_m", 1.5)
        )
        self.max_object_width_m = float(
            _param("max_object_width_m", 1.5)
        )
        self.person_hold_s = float(_param("person_hold_s", 0.5))
        self.input_stale_timeout_s = float(
            _param("input_stale_timeout_s", 0.5)
        )
        self.publish_rate_hz = float(_param("publish_rate_hz", 10.0))
        if self.detection_distance_m <= 0.0:
            raise ValueError("detection_distance_m must be positive")
        if self.input_stale_timeout_s <= 0.0:
            raise ValueError("input_stale_timeout_s must be positive")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.state_machine = PedestrianStopStateMachine(
            stop_confirmation_s=float(_param("stop_confirmation_s", 0.2)),
            clear_confirmation_s=float(_param("clear_confirmation_s", 1.0)),
        )
        self.last_person_message_at = None
        self.last_person_detected_at = None
        self.last_all_obstacles_at = None
        self.last_dynamic_obstacles_at = None
        self.last_odometry_at = None
        self.all_obstacles = []
        self.dynamic_obstacles = []
        self.ego_pose = None

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
        self.all_obstacles_subscriber = rospy.Subscriber(
            self.all_obstacles_topic,
            String,
            self._all_obstacles_callback,
            queue_size=1,
        )
        self.dynamic_obstacles_subscriber = rospy.Subscriber(
            self.dynamic_obstacles_topic,
            String,
            self._dynamic_obstacles_callback,
            queue_size=1,
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
            "Pedestrian fusion: person=%s dynamic=%s all=%s distance=%.2fm "
            "outputs stop=%s resume=%s",
            self.person_detected_topic,
            self.dynamic_obstacles_topic,
            self.all_obstacles_topic,
            self.detection_distance_m,
            self.stop_topic,
            self.resume_topic,
        )

    def _person_callback(self, message):
        now = time.monotonic()
        self.last_person_message_at = now
        if message.data:
            self.last_person_detected_at = now

    @staticmethod
    def _parse_obstacles(message):
        payload = json.loads(message.data)
        obstacles = payload.get("obstacles", [])
        if not isinstance(obstacles, list):
            raise ValueError("obstacles must be a list")
        return obstacles

    def _all_obstacles_callback(self, message):
        try:
            self.all_obstacles = self._parse_obstacles(message)
            self.last_all_obstacles_at = time.monotonic()
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError) as error:
            rospy.logwarn_throttle(1.0, "Invalid all-obstacle state: %s", error)

    def _dynamic_obstacles_callback(self, message):
        try:
            self.dynamic_obstacles = self._parse_obstacles(message)
            self.last_dynamic_obstacles_at = time.monotonic()
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError) as error:
            rospy.logwarn_throttle(1.0, "Invalid dynamic-obstacle state: %s", error)

    def _odometry_callback(self, message):
        pose = message.pose.pose
        self.ego_pose = (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        self.last_odometry_at = time.monotonic()

    def _fresh(self, timestamp, now):
        return (
            timestamp is not None
            and now - timestamp <= self.input_stale_timeout_s
        )

    def _candidates(self, obstacles):
        if self.ego_pose is None:
            return []
        return pedestrian_lidar_candidates(
            obstacles=obstacles,
            ego_x=self.ego_pose[0],
            ego_y=self.ego_pose[1],
            ego_yaw=self.ego_pose[2],
            detection_distance_m=self.detection_distance_m,
            rear_allowance_m=self.rear_allowance_m,
            max_object_length_m=self.max_object_length_m,
            max_object_width_m=self.max_object_width_m,
        )

    def _timer_callback(self, _event):
        now = time.monotonic()
        person_input_fresh = self._fresh(self.last_person_message_at, now)
        person_detected = (
            person_input_fresh
            and self.last_person_detected_at is not None
            and now - self.last_person_detected_at <= self.person_hold_s
        )
        inputs_ready = all(
            (
                person_input_fresh,
                self._fresh(self.last_all_obstacles_at, now),
                self._fresh(self.last_dynamic_obstacles_at, now),
                self._fresh(self.last_odometry_at, now),
            )
        )

        dynamic_candidates = self._candidates(self.dynamic_obstacles)
        all_candidates = self._candidates(self.all_obstacles)
        trigger_hazard = person_detected and bool(dynamic_candidates)
        clear_for_resume = not person_detected and not all_candidates
        decision = self.state_machine.update(
            now=now,
            inputs_ready=inputs_ready,
            trigger_hazard=trigger_hazard,
            clear_for_resume=clear_for_resume,
        )

        self.stop_publisher.publish(Bool(data=decision.stop_required))
        self.resume_publisher.publish(Bool(data=decision.resume_allowed))
        status = {
            "state": "STOP" if decision.stop_required else "DRIVE",
            "inputs_ready": inputs_ready,
            "person_detected": person_detected,
            "dynamic_candidate_count": len(dynamic_candidates),
            "nearby_lidar_count": len(all_candidates),
            "nearest_distance_m": (
                all_candidates[0]["distance_m"] if all_candidates else None
            ),
            "transition": decision.transition,
        }
        self.status_publisher.publish(
            String(data=json.dumps(status, ensure_ascii=False, separators=(",", ":")))
        )

        if decision.transition == "STOP":
            rospy.logerr(
                "PEDESTRIAN STOP: camera person + %d dynamic LiDAR candidate(s)",
                len(dynamic_candidates),
            )
        elif decision.transition == "RESUME":
            rospy.logwarn(
                "PEDESTRIAN CLEAR: camera person absent and LiDAR zone clear; resume path"
            )

    def _shutdown(self):
        # If the fusion process disappears while the controller remains alive,
        # leave it with a fail-safe stop request.
        self.stop_publisher.publish(Bool(data=True))
        self.resume_publisher.publish(Bool(data=False))


if __name__ == "__main__":
    try:
        rospy.init_node("pedestrian_crossing_fusion")
        PedestrianCrossingFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
