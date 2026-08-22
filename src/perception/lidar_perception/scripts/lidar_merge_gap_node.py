#!/usr/bin/env python3
"""Evaluate adjacent-lane insertion space from tracked LiDAR bounding boxes."""

import json
import math
import time

import rospy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from lidar_perception.lidar_merge_gap import (
    MergeGapTracker,
    assess_tracked_merge_gaps,
    format_tracked_merge_gap_status,
)


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _json_safe(value):
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class MergeGapNode:
    def __init__(self):
        self.input_topic = _param(
            "input_topic", "/morai/lidar/tracking/results"
        )
        self.result_topic = _param(
            "result_topic", "/morai/lidar/merge_gap/results"
        )
        self.marker_topic = _param(
            "marker_topic", "/morai/lidar/merge_gap/markers"
        )
        self.available_topic = _param(
            "available_topic", "/perception/merge_gap/available"
        )
        self.unavailable_topic = _param(
            "unavailable_topic", "/perception/merge_gap/unavailable"
        )
        self.highway_gate_required = bool(
            _param("highway_gate_required", False)
        )
        self.highway_gate_topic = _param(
            "highway_gate_topic", "/perception/camera/highway_environment"
        )
        self.highway_gate_timeout_s = float(
            _param("highway_gate_timeout_s", 0.5)
        )
        self.frame_id = _param("frame_id", "morai_lidar")
        self.vehicle_length_m = float(_param("vehicle_length_m", 4.635))
        self.vehicle_width_m = float(_param("vehicle_width_m", 1.892))
        self.vehicle_height_m = float(_param("vehicle_height_m", 2.434))
        self.lane_width_m = float(_param("lane_width_m", 3.5))
        self.lane_lateral_allowance_m = float(
            _param("lane_lateral_allowance_m", 0.4)
        )
        self.longitudinal_margin_m = float(
            _param("longitudinal_margin_m", 1.0)
        )
        self.lateral_margin_m = float(_param("lateral_margin_m", 0.2))
        self.detection_range_m = float(_param("detection_range_m", 40.0))
        self.time_headway_s = float(_param("time_headway_s", 1.5))
        self.minimum_ttc_s = float(_param("minimum_ttc_s", 3.0))
        self.stale_timeout_s = float(_param("stale_timeout_s", 0.5))
        self.log_interval_s = float(_param("log_interval_s", 1.0))
        self.marker_lifetime_s = float(_param("marker_lifetime_s", 0.25))
        confirmation_scans = int(_param("confirmation_scans", 3))
        if self.stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be positive")
        if self.highway_gate_timeout_s <= 0.0:
            raise ValueError("highway_gate_timeout_s must be positive")
        self.confirmation = MergeGapTracker(confirmation_scans)
        self.last_input_at = None
        self.stale_published = False
        self.highway_gate_value = not self.highway_gate_required
        self.last_highway_gate_at = None
        self.outputs_active = False

        self.result_publisher = rospy.Publisher(
            self.result_topic, String, queue_size=1
        )
        self.marker_publisher = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1
        )
        self.available_publisher = rospy.Publisher(
            self.available_topic, Bool, queue_size=1
        )
        self.unavailable_publisher = rospy.Publisher(
            self.unavailable_topic, Bool, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            String,
            self._tracking_callback,
            queue_size=1,
        )
        self.highway_gate_subscriber = None
        if self.highway_gate_required:
            self.highway_gate_subscriber = rospy.Subscriber(
                self.highway_gate_topic,
                Bool,
                self._highway_gate_callback,
                queue_size=1,
            )
        self.stale_timer = rospy.Timer(
            rospy.Duration(min(0.2, 0.5 * self.stale_timeout_s)),
            self._stale_callback,
        )
        rospy.logwarn(
            "\n"
            "================ MERGE SPACE MONITOR STARTED ================\n"
            "input=%s | ego=%.3fx%.3fx%.3fm | lane=%.2fm\n"
            "headway=%.2fs | min_ttc=%.2fs | confirmation=%d scans\n"
            "AVAILABLE means perception-only clearance, not lane-change control.\n"
            "===============================================================",
            self.input_topic,
            self.vehicle_length_m,
            self.vehicle_width_m,
            self.vehicle_height_m,
            self.lane_width_m,
            self.time_headway_s,
            self.minimum_ttc_s,
            confirmation_scans,
        )
        rospy.logwarn(
            "LEFT merge state outputs: available=%s unavailable=%s "
            "highway_gate_required=%s gate_topic=%s",
            self.available_topic,
            self.unavailable_topic,
            self.highway_gate_required,
            self.highway_gate_topic,
        )
        if not self.highway_gate_required:
            self._publish_binary_states()

    def _publish_binary_states(self, assessments=None):
        left_available = bool(
            assessments is not None
            and assessments["left"]["confirmed_available"]
        )
        available = Bool(data=left_available)
        unavailable = Bool(data=not left_available)
        self.available_publisher.publish(available)
        self.unavailable_publisher.publish(unavailable)

    def _highway_gate_active(self):
        if not self.highway_gate_required:
            return True
        if not self.highway_gate_value or self.last_highway_gate_at is None:
            return False
        return (
            time.monotonic() - self.last_highway_gate_at
            <= self.highway_gate_timeout_s
        )

    def _highway_gate_callback(self, message):
        was_active = self._highway_gate_active()
        self.highway_gate_value = bool(message.data)
        self.last_highway_gate_at = time.monotonic()
        is_active = self._highway_gate_active()

        if is_active and not was_active:
            self.confirmation = MergeGapTracker(
                self.confirmation.confirmation_scans
            )
            self.stale_published = False
            rospy.logwarn(
                "Highway environment detected: LEFT merge-gap perception enabled"
            )
        elif was_active and not is_active:
            self._deactivate_outputs("highway_environment_inactive")
            rospy.logwarn(
                "Highway environment lost: LEFT merge-gap perception disabled"
            )

    def _deactivate_outputs(self, reason):
        if not self.outputs_active:
            return
        self.confirmation = MergeGapTracker(
            self.confirmation.confirmation_scans
        )
        # Clear a previously published AVAILABLE state once for safety, then
        # remain silent until the highway gate becomes active again.
        self._publish_invalid(reason)
        self.outputs_active = False

    def _tracking_callback(self, message):
        self.last_input_at = time.monotonic()
        self.stale_published = False
        if not self._highway_gate_active():
            self._deactivate_outputs("highway_environment_inactive")
            return
        try:
            tracks = json.loads(message.data)
            if not isinstance(tracks, list):
                raise ValueError("tracking result must be a JSON list")
            assessments = assess_tracked_merge_gaps(
                tracks=tracks,
                vehicle_length_m=self.vehicle_length_m,
                vehicle_width_m=self.vehicle_width_m,
                vehicle_height_m=self.vehicle_height_m,
                lane_width_m=self.lane_width_m,
                lane_lateral_allowance_m=self.lane_lateral_allowance_m,
                longitudinal_margin_m=self.longitudinal_margin_m,
                lateral_margin_m=self.lateral_margin_m,
                detection_range_m=self.detection_range_m,
                time_headway_s=self.time_headway_s,
                minimum_ttc_s=self.minimum_ttc_s,
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            rospy.logwarn_throttle(1.0, "Invalid LiDAR tracking result: %s", error)
            self._publish_invalid("invalid_tracking_result")
            self.outputs_active = True
            return

        assessments, became_available, became_unavailable = self.confirmation.update(
            assessments
        )
        payload = {
            "valid": True,
            "algorithm": "euclidean_bbox_kalman_hungarian_dynamic_gap",
            "left": assessments["left"],
        }
        if self.highway_gate_required:
            payload["highway_environment"] = True
        self.result_publisher.publish(
            String(data=json.dumps(_json_safe(payload), ensure_ascii=False))
        )
        self._publish_binary_states(assessments)
        self.marker_publisher.publish(self._markers(assessments))
        self.outputs_active = True

        left_text = format_tracked_merge_gap_status(assessments["left"])
        # WARN level is intentional: ROS terminals render this periodic status in
        # a more visible color than ordinary INFO output.
        rospy.logwarn_throttle(
            self.log_interval_s,
            "\n"
            "===================== MERGE SPACE =====================\n"
            "LEFT  | %s\n"
            "=======================================================",
            left_text,
        )
        if "left" in became_available:
            rospy.logwarn(
                "\n"
                "#######################################################\n"
                ">>> MERGE SPACE AVAILABLE: %s <<<\n"
                "%s\n"
                "#######################################################",
                "LEFT",
                left_text,
            )
        if "left" in became_unavailable:
            rospy.logwarn(
                "\n"
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
                ">>> MERGE SPACE LOST/BLOCKED: %s <<<\n"
                "%s\n"
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!",
                "LEFT",
                left_text,
            )

    def _stale_callback(self, _event):
        if not self._highway_gate_active():
            self._deactivate_outputs("highway_environment_inactive_or_stale")
            if self.highway_gate_required:
                rospy.loginfo_throttle(
                    2.0,
                    "Merge-gap perception is idle until %s is true",
                    self.highway_gate_topic,
                )
            return
        if self.last_input_at is None:
            rospy.logwarn_throttle(
                2.0, "Merge-gap node is waiting for %s", self.input_topic
            )
            return
        if time.monotonic() - self.last_input_at <= self.stale_timeout_s:
            return
        if not self.stale_published:
            self.confirmation = MergeGapTracker(
                self.confirmation.confirmation_scans
            )
            self._publish_invalid("tracking_stale")
            self.stale_published = True
        rospy.logwarn_throttle(
            2.0,
            "\n"
            "!!!!!!!!!!!!!!!! MERGE SPACE INVALID !!!!!!!!!!!!!!!!\n"
            "Tracking input stale for more than %.2fs; do not merge.\n"
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!",
            self.stale_timeout_s,
        )

    def _publish_invalid(self, reason):
        payload = {
            "valid": False,
            "algorithm": "euclidean_bbox_kalman_hungarian_dynamic_gap",
            "reason": reason,
            "left": {"confirmed_available": False},
        }
        self.result_publisher.publish(String(data=json.dumps(payload)))
        self._publish_binary_states()
        delete = Marker()
        delete.action = Marker.DELETEALL
        self.marker_publisher.publish(MarkerArray(markers=[delete]))

    def _markers(self, assessments):
        stamp = rospy.Time.now()
        marker_array = MarkerArray()
        delete = Marker()
        delete.header.stamp = stamp
        delete.header.frame_id = self.frame_id
        delete.action = Marker.DELETEALL
        marker_array.markers.append(delete)

        ego = self._base_marker(stamp, "merge_gap_ego", 0, Marker.CUBE)
        ego.pose.position.z = 0.5 * self.vehicle_height_m
        ego.scale.x = self.vehicle_length_m
        ego.scale.y = self.vehicle_width_m
        ego.scale.z = self.vehicle_height_m
        ego.color.r, ego.color.g, ego.color.b, ego.color.a = 0.2, 0.5, 1.0, 0.35
        marker_array.markers.append(ego)

        for index, side in enumerate(("left",)):
            assessment = assessments[side]
            if assessment["confirmed_available"]:
                color = (0.1, 1.0, 0.2, 0.32)
                state = "AVAILABLE"
            elif assessment["available"]:
                color = (1.0, 0.85, 0.1, 0.32)
                state = "CHECKING"
            else:
                color = (1.0, 0.15, 0.1, 0.32)
                state = "BLOCKED"

            rear = assessment["rear_boundary_m"]
            front = assessment["front_boundary_m"]
            corridor = self._base_marker(
                stamp, "merge_gap_corridor", index, Marker.CUBE
            )
            corridor.pose.position.x = 0.5 * (rear + front)
            corridor.pose.position.y = assessment["lane_center_y_m"]
            corridor.pose.position.z = 0.05
            corridor.scale.x = max(0.1, front - rear)
            corridor.scale.y = max(0.1, self.vehicle_width_m)
            corridor.scale.z = 0.1
            (
                corridor.color.r,
                corridor.color.g,
                corridor.color.b,
                corridor.color.a,
            ) = color
            marker_array.markers.append(corridor)

            label = self._base_marker(
                stamp, "merge_gap_labels", index, Marker.TEXT_VIEW_FACING
            )
            label.pose.position.x = 0.0
            label.pose.position.y = assessment["lane_center_y_m"]
            label.pose.position.z = 2.8
            label.scale.z = 0.55
            label.color.r, label.color.g, label.color.b, label.color.a = color
            label.color.a = 1.0
            label.text = "{} {}\n{}".format(
                side.upper(), state, assessment["reason"]
            )
            marker_array.markers.append(label)
        return marker_array

    def _base_marker(self, stamp, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(self.marker_lifetime_s)
        return marker


if __name__ == "__main__":
    try:
        rospy.init_node("lidar_merge_gap")
        MergeGapNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
