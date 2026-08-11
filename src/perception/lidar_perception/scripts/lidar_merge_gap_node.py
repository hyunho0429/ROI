#!/usr/bin/env python3
"""Evaluate adjacent-lane insertion space from tracked LiDAR bounding boxes."""

import json
import math
import time

import rospy
from std_msgs.msg import String
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
        self.confirmation = MergeGapTracker(confirmation_scans)
        self.last_input_at = None
        self.stale_published = False

        self.result_publisher = rospy.Publisher(
            self.result_topic, String, queue_size=1
        )
        self.marker_publisher = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            String,
            self._tracking_callback,
            queue_size=1,
        )
        self.stale_timer = rospy.Timer(
            rospy.Duration(min(0.2, 0.5 * self.stale_timeout_s)),
            self._stale_callback,
        )
        rospy.logwarn(
            "LiDAR merge-gap perception only: input=%s ego=%.3fx%.3fx%.3fm "
            "lane=%.2fm headway=%.2fs min_ttc=%.2fs confirmation=%d",
            self.input_topic,
            self.vehicle_length_m,
            self.vehicle_width_m,
            self.vehicle_height_m,
            self.lane_width_m,
            self.time_headway_s,
            self.minimum_ttc_s,
            confirmation_scans,
        )

    def _tracking_callback(self, message):
        self.last_input_at = time.monotonic()
        self.stale_published = False
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
            return

        assessments, became_available, became_unavailable = self.confirmation.update(
            assessments
        )
        payload = {
            "valid": True,
            "algorithm": "euclidean_bbox_kalman_hungarian_dynamic_gap",
            "left": assessments["left"],
            "right": assessments["right"],
        }
        self.result_publisher.publish(
            String(data=json.dumps(_json_safe(payload), ensure_ascii=False))
        )
        self.marker_publisher.publish(self._markers(assessments))

        left_text = format_tracked_merge_gap_status(assessments["left"])
        right_text = format_tracked_merge_gap_status(assessments["right"])
        rospy.loginfo_throttle(
            self.log_interval_s,
            "MERGE_GAP TRACKED | %s | %s",
            left_text,
            right_text,
        )
        for side in became_available:
            rospy.logwarn(
                "MERGE_GAP AVAILABLE: %s",
                format_tracked_merge_gap_status(assessments[side]),
            )
        for side in became_unavailable:
            rospy.logwarn(
                "MERGE_GAP LOST: %s",
                format_tracked_merge_gap_status(assessments[side]),
            )

    def _stale_callback(self, _event):
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
            "MERGE_GAP INVALID: tracking input stale for more than %.2fs",
            self.stale_timeout_s,
        )

    def _publish_invalid(self, reason):
        payload = {
            "valid": False,
            "algorithm": "euclidean_bbox_kalman_hungarian_dynamic_gap",
            "reason": reason,
            "left": {"confirmed_available": False},
            "right": {"confirmed_available": False},
        }
        self.result_publisher.publish(String(data=json.dumps(payload)))
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

        for index, side in enumerate(("left", "right")):
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
