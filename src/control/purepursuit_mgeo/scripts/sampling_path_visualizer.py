#!/usr/bin/env python3
"""Sampling Candidate Path를 RViz에 시각화하는 ROS 노드.

현재 단계에서는 장애물/회피/제어와 연결하지 않는다.
Global Path 주변에 생성된 Candidate Path만 RViz에서 확인한다.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path as RosPath
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from purepursuit_mgeo.path import PathPoint, load_mgeo_path
from sampling_planner import (
    DEFAULT_OFFSETS_M,
    generate_candidate_paths,
)


class SamplingVisualizer:
    def __init__(self) -> None:
        rospy.init_node("sampling_path_visualizer", anonymous=False)

        path_file = rospy.get_param("~path_file")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.local_length_m = float(
            rospy.get_param("~local_length_m", 30.0)
        )
        self.control_rate_hz = float(
            rospy.get_param("~control_rate_hz", 10.0)
        )

        offsets = rospy.get_param(
            "~candidate_offsets_m",
            list(DEFAULT_OFFSETS_M),
        )
        self.offsets = tuple(float(v) for v in offsets)

        self.points: List[PathPoint] = load_mgeo_path(path_file)
        self.latest_odom: Optional[Odometry] = None

        self.global_pub = rospy.Publisher(
            "~global_path",
            RosPath,
            queue_size=1,
            latch=True,
        )

        self.candidate_pub = rospy.Publisher(
            "~candidate_paths",
            MarkerArray,
            queue_size=1,
        )

        self.odom_sub = rospy.Subscriber(
            "/localization/odometry",
            Odometry,
            self.odom_callback,
            queue_size=1,
        )

        self.publish_global_path()

        rospy.Timer(
            rospy.Duration(1.0 / max(self.control_rate_hz, 1.0)),
            self.timer_callback,
        )

        rospy.loginfo(
            "Sampling visualizer started: points=%d offsets=%s length=%.1fm",
            len(self.points),
            self.offsets,
            self.local_length_m,
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def publish_global_path(self) -> None:
        msg = RosPath()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.Time.now()

        for p in self.points:
            pose = msg.poses
            from geometry_msgs.msg import PoseStamped
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = p.x
            ps.pose.position.y = p.y
            ps.pose.position.z = p.z
            ps.pose.orientation.w = 1.0
            pose.append(ps)

        self.global_pub.publish(msg)

    @staticmethod
    def _marker_color(offset: float) -> ColorRGBA:
        # RViz 구분을 위한 색상. 값은 고정하지 않고 기본 RGBA만 사용한다.
        # 실제 시각적 구분은 namespace/id 및 선 두께로도 가능하다.
        color = ColorRGBA()
        color.a = 1.0

        if abs(offset) < 1e-6:
            color.r = 1.0
            color.g = 1.0
            color.b = 1.0
        elif offset < 0:
            color.r = 1.0
            color.g = 0.6
            color.b = 0.0
        else:
            color.r = 0.0
            color.g = 0.8
            color.b = 1.0

        return color

    def _make_marker(
        self,
        marker_id: int,
        offset: float,
        path: List[PathPoint],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "sampling_candidates"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.12 if abs(offset) > 1e-6 else 0.20
        marker.color = self._marker_color(offset)

        for p in path:
            point = Point()
            point.x = p.x
            point.y = p.y
            point.z = p.z + 0.05
            marker.points.append(point)

        return marker

    def timer_callback(self, _event) -> None:
        if self.latest_odom is None:
            rospy.logwarn_throttle(
                5.0,
                "Sampling visualizer가 /localization/odometry를 기다리는 중이다.",
            )
            return

        pose = self.latest_odom.pose.pose

        candidates = generate_candidate_paths(
            self.points,
            pose.position.x,
            pose.position.y,
            self.offsets,
            self.local_length_m,
        )

        markers = MarkerArray()

        # 이전 marker를 지우기 위한 DELETEALL
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for marker_id, offset in enumerate(sorted(candidates.keys())):
            path = candidates[offset]

            if len(path) < 2:
                continue

            markers.markers.append(
                self._make_marker(
                    marker_id,
                    offset,
                    path,
                )
            )

        self.candidate_pub.publish(markers)

        nearest_index = self._nearest_index(
            pose.position.x,
            pose.position.y,
        )

        rospy.loginfo_throttle(
            1.0,
            "Sampling: nearest=%d candidates=%d offsets=%s",
            nearest_index,
            len(candidates),
            self.offsets,
        )

    def _nearest_index(self, x: float, y: float) -> int:
        from purepursuit_mgeo.path import nearest_path_index
        return nearest_path_index(self.points, x, y)


if __name__ == "__main__":
    try:
        SamplingVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
