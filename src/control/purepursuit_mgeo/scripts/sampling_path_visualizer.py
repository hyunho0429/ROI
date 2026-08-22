#!/usr/bin/env python3
"""MGeo Link 기반 Lane Change Candidate Path RViz 시각화 노드.

현재 단계:
- 장애물 토픽 사용 안 함
- 제어/Pure Pursuit 변경 안 함
- MGeo에서 실제 좌/우 이동 가능한 차선을 찾음
- 차선변경 시작거리/변경길이를 달리한 실제 주행 후보를 RViz에 표시
"""

from __future__ import annotations

import ast
from typing import List, Optional

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from purepursuit_mgeo.path import PathPoint, load_mgeo_path, nearest_path_index
from sampling_planner import (
    LaneChangeCandidate,
    LinkSpatialIndex,
    generate_lane_change_candidates,
    load_mgeo_links,
)


def _parse_float_list(value, default):
    if value is None:
        return tuple(default)

    if isinstance(value, str):
        value = ast.literal_eval(value)

    return tuple(float(v) for v in value)


class SamplingVisualizer:
    def __init__(self) -> None:
        rospy.init_node("sampling_path_visualizer", anonymous=False)

        path_file = rospy.get_param("~path_file")
        link_set_file = rospy.get_param("~link_set_file")

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.local_length_m = float(
            rospy.get_param("~local_length_m", 45.0)
        )
        self.control_rate_hz = float(
            rospy.get_param("~control_rate_hz", 10.0)
        )
        self.sample_spacing_m = float(
            rospy.get_param("~sample_spacing_m", 0.5)
        )

        self.start_distances_m = _parse_float_list(
            rospy.get_param(
                "~lane_change_start_distances_m",
                [3.0, 7.0, 11.0],
            ),
            [3.0, 7.0, 11.0],
        )

        self.change_lengths_m = _parse_float_list(
            rospy.get_param(
                "~lane_change_lengths_m",
                [15.0, 22.0],
            ),
            [15.0, 22.0],
        )

        self.points: List[PathPoint] = load_mgeo_path(path_file)
        self.links = load_mgeo_links(link_set_file)

        # link_set 전체를 매 주기 brute-force 하지 않도록 1회 spatial index 생성
        self.link_index = LinkSpatialIndex(
            self.links,
            cell_size_m=float(
                rospy.get_param("~link_index_cell_size_m", 10.0)
            ),
            point_stride=int(
                rospy.get_param("~link_index_point_stride", 3)
            ),
        )

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

        self.current_link_pub = rospy.Publisher(
            "~current_link",
            Marker,
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
            "MGeo lane sampling visualizer started: global_points=%d links=%d "
            "local_length=%.1fm starts=%s lengths=%s",
            len(self.points),
            len(self.links),
            self.local_length_m,
            self.start_distances_m,
            self.change_lengths_m,
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def publish_global_path(self) -> None:
        msg = RosPath()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.Time.now()

        for p in self.points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = p.x
            ps.pose.position.y = p.y
            ps.pose.position.z = p.z
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self.global_pub.publish(msg)

    @staticmethod
    def _candidate_color(side: str, variant: int) -> ColorRGBA:
        color = ColorRGBA()
        color.a = 0.95

        # 좌/우 방향을 RViz에서 바로 구분하기 위한 색상
        if side == "left":
            color.r = 0.1
            color.g = 0.75
            color.b = 1.0
        else:
            color.r = 1.0
            color.g = 0.55
            color.b = 0.1

        # 동일 방향 후보끼리도 약간 차이가 보이도록 밝기만 조절
        factor = max(0.55, 1.0 - 0.07 * variant)
        color.r *= factor
        color.g *= factor
        color.b *= factor
        return color

    def _candidate_marker(
        self,
        marker_id: int,
        candidate: LaneChangeCandidate,
        side_variant: int,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lane_change_candidates"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.color = self._candidate_color(
            candidate.side,
            side_variant,
        )

        for p in candidate.path:
            point = Point()
            point.x = p.x
            point.y = p.y
            point.z = p.z + 0.10
            marker.points.append(point)

        return marker

    def _current_link_marker(self, link) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "current_mgeo_link"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25

        marker.color.a = 0.9
        marker.color.r = 0.4
        marker.color.g = 1.0
        marker.color.b = 0.4

        for p in link.points:
            point = Point()
            point.x = p.x
            point.y = p.y
            point.z = p.z + 0.08
            marker.points.append(point)

        return marker

    def timer_callback(self, _event) -> None:
        if self.latest_odom is None:
            rospy.logwarn_throttle(
                5.0,
                "Sampling visualizer가 /localization/odometry를 기다리는 중입니다.",
            )
            return

        pose = self.latest_odom.pose.pose
        ego_x = pose.position.x
        ego_y = pose.position.y

        # Ego 자체가 아니라 현재 Global Path의 nearest point를 MGeo Link와 매칭.
        # 인접 차선/반대방향 Link로 잘못 붙는 것을 줄이기 위함.
        global_nearest = nearest_path_index(
            self.points,
            ego_x,
            ego_y,
        )
        reference = self.points[global_nearest]

        current_link = self.link_index.nearest_link(
            reference.x,
            reference.y,
        )

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if current_link is None:
            self.candidate_pub.publish(markers)
            rospy.logwarn_throttle(
                2.0,
                "현재 Global Path 주변의 MGeo Link를 찾지 못했습니다.",
            )
            return

        self.current_link_pub.publish(
            self._current_link_marker(current_link)
        )

        candidates = generate_lane_change_candidates(
            global_path=self.points,
            ego_x=ego_x,
            ego_y=ego_y,
            current_link=current_link,
            links=self.links,
            start_distances_m=self.start_distances_m,
            change_lengths_m=self.change_lengths_m,
            local_length_m=self.local_length_m,
            sample_spacing_m=self.sample_spacing_m,
        )

        side_counts = {"left": 0, "right": 0}

        for marker_id, candidate in enumerate(candidates):
            side_variant = side_counts[candidate.side]
            side_counts[candidate.side] += 1

            markers.markers.append(
                self._candidate_marker(
                    marker_id,
                    candidate,
                    side_variant,
                )
            )

        self.candidate_pub.publish(markers)

        rospy.loginfo_throttle(
            1.0,
            "Sampling: global_idx=%d current_link=%s lane=%s "
            "left=%s right=%s candidates=%d",
            global_nearest,
            current_link.idx,
            str(current_link.ego_lane),
            str(current_link.left_lane_change_dst_link_idx),
            str(current_link.right_lane_change_dst_link_idx),
            len(candidates),
        )


if __name__ == "__main__":
    try:
        SamplingVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
