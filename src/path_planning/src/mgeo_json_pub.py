#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import rospkg
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker

from path_planning.mgeo_json_dijkstra import MGeoJsonGraph


def _default_mgeo_dir():
    package_path = rospkg.RosPack().get_path("path_planning")
    return os.path.join(package_path, "mgeo", "R_KR_PR_K-city_2025")


class MGeoJsonPublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 1.0))
        self.mgeo_dir = rospy.get_param("~mgeo_dir", _default_mgeo_dir())
        self.node_topic = rospy.get_param("~node_topic", "node")
        self.link_topic = rospy.get_param("~link_topic", "link")
        self.node_marker_topic = rospy.get_param("~node_marker_topic", "mgeo_nodes_marker")
        self.link_marker_topic = rospy.get_param("~link_marker_topic", "mgeo_links_marker")

        self.graph = MGeoJsonGraph(self.mgeo_dir)
        self.node_pub = rospy.Publisher(self.node_topic, PointCloud, queue_size=1, latch=True)
        self.link_pub = rospy.Publisher(self.link_topic, PointCloud, queue_size=1, latch=True)
        self.node_marker_pub = rospy.Publisher(
            self.node_marker_topic, Marker, queue_size=1, latch=True
        )
        self.link_marker_pub = rospy.Publisher(
            self.link_marker_topic, Marker, queue_size=1, latch=True
        )
        self.node_msg = self._build_node_cloud()
        self.link_msg = self._build_link_cloud()
        self.node_marker_msg = self._build_node_marker()
        self.link_marker_msg = self._build_link_marker()

        rospy.loginfo(
            "K-City MGeo loaded: %d nodes, %d links",
            len(self.graph.nodes),
            len(self.graph.links),
        )

    def _build_node_cloud(self):
        msg = PointCloud()
        msg.header.frame_id = self.frame_id
        for node in self.graph.nodes.values():
            point = node["point"]
            msg.points.append(Point32(point[0], point[1], point[2]))
        return msg

    def _point(self, point, z_offset=0.0):
        msg = Point()
        msg.x = point[0]
        msg.y = point[1]
        msg.z = point[2] + z_offset if len(point) > 2 else z_offset
        return msg

    def _build_link_cloud(self):
        msg = PointCloud()
        msg.header.frame_id = self.frame_id
        for link in self.graph.links.values():
            for point in link.get("points") or []:
                msg.points.append(Point32(point[0], point[1], point[2]))
        return msg

    def _base_marker(self, marker_id, namespace, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _build_node_marker(self):
        marker = self._base_marker(0, "kcity_nodes", Marker.SPHERE_LIST)
        marker.scale.x = 1.2
        marker.scale.y = 1.2
        marker.scale.z = 1.2
        marker.color.r = 0.25
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 1.0
        for node in self.graph.nodes.values():
            marker.points.append(self._point(node["point"], z_offset=0.5))
        return marker

    def _build_link_marker(self):
        marker = self._base_marker(0, "kcity_links", Marker.LINE_LIST)
        marker.scale.x = 0.35
        marker.color.r = 0.92
        marker.color.g = 0.92
        marker.color.b = 0.92
        marker.color.a = 0.8
        for link in self.graph.links.values():
            points = link.get("points") or []
            for index in range(1, len(points)):
                marker.points.append(self._point(points[index - 1], z_offset=0.2))
                marker.points.append(self._point(points[index], z_offset=0.2))
        return marker

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            self.node_msg.header.stamp = stamp
            self.link_msg.header.stamp = stamp
            self.node_marker_msg.header.stamp = stamp
            self.link_marker_msg.header.stamp = stamp
            self.node_pub.publish(self.node_msg)
            self.link_pub.publish(self.link_msg)
            self.node_marker_pub.publish(self.node_marker_msg)
            self.link_marker_pub.publish(self.link_marker_msg)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mgeo_json_pub", anonymous=True)
    MGeoJsonPublisher().spin()
