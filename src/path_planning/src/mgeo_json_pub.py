#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import rospkg
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

from mgeo_json_dijkstra import MGeoJsonGraph


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

        self.graph = MGeoJsonGraph(self.mgeo_dir)
        self.node_pub = rospy.Publisher(self.node_topic, PointCloud, queue_size=1, latch=True)
        self.link_pub = rospy.Publisher(self.link_topic, PointCloud, queue_size=1, latch=True)
        self.node_msg = self._build_node_cloud()
        self.link_msg = self._build_link_cloud()

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

    def _build_link_cloud(self):
        msg = PointCloud()
        msg.header.frame_id = self.frame_id
        for link in self.graph.links.values():
            for point in link.get("points") or []:
                msg.points.append(Point32(point[0], point[1], point[2]))
        return msg

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            self.node_msg.header.stamp = stamp
            self.link_msg.header.stamp = stamp
            self.node_pub.publish(self.node_msg)
            self.link_pub.publish(self.link_msg)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mgeo_json_pub", anonymous=True)
    MGeoJsonPublisher().spin()
