#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from mgeo_json_dijkstra import MGeoJsonGraph


def _default_mgeo_dir():
    package_path = rospkg.RosPack().get_path("path_planning")
    return os.path.join(package_path, "mgeo", "R_KR_PR_K-city_2025")


def _xy_param(name):
    value = rospy.get_param(name, None)
    if value is None:
        return None
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",")]
    else:
        parts = value
    if len(parts) < 2:
        raise ValueError("{} must be [x, y] or 'x,y'".format(name))
    return float(parts[0]), float(parts[1])


def _path_msg(points, frame_id):
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()

    for point in points:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2] if len(point) > 2 else 0.0
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)

    return msg


class KCityDijkstraPathPublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.topic = rospy.get_param("~topic", "/global_path")
        self.mgeo_dir = rospy.get_param("~mgeo_dir", _default_mgeo_dir())
        self.graph = MGeoJsonGraph(self.mgeo_dir)

        self.publisher = rospy.Publisher(self.topic, Path, queue_size=1, latch=True)
        self.path = self._build_path()
        self.path_msg = _path_msg(self.path["point_path"], self.frame_id)

        rospy.loginfo(
            "K-City Dijkstra path ready: %.2fm, %d nodes, %d links, %d points",
            self.path["distance"],
            len(self.path["node_path"]),
            len(self.path["link_path"]),
            len(self.path["point_path"]),
        )

    def _build_path(self):
        node_list = rospy.get_param("~node_list", [])
        if node_list:
            path = self.graph.route_through_nodes(node_list)
        else:
            start_node = rospy.get_param("~start_node", None)
            goal_node = rospy.get_param("~goal_node", None)
            start_xy = _xy_param("~start_xy")
            goal_xy = _xy_param("~goal_xy")

            if start_xy is not None:
                start_node, start_dist = self.graph.nearest_node(*start_xy)
                rospy.loginfo("start_xy nearest node: %s (%.2fm)", start_node, start_dist)
            if goal_xy is not None:
                goal_node, goal_dist = self.graph.nearest_node(*goal_xy)
                rospy.loginfo("goal_xy nearest node: %s (%.2fm)", goal_node, goal_dist)

            if not start_node or not goal_node:
                raise rospy.ROSException(
                    "Set ~node_list, or set ~start_node/~goal_node, or set ~start_xy/~goal_xy."
                )
            path = self.graph.shortest_path(start_node, goal_node)

        if not path["success"]:
            raise rospy.ROSException("No Dijkstra route found")
        return path

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.path_msg.header.stamp = rospy.Time.now()
            for pose in self.path_msg.poses:
                pose.header = self.path_msg.header
            self.publisher.publish(self.path_msg)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("global_path_kcity_dijkstra", anonymous=True)
    KCityDijkstraPathPublisher().spin()
