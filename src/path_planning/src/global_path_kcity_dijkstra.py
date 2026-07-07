#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import rospkg
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path

from mgeo_json_dijkstra import MGeoJsonGraph


def _default_mgeo_dir():
    package_path = rospkg.RosPack().get_path("path_planning")
    return os.path.join(package_path, "mgeo", "R_KR_PR_K-city_2025")


def _xy_param(name):
    value = rospy.get_param(name, None)
    if value is None or value == "":
        return None
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",")]
    else:
        parts = value
    if not parts:
        return None
    if len(parts) < 2:
        raise ValueError("{} must be [x, y] or 'x,y'".format(name))
    return float(parts[0]), float(parts[1])


def _bool_param(name, default=False):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


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
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.clicked_point_topic = rospy.get_param("~clicked_point_topic", "/clicked_point")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.interactive = _bool_param("~interactive", False)
        self.use_odom_start = _bool_param("~use_odom_start", True)
        self.goal_search_count = int(rospy.get_param("~goal_search_count", 20))
        self.mgeo_dir = rospy.get_param("~mgeo_dir", _default_mgeo_dir())
        self.graph = MGeoJsonGraph(self.mgeo_dir)
        self.last_odom_xy = None

        self.publisher = rospy.Publisher(self.topic, Path, queue_size=1, latch=True)
        self.path = None
        self.path_msg = _path_msg([], self.frame_id)

        if self.use_odom_start:
            rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback)

        if self.interactive:
            rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_callback)
            rospy.Subscriber(self.clicked_point_topic, PointStamped, self._clicked_point_callback)
            rospy.loginfo(
                "Waiting for RViz goal on %s or clicked point on %s",
                self.goal_topic,
                self.clicked_point_topic,
            )

        if not self.interactive or self._has_initial_goal_params():
            self._plan_from_params()

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

    def _has_initial_goal_params(self):
        goal_node = rospy.get_param("~goal_node", None)
        goal_xy = rospy.get_param("~goal_xy", None)
        return (
            bool(rospy.get_param("~node_list", []))
            or bool(goal_node)
            or goal_xy not in (None, "")
        )

    def _plan_from_params(self):
        self._set_path(self._build_path(), "parameter goal")

    def _odom_callback(self, msg):
        self.last_odom_xy = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def _goal_callback(self, msg):
        self._plan_to_xy(msg.pose.position.x, msg.pose.position.y, "RViz 2D Nav Goal")

    def _clicked_point_callback(self, msg):
        self._plan_to_xy(msg.point.x, msg.point.y, "RViz clicked point")

    def _resolve_start_node(self):
        if self.use_odom_start and self.last_odom_xy is not None:
            start_node, distance = self.graph.nearest_node(*self.last_odom_xy)
            rospy.loginfo("start from odom nearest node: %s (%.2fm)", start_node, distance)
            return start_node

        start_xy = _xy_param("~start_xy")
        if start_xy is not None:
            start_node, distance = self.graph.nearest_node(*start_xy)
            rospy.loginfo("start from param xy nearest node: %s (%.2fm)", start_node, distance)
            return start_node

        start_node = rospy.get_param("~start_node", None)
        if start_node:
            return start_node

        raise rospy.ROSException(
            "No start pose. Publish /odom or set ~start_node/~start_xy."
        )

    def _plan_to_xy(self, goal_x, goal_y, source):
        try:
            start_node = self._resolve_start_node()
            path = None
            selected_goal_node = None
            selected_goal_distance = None
            goal_candidates = self.graph.nearest_nodes(
                goal_x,
                goal_y,
                limit=max(1, self.goal_search_count),
            )
            for goal_node, goal_distance in goal_candidates:
                candidate_path = self.graph.shortest_path(start_node, goal_node)
                if candidate_path["success"]:
                    path = candidate_path
                    selected_goal_node = goal_node
                    selected_goal_distance = goal_distance
                    break

            if path is None:
                rospy.logwarn(
                    "No Dijkstra route from %s to any of the nearest %d goal nodes",
                    start_node,
                    len(goal_candidates),
                )
                return

            rospy.loginfo(
                "%s selected goal node: %s (%.2fm from requested point)",
                source,
                selected_goal_node,
                selected_goal_distance,
            )
            self._set_path(path, source)
        except Exception as exc:
            rospy.logerr("Failed to build Dijkstra path: %s", exc)

    def _set_path(self, path, source):
        self.path = path
        self.path_msg = _path_msg(path["point_path"], self.frame_id)
        rospy.loginfo(
            "K-City Dijkstra path updated from %s: %.2fm, %d nodes, %d links, %d points",
            source,
            path["distance"],
            len(path["node_path"]),
            len(path["link_path"]),
            len(path["point_path"]),
        )

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
