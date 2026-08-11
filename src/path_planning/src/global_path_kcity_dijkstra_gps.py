#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import rospkg
import subprocess
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

# MORAI 차량 상태 메시지 
from morai_msgs.msg import GPSMessage
import pyproj
from path_planning.mgeo_json_dijkstra import MGeoJsonGraph


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


def _path_marker_msg(points, frame_id):
    msg = Marker()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    msg.ns = "kcity_dijkstra_path"
    msg.id = 0
    msg.type = Marker.LINE_STRIP
    
    if len(points) < 2:
        msg.action = Marker.DELETE
        return msg
        
    msg.action = Marker.ADD
    msg.pose.orientation.w = 1.0
    msg.scale.x = 2.0
    msg.color.r = 0.0
    msg.color.g = 1.0
    msg.color.b = 0.05
    msg.color.a = 1.0

    for point in points:
        marker_point = Point()
        marker_point.x = point[0]
        marker_point.y = point[1]
        marker_point.z = point[2] + 3.0 if len(point) > 2 else 3.0
        msg.points.append(marker_point)

    return msg


class KCityDijkstraPathPublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.topic = rospy.get_param("~topic", "/global_path")
        self.marker_topic = rospy.get_param("~marker_topic", "/global_path_marker")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.clicked_point_topic = rospy.get_param("~clicked_point_topic", "/clicked_point")
        
        # [수정 핵심] 글로벌 파라미터와 프라이빗 파라미터가 꼬이는 것을 방지하기 위해 명시적으로 수동 바인딩 처리
        if rospy.has_param("~odom_topic"):
            self.ego_topic = rospy.get_param("~odom_topic")
        else:
            self.ego_topic = "/Ego_topic"
            
        rospy.loginfo(f"[CONFIG] Subscribing vehicle status target topic -> {self.ego_topic}")
        
        rospy.loginfo("RViz를 자동으로 엽니다...")
        
        # 저장해둔 .rviz 설정 파일 경로 (경로가 다르면 수정해주세요)
        rviz_config_path = os.path.expanduser("~/catkin_ws/src/ROI/src/path_planning/rviz/default.rviz")
        
        try:
            # 설정 파일이 존재하면 그 설정대로 RViz 실행
            if os.path.exists(rviz_config_path):
                subprocess.Popen(["rviz", "-d", rviz_config_path])
            else:
                # 설정 파일이 없으면 기본 빈 창으로 실행
                subprocess.Popen(["rviz"])
        except Exception as e:
            rospy.logwarn(f"RViz 실행 실패: {e}")
        
        self.interactive = _bool_param("~interactive", True) 
        self.goal_search_count = int(rospy.get_param("~goal_search_count", 20))
        self.mgeo_dir = rospy.get_param("~mgeo_dir", _default_mgeo_dir())
        self.graph = MGeoJsonGraph(self.mgeo_dir)

        self.converter = pyproj.Transformer.from_crs(
            "EPSG:4326",
            "EPSG:32652",
            always_xy=True
        )
        self.origin_easting = 302595.0
        self.origin_northing = 4124145.0
        self.last_odom_xy = None

        self.publisher = rospy.Publisher(self.topic, Path, queue_size=1, latch=True)
        self.marker_publisher = rospy.Publisher(self.marker_topic, Marker, queue_size=1, latch=True)
        
        self.path_msg = _path_msg([], self.frame_id)
        self.path_marker_msg = _path_marker_msg([], self.frame_id)

        # 차량 실시간 상태 토픽 수신 등록
        rospy.Subscriber("/gps", GPSMessage, self._gps_callback)
        
        if self.interactive:
            rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_callback)
            rospy.Subscriber(self.clicked_point_topic, PointStamped, self._clicked_point_callback)
            rospy.loginfo("Waiting for RViz 2D Nav Goal...")

    def _build_path(self, goal_node):
        start_node = self._resolve_start_node() 
        if not start_node or not goal_node:
            raise rospy.ROSException("Start node or Goal node missing.")
            
        path = self.graph.shortest_path(start_node, goal_node)
        if not path["success"]:
            raise rospy.ROSException("No Dijkstra route found")
        return path

    def _gps_callback(self, msg):
        easting, northing = self.converter.transform(
            msg.longitude,
            msg.latitude
        )

        x = easting - self.origin_easting
        y = northing - self.origin_northing

        if self.last_odom_xy is None:
            rospy.loginfo("⚡ GPS connected!")

        self.last_odom_xy = (x, y)

    def _goal_callback(self, msg):
        self._plan_to_xy(msg.pose.position.x, msg.pose.position.y, "RViz 2D Nav Goal")

    def _clicked_point_callback(self, msg):
        self._plan_to_xy(msg.point.x, msg.point.y, "RViz clicked point")

    def _resolve_start_node(self):
        # 실시간 수신된 정보가 있으면 최우선으로 매핑하여 시작 노드로 삼음
        if self.last_odom_xy is not None:
            start_node, distance = self.graph.nearest_node(*self.last_odom_xy)
            rospy.loginfo(f"DYNAMIC START -> Node: {start_node} ({distance:.2f}m)")
            return start_node

        # 에러를 던져서 멈추게 하지 않고, 데이터 수신 대기 상태임을 명확히 경고 로그로 표현
        raise rospy.ROSException("GPS data has not been received yet. Please check /gps topic.")

    def _plan_to_xy(self, goal_x, goal_y, source):
        try:
            path = None
            selected_goal_node = None
            selected_goal_distance = None
            goal_candidates = self.graph.nearest_nodes(
                goal_x,
                goal_y,
                limit=max(1, self.goal_search_count),
            )
            for goal_node, goal_distance in goal_candidates:
                candidate_path = self._build_path(goal_node=goal_node)
                if candidate_path["success"]:
                    path = candidate_path
                    selected_goal_node = goal_node
                    selected_goal_distance = goal_distance
                    break

            if path is None:
                rospy.logwarn("No Dijkstra route found to nearby goal nodes.")
                return

            self._set_path(path, source)
        except Exception as exc:
            rospy.logerr("Failed to build Dijkstra path: %s", exc)

    def _set_path(self, path, source):
        self.path_msg = _path_msg(path["point_path"], self.frame_id)
        self.path_marker_msg = _path_marker_msg(path["point_path"], self.frame_id)
        rospy.loginfo(f"Path calculated successfully from {source}! Distance: {path['distance']:.2f}m")

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.path_msg.header.stamp = rospy.Time.now()
            for pose in self.path_msg.poses:
                pose.header = self.path_msg.header
            self.path_marker_msg.header.stamp = self.path_msg.header.stamp
            
            self.publisher.publish(self.path_msg)
            self.marker_publisher.publish(self.path_marker_msg)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("global_path_kcity_dijkstra", anonymous=True)
    KCityDijkstraPathPublisher().spin()
