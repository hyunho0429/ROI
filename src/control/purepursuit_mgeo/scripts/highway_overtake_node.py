#!/usr/bin/env python3
"""ROS node for the camera-lane highway overtaking planner.

All decisions live in purepursuit_mgeo.highway_overtake (also exercised by
tools/highway_overtake_sim.py); this node only moves data in and out.

Inputs
  /localization/odometry                     nav_msgs/Odometry (map frame)
  /perception/lidar/tracked_obstacles_map    lidar_perception/LidarObstacleArray
  /perception/camera/lane_info               std_msgs/String (lane JSON)
  /avoidance_path_manager/active_path        nav_msgs/Path (passed through outside the zone)
  /avoidance_path_manager/stop_required      std_msgs/Bool

Outputs (~ = /highway_overtake/), the same contract as highway_lane_strategy:
  ~active_path ~stop_required ~target_speed_mps ~active ~fast_change_active
  ~lead_brake_required ~lead_emergency_brake
  ~zone_active   fixed highway-zone flag from the map
  ~state         JSON status (state, reason, gap decision, lead, caps, tracks)
  ~markers       RViz: nearby vehicles coloured by their role
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import fields

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool, Float64, String
from visualization_msgs.msg import Marker, MarkerArray

from lidar_perception.msg import LidarObstacleArray
from purepursuit_mgeo.highway_overtake import (
    HighwayOvertakePlanner, Obstacle, OvertakeConfig, Route,
)
from purepursuit_mgeo.path import load_mgeo_path


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


class HighwayOvertakeNode:
    def __init__(self) -> None:
        rospy.init_node("highway_overtake", anonymous=False)
        self.map_frame = rospy.get_param("~map_frame", "map")
        route = [(p.x, p.y) for p in load_mgeo_path(rospy.get_param("~path_file"))]

        values = {}
        for item in fields(OvertakeConfig):
            if rospy.has_param("~" + item.name):
                values[item.name] = rospy.get_param("~" + item.name)
        cfg = OvertakeConfig.from_mapping(values)
        resolver = Route(route)
        for key in ("zone_start", "zone_end", "entry_taper_start", "entry_taper_end"):
            xy = rospy.get_param("~%s_xy" % key, None)
            if xy:
                setattr(cfg, key + "_s", resolver.s_of_xy(float(xy[0]), float(xy[1])))
        if cfg.zone_end_s <= cfg.zone_start_s:
            raise ValueError("zone_end_xy must lie after zone_start_xy on the route")
        self.planner = HighwayOvertakePlanner(cfg, route)
        self.lock = threading.Lock()

        rospy.Subscriber(rospy.get_param("~odom_topic", "/localization/odometry"),
                         Odometry, self._odom_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~obstacle_topic", "/perception/lidar/tracked_obstacles_map"),
                         LidarObstacleArray, self._obstacles_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~lane_info_topic", "/perception/camera/lane_info"),
                         String, self._lane_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~base_path_topic", "/avoidance_path_manager/active_path"),
                         RosPath, self._base_path_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~base_stop_topic", "/avoidance_path_manager/stop_required"),
                         Bool, self._base_stop_cb, queue_size=1)

        self.path_pub = rospy.Publisher("~active_path", RosPath, queue_size=1)
        self.stop_pub = rospy.Publisher("~stop_required", Bool, queue_size=1)
        self.speed_pub = rospy.Publisher("~target_speed_mps", Float64, queue_size=1)
        self.active_pub = rospy.Publisher("~active", Bool, queue_size=1)
        self.fast_pub = rospy.Publisher("~fast_change_active", Bool, queue_size=1)
        self.lead_brake_pub = rospy.Publisher("~lead_brake_required", Bool, queue_size=1)
        self.lead_emergency_pub = rospy.Publisher("~lead_emergency_brake", Bool, queue_size=1)
        self.zone_pub = rospy.Publisher("~zone_active", Bool, queue_size=1)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1)
        self.publish_markers = bool(rospy.get_param("~publish_markers", True))
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)

        rospy.logwarn(
            "highway_overtake: zone s=%.1f..%.1f (%.0f m), entry taper s=%s..%s, "
            "max %d changes, max speed %.1f m/s, entry angle %.1f deg, lateral accel %.1f",
            cfg.zone_start_s, cfg.zone_end_s, cfg.zone_end_s - cfg.zone_start_s,
            "-" if cfg.entry_taper_start_s is None else "%.1f" % cfg.entry_taper_start_s,
            "-" if cfg.entry_taper_end_s is None else "%.1f" % cfg.entry_taper_end_s,
            cfg.max_changes, cfg.max_speed_mps, cfg.max_entry_angle_deg,
            cfg.max_lateral_accel_mps2)
        rate = float(rospy.get_param("~rate_hz", 20.0))
        self.timer = rospy.Timer(rospy.Duration(1.0/max(rate, 1.0)), self._tick)

    # ----- callbacks (wall-clock stamps, like the camera lane_info) --------
    def _odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        with self.lock:
            self.planner.on_odom(time.time(), pose.position.x, pose.position.y,
                                 yaw_from_quaternion(pose.orientation), speed)

    def _obstacles_cb(self, msg: LidarObstacleArray) -> None:
        obstacles = [Obstacle(int(o.id), float(o.center_x_map), float(o.center_y_map),
                              float(o.velocity_x_map), float(o.velocity_y_map),
                              float(o.length), float(o.width)) for o in msg.obstacles]
        with self.lock:
            self.planner.on_obstacles(time.time(), obstacles)

    def _lane_cb(self, msg: String) -> None:
        try:
            info = json.loads(msg.data)
        except ValueError as exc:
            rospy.logwarn_throttle(2.0, "highway_overtake: lane_info JSON error: %s", exc)
            return
        with self.lock:
            self.planner.on_lane_info(time.time(), info)

    def _base_path_cb(self, msg: RosPath) -> None:
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self.lock:
            self.planner.on_base_path(time.time(), points if len(points) >= 2 else None)

    def _base_stop_cb(self, msg: Bool) -> None:
        with self.lock:
            self.planner.on_base_path(time.time(), None, bool(msg.data))

    # ----- output -----------------------------------------------------------
    def _ros_path(self, points, stamp) -> RosPath:
        msg = RosPath()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        for i, (x, y) in enumerate(points):
            a = points[max(0, i-1)]
            b = points[min(len(points)-1, i+1)]
            yaw = math.atan2(b[1]-a[1], b[0]-a[0])
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(0.5*yaw)
            pose.pose.orientation.w = math.cos(0.5*yaw)
            msg.poses.append(pose)
        return msg

    def _markers(self, status, stamp) -> MarkerArray:
        array = MarkerArray()
        clear = Marker()
        clear.action = 3        # DELETEALL
        array.markers.append(clear)
        gap = status.get("gap") or {}
        blocker = None if gap.get("ok", True) else gap.get("blocker")
        lead = (status.get("follow") or {}).get("lead")
        for k, trk in enumerate(status.get("tracks", [])):
            color = ((1.0, 0.0, 0.0) if trk["id"] == blocker else
                     (1.0, 0.55, 0.0) if trk["id"] == lead else
                     (0.0, 0.8, 1.0) if trk["lane"] == 1 else (0.6, 0.6, 0.6))
            box = Marker()
            box.header.frame_id, box.header.stamp = self.map_frame, stamp
            box.ns, box.id, box.type, box.action = "tracks", 2*k, Marker.CUBE, Marker.ADD
            box.pose.position.x, box.pose.position.y, box.pose.position.z = trk["x"], trk["y"], 0.8
            box.pose.orientation.w = 1.0
            box.scale.x, box.scale.y, box.scale.z = 4.6, 1.9, 1.6
            box.color.r, box.color.g, box.color.b = color
            box.color.a = 0.7
            text = Marker()
            text.header = box.header
            text.ns, text.id, text.type, text.action = "tracks", 2*k+1, Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = trk["x"], trk["y"], 2.6
            text.pose.orientation.w = 1.0
            text.scale.z = 1.0
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = "%d L%+d %.0fkm/h" % (trk["id"], trk["lane"], trk["v"]*3.6)
            array.markers.extend([box, text])
        return array

    def _tick(self, _event) -> None:
        with self.lock:
            out = self.planner.step(time.time())
        stamp = rospy.Time.now()
        if out.path:
            self.path_pub.publish(self._ros_path(out.path, stamp))
        self.stop_pub.publish(Bool(data=bool(out.stop)))
        self.speed_pub.publish(Float64(data=float(out.target_speed)))
        self.active_pub.publish(Bool(data=bool(out.active)))
        self.fast_pub.publish(Bool(data=bool(out.fast_change)))
        self.lead_brake_pub.publish(Bool(data=bool(out.lead_brake)))
        self.lead_emergency_pub.publish(Bool(data=bool(out.lead_emergency)))
        self.zone_pub.publish(Bool(data=bool(out.zone_active)))
        self.state_pub.publish(String(data=json.dumps(out.status, separators=(",", ":"))))
        if self.publish_markers and out.active:
            self.marker_pub.publish(self._markers(out.status, stamp))
        for event in out.events:
            rospy.logwarn("HIGHWAY_OVERTAKE %s", event)
        status = out.status
        rospy.loginfo_throttle(
            1.0, "HIGHWAY_OVERTAKE state=%s reason=%s lane=%s target=%.1fm/s stop=%s gap=%s lead=%s",
            out.state, status.get("reason"), status.get("lane_changes_done"), out.target_speed,
            out.stop, status.get("gap"), (status.get("follow") or {}).get("lead"))


if __name__ == "__main__":
    try:
        HighwayOvertakeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
