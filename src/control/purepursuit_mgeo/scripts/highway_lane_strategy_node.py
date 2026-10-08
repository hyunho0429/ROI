#!/usr/bin/env python3
"""ROS adapter for the single-target Frenet highway controller.

All planning/control decisions live in purepursuit_mgeo.highway. This adapter
aligns camera observations with historical ego poses and publishes rolling
paths; it never sends actuator commands or clears independent mission stops.
"""
import json
import math
import threading
import time
from collections import deque
from dataclasses import replace

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float64, String
from visualization_msgs.msg import Marker
from lidar_perception.msg import LidarObstacleArray
from purepursuit_mgeo.highway import (
    Config, Ego, Highway, Obstacle, guarded_global_path, lane_from_json,
)
from purepursuit_mgeo.lane_geometry import pose_at
from purepursuit_mgeo.path import load_mgeo_path


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class HighwayNode:
    def __init__(self):
        rospy.init_node('highway_lane_strategy')
        cfg = Config(cruise_speed=float(rospy.get_param('~cruise_speed_mps', 80./3.6)),
                     lane_hold_s=float(rospy.get_param('~lane_hold_s', 5.0)))
        self.planner = Highway(cfg)
        self.lock = threading.RLock()
        self.history = deque(maxlen=500)
        self.ego = None
        self.odom_at = self.lidar_at = self.base_at = self.base_stop_at = -math.inf
        self.lidar_stamp = None
        self.obstacles = []
        self.enabled = False
        self.base = None
        self.base_stop = True
        self.frame = rospy.get_param('~map_frame', 'map')
        route_file = rospy.get_param('~global_path_file', '')
        try:
            self.global_route = load_mgeo_path(route_file) if route_file else []
        except (OSError, ValueError) as exc:
            self.global_route = []
            rospy.logwarn('HIGHWAY global route unavailable: %s', exc)
        root = '/highway_lane_strategy/'
        self.path_pub = rospy.Publisher(root+'active_path', Path, queue_size=1)
        self.stop_pub = rospy.Publisher(root+'stop_required', Bool, queue_size=1)
        self.speed_pub = rospy.Publisher(root+'target_speed_mps', Float64, queue_size=1)
        self.active_pub = rospy.Publisher(root+'active', Bool, queue_size=1)
        self.change_pub = rospy.Publisher(root+'fast_change_active', Bool, queue_size=1)
        self.lead_pub = rospy.Publisher(root+'lead_brake_required', Bool, queue_size=1)
        self.emergency_pub = rospy.Publisher(root+'lead_emergency_brake', Bool, queue_size=1)
        self.state_pub = rospy.Publisher(root+'state', String, queue_size=1)
        self.marker_pub = rospy.Publisher(root+'status_marker', Marker, queue_size=1)
        rospy.Subscriber(rospy.get_param('~odom_topic', '/localization/odometry'), Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~obstacle_topic', '/perception/lidar/tracked_obstacles_map'), LidarObstacleArray, self.lidar_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~lane_info_topic', '/perception/camera/lane_info'), String, self.lane_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~highway_topic', '/perception/camera/highway_environment'), Bool, self.enable_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~highway_request_topic', '/planning/highway_lane_change_request'), Bool, self.enable_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~base_path_topic', '/avoidance_path_manager/active_path'), Path, self.base_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~base_stop_topic', '/avoidance_path_manager/stop_required'), Bool, self.stop_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.05), self.tick)
        rospy.logwarn('HIGHWAY single-target Frenet: cruise=%.2f m/s, hold=%.1fs, left-solid final lock enabled', cfg.cruise_speed, cfg.lane_hold_s)

    def measurement_time(self, header):
        if header.stamp.to_sec() <= 0:
            return -math.inf
        age = (rospy.Time.now()-header.stamp).to_sec()
        if age < -.05:
            return -math.inf
        return time.time()-max(0., age)

    def odom_cb(self, msg):
        with self.lock:
            p, v = msg.pose.pose, msg.twist.twist.linear
            values = (p.position.x, p.position.y, yaw(p.orientation), math.hypot(v.x,v.y))
            if not all(math.isfinite(x) for x in values):
                return
            self.ego = Ego(*values)
            self.odom_at = self.measurement_time(msg.header)
            if math.isfinite(self.odom_at):
                self.history.append((self.odom_at, values[:3]))

    def lidar_cb(self, msg):
        with self.lock:
            if msg.header.frame_id and msg.header.frame_id != self.frame:
                rospy.logwarn_throttle(1., 'HIGHWAY rejected LiDAR frame=%s', msg.header.frame_id)
                return
            stamp = msg.header.stamp.to_sec()
            if self.lidar_stamp is not None and stamp <= self.lidar_stamp:
                return
            objects = []
            for o in msg.obstacles:
                values = (o.center_x_map, o.center_y_map, o.yaw, o.length, o.width, o.velocity_x_map, o.velocity_y_map)
                if not all(math.isfinite(v) for v in values) or o.length <= 0 or o.width <= 0:
                    rospy.logwarn_throttle(1., 'HIGHWAY rejected malformed LiDAR frame')
                    return
                objects.append(Obstacle(o.id, *values))
            self.obstacles = objects
            self.lidar_stamp = stamp
            self.lidar_at = self.measurement_time(msg.header)

    def lane_cb(self, msg):
        with self.lock:
            try:
                info = json.loads(msg.data)
                if info.get('observation_time_source') != 'camera_receive_wall':
                    raise ValueError('camera_observation_clock_unknown')
                stamp = float(info['timestamp'])
                pose = pose_at(self.history, stamp, tolerance=.075)
                if pose is None or self.ego is None:
                    raise ValueError('camera_pose_not_synchronized')
                lane = lane_from_json(info, pose, self.planner.cfg)
                self.planner.observe(lane, time.time(), self.ego)
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                self.planner.lane_reason = str(exc)
                rospy.logwarn_throttle(2., 'HIGHWAY camera rejected: %s', exc)

    def enable_cb(self, msg):
        with self.lock:
            self.enabled = self.enabled or bool(msg.data)

    def base_cb(self, msg):
        with self.lock:
            self.base, self.base_at = msg, time.time()

    def stop_cb(self, msg):
        with self.lock:
            self.base_stop, self.base_stop_at = bool(msg.data), time.time()

    def tick(self, _):
        with self.lock:
            now = time.time()
            if self.ego is None:
                self.stop_pub.publish(Bool(True))
                rospy.logwarn_throttle(1., 'HIGHWAY waiting odometry')
                return
            age = max(0., now-self.lidar_at)
            objects = [replace(o, x=o.x+o.vx*min(age,.5), y=o.y+o.vy*min(age,.5)) for o in self.obstacles]
            result = self.planner.step(now, self.ego, objects, self.enabled,
                                       now-self.odom_at, age, self.lidar_stamp)
            active = result.state != 'OFF'
            stop = result.stop
            points = result.path
            path_source = 'lane_change' if self.planner.change is not None else 'lane_center'
            if active and self.planner.changes == 0 and self.planner.change is None and not self.planner.locked:
                mapped, clipped = guarded_global_path(
                    self.global_route, self.ego, self.planner.road,
                    self.planner.lane.width if self.planner.lane is not None else 0.,
                    self.planner.cfg, max(65., self.ego.speed*3.))
                if mapped:
                    points = mapped
                    path_source = 'global_guarded' if clipped else 'global'
            if points:
                path = Path()
                path.header.frame_id = self.frame
                path.header.stamp = rospy.Time.now()
                for x,y in points:
                    p = PoseStamped()
                    p.header = path.header
                    p.pose.position.x, p.pose.position.y = x,y
                    p.pose.orientation.w = 1.
                    path.poses.append(p)
                self.path_pub.publish(path)
            elif self.base is not None and now-self.base_at <= 1.:
                self.path_pub.publish(self.base)
                path_source = 'base'
            if not active:
                stop = self.base_stop or now-self.base_at > 1. or now-self.base_stop_at > 1.
            self.stop_pub.publish(Bool(stop))
            self.speed_pub.publish(Float64(result.target_speed))
            self.active_pub.publish(Bool(active))
            self.change_pub.publish(Bool(result.state in ('CHANGE','SETTLE')))
            self.lead_pub.publish(Bool(bool(result.diagnostics.get('follow'))))
            self.emergency_pub.publish(Bool(result.reason == 'front_collision_emergency'))
            status = dict(state=result.state, reason=result.reason, active=active, stop=stop,
                          path_source=path_source,
                          target_speed_mps=result.target_speed, **result.diagnostics)
            self.state_pub.publish(String(json.dumps(status)))
            label = '%s: %s\npath=%s changes=%d target=%.1f km/h' % (
                result.state, result.reason, path_source,
                self.planner.changes, 3.6*result.target_speed)
            marker = Marker()
            marker.header.frame_id = self.frame
            marker.header.stamp = rospy.Time.now()
            marker.ns, marker.id = 'highway_status', 0
            marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            marker.pose.position.x, marker.pose.position.y = self.ego.x, self.ego.y
            marker.pose.position.z = 4.
            marker.pose.orientation.w = 1.
            marker.scale.z = 1.
            marker.color.r, marker.color.g, marker.color.a = (1. if stop else .2), (0. if stop else 1.), 1.
            marker.text = label
            self.marker_pub.publish(marker)
            rospy.loginfo_throttle(1., 'HIGHWAY state=%s reason=%s path=%s change=%d stop=%s d=%s yaw=%s lane=%s',
                                   result.state,result.reason,path_source,self.planner.changes,stop,
                                   result.diagnostics.get('lateral_error'),result.diagnostics.get('heading_error_deg'),self.planner.lane_reason)


if __name__ == '__main__':
    HighwayNode()
    rospy.spin()
