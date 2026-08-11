#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import math
import pyproj
import tf

from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from morai_msgs.msg import GPSMessage
from morai_msgs.msg import CtrlCmd

from pure_pursuit import PurePursuitController


############################################################
# GPS Converter
############################################################

class GPSConverter:

    def __init__(self):

        # WGS84 -> UTM52
        self.wgs84 = pyproj.CRS("EPSG:4326")
        self.utm = pyproj.CRS("EPSG:32652")

        self.transformer = pyproj.Transformer.from_crs(
            self.wgs84,
            self.utm,
            always_xy=True
        )

        # MORAI K-City Origin
        self.origin_easting = 302595.0
        self.origin_northing = 4124145.0

    ########################################################

    def wgs84_to_local(self, longitude, latitude):

        east, north = self.transformer.transform(
            longitude,
            latitude
        )

        local_x = east - self.origin_easting
        local_y = north - self.origin_northing

        return local_x, local_y


############################################################
# Pure Pursuit Node
############################################################

class PurePursuitNode:

    def __init__(self):

        rospy.init_node("pure_pursuit_node")

        ####################################################
        # Parameters
        ####################################################

        self.lookahead = rospy.get_param(
            "~lookahead_distance",
            3.0
        )

        self.wheel_base = rospy.get_param(
            "~wheel_base",
            1.04
        )

        speed_kph = rospy.get_param(
            "~speed",
            8.0
        )

        self.velocity = speed_kph / 1
        

        ####################################################
        # Controller
        ####################################################

        self.pp = PurePursuitController(
            wheel_base=self.wheel_base,
            lookahead_distance=self.lookahead
        )

        ####################################################
        # GPS Converter
        ####################################################

        self.converter = GPSConverter()

        ####################################################
        # Vehicle State
        ####################################################

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        ####################################################
        # Flags
        ####################################################

        self.is_gps = False
        self.is_imu = False
        self.is_path = False

        ####################################################
        # Publisher
        ####################################################

        self.ctrl_pub = rospy.Publisher(
            "/ctrl_cmd",
            CtrlCmd,
            queue_size=1
        )

        ####################################################
        # Subscriber
        ####################################################

        rospy.Subscriber(
            "/gps",
            GPSMessage,
            self.gps_callback
        )

        rospy.Subscriber(
            "/imu",
            Imu,
            self.imu_callback
        )

        rospy.Subscriber(
            "/global_path",
            Path,
            self.path_callback
        )

        rospy.loginfo("Pure Pursuit Node Started")

    ########################################################
    # GPS Callback
    ########################################################

    def gps_callback(self, msg):

        self.current_x, self.current_y = \
            self.converter.wgs84_to_local(
                msg.longitude,
                msg.latitude
            )

        self.is_gps = True

    
    ########################################################
    # IMU Callback
    ########################################################

    def imu_callback(self, msg):

        q = msg.orientation

        quaternion = (
            q.x,
            q.y,
            q.z,
            q.w
        )

        roll, pitch, yaw = \
            tf.transformations.euler_from_quaternion(
                quaternion
            )

        self.current_yaw = yaw

        self.is_imu = True

    ########################################################
    # Path Callback
    ########################################################

    def path_callback(self, msg):

        path = []

        for pose in msg.poses:

            x = pose.pose.position.x
            y = pose.pose.position.y

            path.append((x, y))

        self.pp.update_path(path)

        self.is_path = True


    ########################################################
    # Publish Control
    ########################################################

    def publish_control(self):

        if not self.is_gps:
            return

        if not self.is_imu:
            return

        if not self.is_path:
            return

        ####################################################
        # Update Vehicle State
        ####################################################

        self.pp.update_pose(
            self.current_x,
            self.current_y,
            self.current_yaw
        )

        ####################################################
        # Pure Pursuit
        ####################################################

        delta = self.pp.pure_pursuit()

        ####################################################
        # CtrlCmd
        ####################################################

        ctrl_msg = CtrlCmd()

        # Velocity Control
        ctrl_msg.longlCmdType = 2

        ctrl_msg.velocity = self.velocity

        ctrl_msg.accel = 0.0
        ctrl_msg.brake = 0.0

        # Steering
        ctrl_msg.steering = delta
        ctrl_msg.rear_steer = 0.0

        ctrl_msg.acceleration = 0.0

        self.ctrl_pub.publish(ctrl_msg)
    ########################################################
    # Main Loop
    ########################################################

    def run(self):

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            self.publish_control()

            rate.sleep()


############################################################
# Main
############################################################

def main():

    node = PurePursuitNode()

    node.run()


if __name__ == "__main__":

    main()
