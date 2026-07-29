#!/usr/bin/env python3
"""Publish MORAI VLP-16 UDP LiDAR front-FOV points as ROS PointCloud2.

This node does not use MORAI ROS sensor topics.  It receives the competition
UDP LiDAR packet directly, decodes the Velodyne/VLP-16 payload, transforms it
to an ego-local frame, samples only a configured front field-of-view, and
publishes the result for RViz.

Frame convention of the published cloud:
    +x: forward, +y: left, +z: up
"""

import math
import socket
from collections import deque

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

try:
    from path_planning.morai_competition_config import BIND_IP
except ImportError:
    BIND_IP = "0.0.0.0"

try:
    from path_planning.morai_competition_config import LIDAR_HOST_PORT, LIDAR_PORT
except ImportError:
    LIDAR_HOST_PORT = 2000
    LIDAR_PORT = 2001

from path_planning.morai_udp_lidar import (
    LidarPacketError,
    parse_lidar_intensity_packet,
)


POINT_FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("intensity", 12, PointField.FLOAT32, 1),
    PointField("ring", 16, PointField.FLOAT32, 1),
    PointField("bearing_deg", 20, PointField.FLOAT32, 1),
]


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _rotate_xy(x_forward, y_left, yaw_offset_deg):
    yaw_rad = math.radians(yaw_offset_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        cos_yaw * x_forward - sin_yaw * y_left,
        sin_yaw * x_forward + cos_yaw * y_left,
    )


def _sample_front_points(points, params):
    sampled = []
    for point in points:
        if point.distance_m < params["min_distance_m"]:
            continue

        x_forward, y_left = _rotate_xy(
            point.x_m,
            point.y_m,
            params["lidar_yaw_offset_deg"],
        )
        z_up = point.z_m
        bearing_deg = math.degrees(math.atan2(y_left, x_forward))

        if not (-params["fov_right_deg"] <= bearing_deg <= params["fov_left_deg"]):
            continue
        if not (params["x_min_m"] <= x_forward <= params["x_max_m"]):
            continue
        if abs(y_left) > params["y_abs_m"]:
            continue
        if not (params["z_min_m"] <= z_up <= params["z_max_m"]):
            continue

        sampled.append(
            (
                float(x_forward),
                float(y_left),
                float(z_up),
                float(point.intensity),
                float(point.laser_id),
                float(bearing_deg),
            )
        )
    return sampled


def _make_cloud(points, frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return point_cloud2.create_cloud(header, POINT_FIELDS, points)


def main():
    rospy.init_node("morai_lidar_pointcloud_udp")

    params = {
        "bind_ip": _param("bind_ip", BIND_IP),
        "host_port": int(_param("host_port", LIDAR_HOST_PORT)),
        "destination_port": int(_param("destination_port", LIDAR_PORT)),
        "frame_id": _param("frame_id", "morai_lidar"),
        "topic": _param("topic", "/morai/lidar/front_points"),
        "packets_per_cloud": int(_param("packets_per_cloud", 80)),
        "rolling_clouds": int(_param("rolling_clouds", 3)),
        "socket_timeout_s": float(_param("socket_timeout_s", 1.0)),
        "lidar_yaw_offset_deg": float(_param("lidar_yaw_offset_deg", 0.0)),
        "fov_left_deg": float(_param("fov_left_deg", 90.0)),
        "fov_right_deg": float(_param("fov_right_deg", 90.0)),
        "x_min_m": float(_param("x_min_m", 0.3)),
        "x_max_m": float(_param("x_max_m", 40.0)),
        "y_abs_m": float(_param("y_abs_m", 8.0)),
        "z_min_m": float(_param("z_min_m", -2.5)),
        "z_max_m": float(_param("z_max_m", 3.0)),
        "min_distance_m": float(_param("min_distance_m", 0.2)),
    }

    if params["packets_per_cloud"] < 1:
        raise ValueError("~packets_per_cloud must be at least 1")
    if params["rolling_clouds"] < 1:
        raise ValueError("~rolling_clouds must be at least 1")
    if params["fov_left_deg"] < 0.0 or params["fov_right_deg"] < 0.0:
        raise ValueError("FOV limits cannot be negative")
    if params["fov_left_deg"] > 180.0 or params["fov_right_deg"] > 180.0:
        raise ValueError("FOV limits cannot exceed 180 degrees")

    publisher = rospy.Publisher(params["topic"], PointCloud2, queue_size=1)

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    udp_socket.bind((params["bind_ip"], params["destination_port"]))
    udp_socket.settimeout(params["socket_timeout_s"])

    rospy.loginfo(
        "MORAI LiDAR PointCloud2 UDP: source *:%d -> %s:%d, topic=%s, "
        "frame=%s, FOV=-%.1f..+%.1f deg, rolling_clouds=%d",
        params["host_port"],
        params["bind_ip"],
        params["destination_port"],
        params["topic"],
        params["frame_id"],
        params["fov_right_deg"],
        params["fov_left_deg"],
        params["rolling_clouds"],
    )

    rolling_clouds = deque(maxlen=params["rolling_clouds"])

    try:
        while not rospy.is_shutdown():
            cloud_points = []
            packets = 0
            bad_packets = 0
            while packets < params["packets_per_cloud"] and not rospy.is_shutdown():
                try:
                    packet, sender = udp_socket.recvfrom(65535)
                except socket.timeout:
                    rospy.logwarn_throttle(
                        2.0,
                        "No LiDAR UDP packet received within %.1fs",
                        params["socket_timeout_s"],
                    )
                    break

                if sender[1] != params["host_port"]:
                    rospy.logwarn_throttle(
                        5.0,
                        "LiDAR UDP sender source port %d, expected %d",
                        sender[1],
                        params["host_port"],
                    )

                packets += 1
                try:
                    lidar_packet = parse_lidar_intensity_packet(packet)
                except (LidarPacketError, ValueError) as error:
                    bad_packets += 1
                    rospy.logwarn_throttle(2.0, "Bad LiDAR packet: %s", error)
                    continue

                cloud_points.extend(_sample_front_points(lidar_packet.points, params))

            if cloud_points:
                rolling_clouds.append(cloud_points)
            accumulated_points = [
                point
                for cloud in rolling_clouds
                for point in cloud
            ]
            if accumulated_points:
                publisher.publish(_make_cloud(accumulated_points, params["frame_id"]))
            rospy.loginfo_throttle(
                1.0,
                "LiDAR cloud: packets=%d bad=%d sampled_points=%d accumulated_points=%d",
                packets,
                bad_packets,
                len(cloud_points),
                len(accumulated_points),
            )
    finally:
        udp_socket.close()


if __name__ == "__main__":
    main()
