#!/usr/bin/env python3
"""Publish MORAI VLP-16 UDP LiDAR sampled-FOV points as ROS PointCloud2.

This node does not use MORAI ROS sensor topics.  It receives the competition
UDP LiDAR packet directly, decodes the Velodyne/VLP-16 payload, transforms it
to an ego-local frame, samples a configured field-of-view, and
publishes the result for RViz.

Frame convention of the published cloud:
    +x: forward, +y: left, +z: up
"""

import math
import socket
import time
from collections import deque

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Header

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
    PointField("distance_m", 12, PointField.FLOAT32, 1),
    PointField("intensity", 16, PointField.FLOAT32, 1),
    PointField("ring", 20, PointField.FLOAT32, 1),
    PointField("bearing_deg", 24, PointField.FLOAT32, 1),
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


def _is_in_sampled_area(x_forward, y_left, bearing_deg, params):
    if params["rear_blind_deg"] > 0.0:
        rear_delta = abs(abs(bearing_deg) - 180.0)
        if rear_delta <= 0.5 * params["rear_blind_deg"]:
            return False
    if not (-params["fov_right_deg"] <= bearing_deg <= params["fov_left_deg"]):
        return False
    if not (params["x_min_m"] <= x_forward <= params["x_max_m"]):
        return False
    if abs(y_left) > params["y_abs_m"]:
        return False
    return True


def _sample_points(points, params):
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

        if not _is_in_sampled_area(x_forward, y_left, bearing_deg, params):
            continue
        if not (params["z_min_m"] <= z_up <= params["z_max_m"]):
            continue

        sampled.append(
            (
                float(x_forward),
                float(y_left),
                float(z_up),
                float(point.distance_m),
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


def _nearest_distance(points, params):
    candidates = [
        point
        for point in points
        if params["nearest_x_min_m"] <= point[0] <= params["nearest_x_max_m"]
        and abs(point[1]) <= params["nearest_y_abs_m"]
        and params["nearest_z_min_m"] <= point[2] <= params["nearest_z_max_m"]
    ]
    if not candidates:
        return None
    return min(point[3] for point in candidates)


def main():
    rospy.init_node("morai_lidar_pointcloud_udp")

    params = {
        "bind_ip": _param("bind_ip", BIND_IP),
        "host_port": int(_param("host_port", LIDAR_HOST_PORT)),
        "destination_port": int(_param("destination_port", LIDAR_PORT)),
        "frame_id": _param("frame_id", "morai_lidar"),
        "topic": _param("topic", "/morai/lidar/live_points"),
        "display_topic": _param("display_topic", "/morai/lidar/display_points"),
        "nearest_distance_topic": _param(
            "nearest_distance_topic",
            "/morai/lidar/nearest_distance_m",
        ),
        "packets_per_cloud": int(_param("packets_per_cloud", 80)),
        "rolling_clouds": int(_param("rolling_clouds", 1)),
        "display_rolling_clouds": int(_param("display_rolling_clouds", 5)),
        "display_history_s": float(_param("display_history_s", 0.35)),
        "max_cloud_age_s": float(_param("max_cloud_age_s", 0.10)),
        "socket_timeout_s": float(_param("socket_timeout_s", 1.0)),
        "lidar_yaw_offset_deg": float(_param("lidar_yaw_offset_deg", 0.0)),
        "fov_left_deg": float(_param("fov_left_deg", 180.0)),
        "fov_right_deg": float(_param("fov_right_deg", 180.0)),
        "rear_blind_deg": float(_param("rear_blind_deg", 60.0)),
        "x_min_m": float(_param("x_min_m", -40.0)),
        "x_max_m": float(_param("x_max_m", 40.0)),
        "y_abs_m": float(_param("y_abs_m", 40.0)),
        "z_min_m": float(_param("z_min_m", -2.5)),
        "z_max_m": float(_param("z_max_m", 3.0)),
        "min_distance_m": float(_param("min_distance_m", 0.2)),
        "nearest_x_min_m": float(_param("nearest_x_min_m", 1.0)),
        "nearest_x_max_m": float(_param("nearest_x_max_m", 40.0)),
        "nearest_y_abs_m": float(_param("nearest_y_abs_m", 5.0)),
        "nearest_z_min_m": float(_param("nearest_z_min_m", -1.2)),
        "nearest_z_max_m": float(_param("nearest_z_max_m", 2.5)),
    }

    if params["packets_per_cloud"] < 1:
        raise ValueError("~packets_per_cloud must be at least 1")
    if params["rolling_clouds"] < 1:
        raise ValueError("~rolling_clouds must be at least 1")
    if params["display_rolling_clouds"] < 1:
        raise ValueError("~display_rolling_clouds must be at least 1")
    if params["display_history_s"] < 0.0:
        raise ValueError("~display_history_s cannot be negative")
    if params["max_cloud_age_s"] < 0.0:
        raise ValueError("~max_cloud_age_s cannot be negative")
    if params["fov_left_deg"] < 0.0 or params["fov_right_deg"] < 0.0:
        raise ValueError("FOV limits cannot be negative")
    if params["fov_left_deg"] > 180.0 or params["fov_right_deg"] > 180.0:
        raise ValueError("FOV limits cannot exceed 180 degrees")
    if params["rear_blind_deg"] < 0.0 or params["rear_blind_deg"] > 180.0:
        raise ValueError("rear blind sector must be between 0 and 180 degrees")
    if params["nearest_x_max_m"] <= params["nearest_x_min_m"]:
        raise ValueError("nearest x range must satisfy min < max")
    if params["nearest_y_abs_m"] < 0.0:
        raise ValueError("nearest_y_abs_m cannot be negative")
    if params["nearest_z_max_m"] <= params["nearest_z_min_m"]:
        raise ValueError("nearest z range must satisfy min < max")

    publisher = rospy.Publisher(params["topic"], PointCloud2, queue_size=1)
    display_publisher = rospy.Publisher(params["display_topic"], PointCloud2, queue_size=1)
    nearest_distance_publisher = rospy.Publisher(
        params["nearest_distance_topic"],
        Float32,
        queue_size=1,
    )

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    udp_socket.bind((params["bind_ip"], params["destination_port"]))
    udp_socket.settimeout(params["socket_timeout_s"])

    rospy.loginfo(
        "MORAI LiDAR PointCloud2 UDP: source *:%d -> %s:%d, topic=%s, "
        "display_topic=%s, nearest_distance_topic=%s, frame=%s, "
        "FOV=-%.1f..+%.1f deg, rear_blind=%.1f deg, "
        "rolling_clouds=%d, display_rolling_clouds=%d, display_history=%.3fs, "
        "max_cloud_age=%.3fs",
        params["host_port"],
        params["bind_ip"],
        params["destination_port"],
        params["topic"],
        params["display_topic"],
        params["nearest_distance_topic"],
        params["frame_id"],
        params["fov_right_deg"],
        params["fov_left_deg"],
        params["rear_blind_deg"],
        params["rolling_clouds"],
        params["display_rolling_clouds"],
        params["display_history_s"],
        params["max_cloud_age_s"],
    )

    rolling_clouds = deque(maxlen=params["rolling_clouds"])
    display_rolling_clouds = deque(maxlen=params["display_rolling_clouds"])

    try:
        while not rospy.is_shutdown():
            cloud_points = []
            packets = 0
            bad_packets = 0
            cloud_started_at = time.monotonic()
            while packets < params["packets_per_cloud"] and not rospy.is_shutdown():
                if (
                    packets > 0
                    and params["max_cloud_age_s"] > 0.0
                    and time.monotonic() - cloud_started_at >= params["max_cloud_age_s"]
                ):
                    break
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

                cloud_points.extend(_sample_points(lidar_packet.points, params))

            if cloud_points:
                rolling_clouds.append(cloud_points)
                display_rolling_clouds.append((time.monotonic(), cloud_points))
            accumulated_points = [
                point
                for cloud in rolling_clouds
                for point in cloud
            ]
            now = time.monotonic()
            while (
                display_rolling_clouds
                and params["display_history_s"] > 0.0
                and now - display_rolling_clouds[0][0] > params["display_history_s"]
            ):
                display_rolling_clouds.popleft()
            display_points = [
                point
                for _, cloud in display_rolling_clouds
                for point in cloud
            ]
            if accumulated_points:
                publisher.publish(_make_cloud(accumulated_points, params["frame_id"]))
                nearest_distance = _nearest_distance(accumulated_points, params)
                if nearest_distance is not None:
                    nearest_distance_publisher.publish(Float32(data=nearest_distance))
            if display_points:
                display_publisher.publish(_make_cloud(display_points, params["frame_id"]))
            nearest_text = _nearest_distance(accumulated_points, params)
            rospy.loginfo_throttle(
                1.0,
                "LiDAR cloud: packets=%d bad=%d live_points=%d display_points=%d "
                "nearest=%s age=%.3fs",
                packets,
                bad_packets,
                len(accumulated_points),
                len(display_points),
                "n/a" if nearest_text is None else "{:.2f}m".format(nearest_text),
                time.monotonic() - cloud_started_at,
            )
    finally:
        udp_socket.close()


if __name__ == "__main__":
    main()
