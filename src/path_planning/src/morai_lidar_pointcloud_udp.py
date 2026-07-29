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
import json
import socket
import time
from collections import deque
from collections import defaultdict

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Header, String
from visualization_msgs.msg import Marker, MarkerArray

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

POINT_X = 0
POINT_Y = 1
POINT_Z = 2
POINT_DISTANCE = 3
POINT_INTENSITY = 4
POINT_RING = 5
POINT_BEARING = 6


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _bool_param(name, default):
    value = _param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


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


def _cluster_candidate_points(points, params):
    candidates = [
        point
        for point in points
        if params["cluster_x_min_m"] <= point[POINT_X] <= params["cluster_x_max_m"]
        and abs(point[POINT_Y]) <= params["cluster_y_abs_m"]
        and params["cluster_z_min_m"] <= point[POINT_Z] <= params["cluster_z_max_m"]
    ]
    max_points = params["cluster_max_input_points"]
    if max_points > 0 and len(candidates) > max_points:
        step = int(math.ceil(float(len(candidates)) / float(max_points)))
        candidates = candidates[::step]
    if len(candidates) < params["cluster_min_points"]:
        return []

    tolerance = params["cluster_tolerance_m"]
    tolerance_sq = tolerance * tolerance
    grid = defaultdict(list)
    for index, point in enumerate(candidates):
        key = (
            int(math.floor(point[POINT_X] / tolerance)),
            int(math.floor(point[POINT_Y] / tolerance)),
            int(math.floor(point[POINT_Z] / tolerance)),
        )
        grid[key].append(index)

    visited = [False] * len(candidates)
    clusters = []
    for seed_index in range(len(candidates)):
        if visited[seed_index]:
            continue
        visited[seed_index] = True
        queue = [seed_index]
        cluster_indices = []
        while queue:
            current_index = queue.pop()
            cluster_indices.append(current_index)
            current = candidates[current_index]
            current_key = (
                int(math.floor(current[POINT_X] / tolerance)),
                int(math.floor(current[POINT_Y] / tolerance)),
                int(math.floor(current[POINT_Z] / tolerance)),
            )
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        neighbor_key = (
                            current_key[0] + dx,
                            current_key[1] + dy,
                            current_key[2] + dz,
                        )
                        for neighbor_index in grid.get(neighbor_key, ()):
                            if visited[neighbor_index]:
                                continue
                            neighbor = candidates[neighbor_index]
                            distance_sq = (
                                (current[POINT_X] - neighbor[POINT_X]) ** 2
                                + (current[POINT_Y] - neighbor[POINT_Y]) ** 2
                                + (current[POINT_Z] - neighbor[POINT_Z]) ** 2
                            )
                            if distance_sq <= tolerance_sq:
                                visited[neighbor_index] = True
                                queue.append(neighbor_index)

        if len(cluster_indices) < params["cluster_min_points"]:
            continue
        cluster_points = [candidates[index] for index in cluster_indices]
        clusters.append(_summarize_cluster(cluster_points))

    clusters.sort(key=lambda cluster: cluster["nearest_distance_m"])
    return clusters[: params["cluster_max_clusters"]]


def _summarize_cluster(points):
    xs = [point[POINT_X] for point in points]
    ys = [point[POINT_Y] for point in points]
    zs = [point[POINT_Z] for point in points]
    centroid_x = sum(xs) / len(xs)
    centroid_y = sum(ys) / len(ys)
    centroid_z = sum(zs) / len(zs)
    centroid_distance = math.sqrt(centroid_x * centroid_x + centroid_y * centroid_y)
    bearing_deg = math.degrees(math.atan2(centroid_y, centroid_x))
    nearest_distance = min(point[POINT_DISTANCE] for point in points)
    return {
        "id": 0,
        "point_count": len(points),
        "centroid_x_m": centroid_x,
        "centroid_y_m": centroid_y,
        "centroid_z_m": centroid_z,
        "centroid_distance_m": centroid_distance,
        "bearing_deg": bearing_deg,
        "nearest_distance_m": nearest_distance,
        "min_x_m": min(xs),
        "max_x_m": max(xs),
        "min_y_m": min(ys),
        "max_y_m": max(ys),
        "min_z_m": min(zs),
        "max_z_m": max(zs),
    }


def _format_clusters(clusters):
    summaries = []
    for index, cluster in enumerate(clusters):
        cluster = dict(cluster)
        cluster["id"] = index
        summaries.append(cluster)
    return summaries


def _clusters_to_json(clusters):
    fields = []
    for cluster in clusters:
        ordered = {
            "id": cluster["id"],
            "point_count": cluster["point_count"],
            "distance_m": round(cluster["centroid_distance_m"], 3),
            "nearest_distance_m": round(cluster["nearest_distance_m"], 3),
            "bearing_deg": round(cluster["bearing_deg"], 2),
            "centroid": [
                round(cluster["centroid_x_m"], 3),
                round(cluster["centroid_y_m"], 3),
                round(cluster["centroid_z_m"], 3),
            ],
            "bbox_min": [
                round(cluster["min_x_m"], 3),
                round(cluster["min_y_m"], 3),
                round(cluster["min_z_m"], 3),
            ],
            "bbox_max": [
                round(cluster["max_x_m"], 3),
                round(cluster["max_y_m"], 3),
                round(cluster["max_z_m"], 3),
            ],
        }
        fields.append(ordered)
    return json.dumps(fields, ensure_ascii=False)


def _make_obstacle_markers(clusters, frame_id):
    marker_array = MarkerArray()

    clear_marker = Marker()
    clear_marker.header.frame_id = frame_id
    clear_marker.header.stamp = rospy.Time.now()
    clear_marker.action = Marker.DELETEALL
    marker_array.markers.append(clear_marker)

    stamp = rospy.Time.now()
    for cluster in clusters:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "morai_lidar_obstacles"
        marker.id = cluster["id"] * 2
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = cluster["centroid_x_m"]
        marker.pose.position.y = cluster["centroid_y_m"]
        marker.pose.position.z = cluster["centroid_z_m"]
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.2, cluster["max_x_m"] - cluster["min_x_m"])
        marker.scale.y = max(0.2, cluster["max_y_m"] - cluster["min_y_m"])
        marker.scale.z = max(0.2, cluster["max_z_m"] - cluster["min_z_m"])
        marker.color.r = 1.0
        marker.color.g = 0.25
        marker.color.b = 0.05
        marker.color.a = 0.45
        marker.lifetime = rospy.Duration(0.4)
        marker_array.markers.append(marker)

        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp = stamp
        text.ns = "morai_lidar_obstacle_labels"
        text.id = cluster["id"] * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = cluster["centroid_x_m"]
        text.pose.position.y = cluster["centroid_y_m"]
        text.pose.position.z = cluster["max_z_m"] + 0.6
        text.pose.orientation.w = 1.0
        text.scale.z = 0.55
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = "#{id} {distance:.1f}m {angle:+.0f}deg n={count}".format(
            id=cluster["id"],
            distance=cluster["centroid_distance_m"],
            angle=cluster["bearing_deg"],
            count=cluster["point_count"],
        )
        text.lifetime = rospy.Duration(0.4)
        marker_array.markers.append(text)

    return marker_array


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
        "obstacle_topic": _param("obstacle_topic", "/morai/lidar/obstacles"),
        "obstacle_marker_topic": _param(
            "obstacle_marker_topic",
            "/morai/lidar/obstacle_markers",
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
        "cluster_enabled": _bool_param("cluster_enabled", True),
        "cluster_tolerance_m": float(_param("cluster_tolerance_m", 0.8)),
        "cluster_min_points": int(_param("cluster_min_points", 5)),
        "cluster_max_clusters": int(_param("cluster_max_clusters", 8)),
        "cluster_max_input_points": int(_param("cluster_max_input_points", 5000)),
        "cluster_x_min_m": float(_param("cluster_x_min_m", 1.0)),
        "cluster_x_max_m": float(_param("cluster_x_max_m", 40.0)),
        "cluster_y_abs_m": float(_param("cluster_y_abs_m", 8.0)),
        "cluster_z_min_m": float(_param("cluster_z_min_m", -1.2)),
        "cluster_z_max_m": float(_param("cluster_z_max_m", 2.5)),
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
    if params["cluster_tolerance_m"] <= 0.0:
        raise ValueError("cluster_tolerance_m must be positive")
    if params["cluster_min_points"] < 1:
        raise ValueError("cluster_min_points must be at least 1")
    if params["cluster_max_clusters"] < 1:
        raise ValueError("cluster_max_clusters must be at least 1")
    if params["cluster_max_input_points"] < 0:
        raise ValueError("cluster_max_input_points cannot be negative")
    if params["cluster_x_max_m"] <= params["cluster_x_min_m"]:
        raise ValueError("cluster x range must satisfy min < max")
    if params["cluster_y_abs_m"] < 0.0:
        raise ValueError("cluster_y_abs_m cannot be negative")
    if params["cluster_z_max_m"] <= params["cluster_z_min_m"]:
        raise ValueError("cluster z range must satisfy min < max")

    publisher = rospy.Publisher(params["topic"], PointCloud2, queue_size=1)
    display_publisher = rospy.Publisher(params["display_topic"], PointCloud2, queue_size=1)
    nearest_distance_publisher = rospy.Publisher(
        params["nearest_distance_topic"],
        Float32,
        queue_size=1,
    )
    obstacle_publisher = rospy.Publisher(params["obstacle_topic"], String, queue_size=1)
    obstacle_marker_publisher = rospy.Publisher(
        params["obstacle_marker_topic"],
        MarkerArray,
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
            clusters = []
            if accumulated_points:
                publisher.publish(_make_cloud(accumulated_points, params["frame_id"]))
                nearest_distance = _nearest_distance(accumulated_points, params)
                if nearest_distance is not None:
                    nearest_distance_publisher.publish(Float32(data=nearest_distance))
                if params["cluster_enabled"]:
                    clusters = _format_clusters(
                        _cluster_candidate_points(accumulated_points, params)
                    )
                    obstacle_publisher.publish(String(data=_clusters_to_json(clusters)))
                    obstacle_marker_publisher.publish(
                        _make_obstacle_markers(clusters, params["frame_id"])
                    )
            if display_points:
                display_publisher.publish(_make_cloud(display_points, params["frame_id"]))
            nearest_text = _nearest_distance(accumulated_points, params)
            rospy.loginfo_throttle(
                1.0,
                "LiDAR cloud: packets=%d bad=%d live_points=%d display_points=%d "
                "nearest=%s clusters=%d age=%.3fs",
                packets,
                bad_packets,
                len(accumulated_points),
                len(display_points),
                "n/a" if nearest_text is None else "{:.2f}m".format(nearest_text),
                len(clusters),
                time.monotonic() - cloud_started_at,
            )
    finally:
        udp_socket.close()


if __name__ == "__main__":
    main()
