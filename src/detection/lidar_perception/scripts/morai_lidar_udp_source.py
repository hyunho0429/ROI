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
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Header, String
from visualization_msgs.msg import Marker, MarkerArray

from lidar_perception.lidar_direct_localization import DirectGpsImuPoseEstimator
from lidar_perception.lidar_deskew import deskew_scan, pose_at
from lidar_perception.lidar_merge_gap import (
    MergeGapTracker,
    assess_merge_gaps,
    format_merge_gap_status,
)
from lidar_perception.lidar_obstacle_filter import filter_vertical_support

try:
    from lidar_perception.morai_competition_config import BIND_IP
except ImportError:
    BIND_IP = "0.0.0.0"

try:
    from lidar_perception.morai_competition_config import (
        GPS_PORT,
        IMU_PORT,
        LIDAR_HOST_PORT,
        LIDAR_PORT,
    )
except ImportError:
    GPS_PORT = 3001
    IMU_PORT = 4001
    LIDAR_HOST_PORT = 2000
    LIDAR_PORT = 2001
try:
    from lidar_perception.morai_competition_config import LIDAR_POSE_IP, LIDAR_POSE_PORT
except ImportError:
    LIDAR_POSE_IP = "127.0.0.1"
    LIDAR_POSE_PORT = 4012

from lidar_perception.morai_udp_lidar import (
    LidarPacketError,
    parse_lidar_intensity_packet,
    should_publish_cloud,
)
from lidar_perception.morai_udp_gps import GpsPacketError
from lidar_perception.morai_udp_imu import ImuPacketError
from lidar_perception.morai_udp_localization_pose import (
    LocalizationPose,
    LocalizationPosePacketError,
    decode_localization_pose,
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


def _drain_pose_socket(udp_socket, pose_samples, now, buffer_s):
    received_count = 0
    invalid_count = 0
    while True:
        try:
            packet, _sender = udp_socket.recvfrom(256)
        except BlockingIOError:
            break
        try:
            pose = decode_localization_pose(packet)
        except LocalizationPosePacketError:
            invalid_count += 1
            continue
        if pose.timestamp_monotonic_s > now + 0.5:
            invalid_count += 1
            continue
        if (
            pose_samples
            and pose.timestamp_monotonic_s
            <= pose_samples[-1].timestamp_monotonic_s
        ):
            continue
        pose_samples.append(pose)
        received_count += 1

    cutoff = now - buffer_s
    while len(pose_samples) > 1 and pose_samples[1].timestamp_monotonic_s < cutoff:
        pose_samples.popleft()
    return received_count, invalid_count


def _append_odometry_pose(message, pose_samples, buffer_s, pose_stats):
    """Convert local ROS odometry to the monotonic pose timeline used by deskew.

    MORAI sensor transport remains UDP.  This callback only reuses the fused
    odometry already produced locally by the UDP EKF, avoiding a second bind on
    the GPS and IMU ports.
    """
    received_at = time.monotonic()
    orientation = message.pose.pose.orientation
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    twist = message.twist.twist
    position = message.pose.pose.position
    pose_samples.append(
        LocalizationPose(
            timestamp_monotonic_s=received_at,
            x_m=float(position.x),
            y_m=float(position.y),
            z_m=float(position.z),
            yaw_rad=math.atan2(sin_yaw, cos_yaw),
            speed_mps=math.hypot(twist.linear.x, twist.linear.y),
            yaw_rate_radps=float(twist.angular.z),
        )
    )
    cutoff = received_at - float(buffer_s)
    while pose_samples and pose_samples[0].timestamp_monotonic_s < cutoff:
        pose_samples.popleft()
    pose_stats["received"] += 1


def _append_pose_sample(pose_samples, pose):
    if pose is None:
        return False
    if (
        pose_samples
        and pose.timestamp_monotonic_s
        <= pose_samples[-1].timestamp_monotonic_s
    ):
        return False
    pose_samples.append(pose)
    return True


def _prune_pose_samples(pose_samples, now, buffer_s):
    cutoff = now - buffer_s
    while len(pose_samples) > 1 and pose_samples[1].timestamp_monotonic_s < cutoff:
        pose_samples.popleft()


def _drain_direct_sensor_sockets(
    sensor_sockets,
    estimator,
    pose_samples,
    now,
    buffer_s,
):
    pose_count = 0
    invalid_count = 0
    for sensor_name, udp_socket in sensor_sockets:
        while True:
            try:
                packet, _sender = udp_socket.recvfrom(65535)
            except BlockingIOError:
                break
            received_at = time.monotonic()
            try:
                if sensor_name == "gps":
                    pose = estimator.add_gps_packet(packet, received_at)
                else:
                    pose = estimator.add_imu_packet(packet, received_at)
            except (GpsPacketError, ImuPacketError, ValueError):
                invalid_count += 1
                continue
            if _append_pose_sample(pose_samples, pose):
                pose_count += 1
    _prune_pose_samples(pose_samples, now, buffer_s)
    return pose_count, invalid_count


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

        # ============================================================
        # Ground Removal
        # threshold 이하의 점은 sampled에 넣지 않음
        # → 이후 PointCloud / clustering 연산량 감소
        # ============================================================
        if (
            params["ground_remove_enabled"]
            and z_up <= params["ground_z_threshold_m"]
        ):
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


def _nearest_candidate_distances(points, params):
    candidates = [
        point
        for point in points
        if params["nearest_x_min_m"] <= point[POINT_X] <= params["nearest_x_max_m"]
        and abs(point[POINT_Y]) <= params["nearest_y_abs_m"]
        and params["nearest_z_min_m"]
        <= point[POINT_Z]
        <= params["nearest_z_max_m"]
    ]
    candidates = _vertical_support_candidates(candidates, params)
    return [point[POINT_DISTANCE] for point in candidates]


def _vertical_support_candidates(points, params):
    if not params["vertical_support_enabled"]:
        return points
    return filter_vertical_support(
        points,
        params["vertical_support_radius_m"],
        params["vertical_support_min_height_m"],
    )


def _nearest_distance(points, params):
    distances = _nearest_candidate_distances(points, params)
    if len(distances) < params["nearest_min_points"]:
        return None
    return min(distances)


def _cluster_candidate_points(points, params):
    candidates = [
        point
        for point in points
        if params["cluster_x_min_m"] <= point[POINT_X] <= params["cluster_x_max_m"]
        and abs(point[POINT_Y]) <= params["cluster_y_abs_m"]
        and params["cluster_z_min_m"] <= point[POINT_Z] <= params["cluster_z_max_m"]
    ]
    candidates = _vertical_support_candidates(candidates, params)
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
        cluster = _summarize_cluster(cluster_points)
        cluster_height = cluster["max_z_m"] - cluster["min_z_m"]
        if cluster_height < params["cluster_min_height_m"]:
            continue
        clusters.append(cluster)

    clusters.sort(key=lambda cluster: cluster["nearest_distance_m"])
    return clusters[: params["cluster_max_clusters"]]


def _merge_gap_candidate_clusters(points, params):
    lane_width = params["adjacent_lane_width_m"]
    lane_tolerance = (
        0.5
        * max(0.0, lane_width - params["ego_size_y_m"])
        + params["gap_lane_lateral_allowance_m"]
    )
    adjacent_lane_points = [
        point
        for point in points
        if min(
            abs(point[POINT_Y] - lane_width),
            abs(point[POINT_Y] + lane_width),
        )
        <= lane_tolerance
    ]
    gap_params = dict(params)
    gap_params["cluster_x_min_m"] = -params["gap_detection_range_m"]
    gap_params["cluster_x_max_m"] = params["gap_detection_range_m"]
    gap_params["cluster_y_abs_m"] = lane_width + lane_tolerance
    gap_params["cluster_max_clusters"] = params["gap_max_clusters"]
    return _cluster_candidate_points(adjacent_lane_points, gap_params)


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


def _make_obstacle_markers(clusters, frame_id, max_clusters, lifetime_s):
    marker_array = MarkerArray()

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
        marker.lifetime = rospy.Duration(lifetime_s)
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
        text.lifetime = rospy.Duration(lifetime_s)
        marker_array.markers.append(text)

    for stale_id in range(len(clusters), max_clusters):
        for marker_offset in (0, 1):
            delete_marker = Marker()
            delete_marker.header.frame_id = frame_id
            delete_marker.header.stamp = stamp
            delete_marker.ns = (
                "morai_lidar_obstacles"
                if marker_offset == 0
                else "morai_lidar_obstacle_labels"
            )
            delete_marker.id = stale_id * 2 + marker_offset
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)

    return marker_array


def main():
    rospy.init_node("morai_lidar_pointcloud_udp_ground")

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
        "packets_per_cloud": int(_param("packets_per_cloud", 15)),
        "rolling_clouds": int(_param("rolling_clouds", 1)),
        "display_rolling_clouds": int(_param("display_rolling_clouds", 1)),
        "display_history_s": float(_param("display_history_s", 0.0)),
        "max_cloud_age_s": float(_param("max_cloud_age_s", 0.05)),
        "socket_timeout_s": float(_param("socket_timeout_s", 1.0)),
        "deskew_enabled": _bool_param("deskew_enabled", True),
        "deskew_pose_source": str(
            _param("deskew_pose_source", "sensor_udp")
        ).strip().lower(),
        "gps_port": int(_param("gps_port", GPS_PORT)),
        "imu_port": int(_param("imu_port", IMU_PORT)),
        "pose_bind_ip": _param("pose_bind_ip", LIDAR_POSE_IP),
        "pose_port": int(_param("pose_port", LIDAR_POSE_PORT)),
        "odometry_topic": _param(
            "odometry_topic", "/localization/odometry"
        ),
        "pose_buffer_s": float(_param("pose_buffer_s", 1.0)),
        "pose_extrapolation_limit_s": float(
            _param("pose_extrapolation_limit_s", 0.12)
        ),
        "lidar_yaw_offset_deg": float(_param("lidar_yaw_offset_deg", 0.0)),
        "fov_left_deg": float(_param("fov_left_deg", 180.0)),
        "fov_right_deg": float(_param("fov_right_deg", 180.0)),
        "rear_blind_deg": float(_param("rear_blind_deg", 0.0)),
        "x_min_m": float(_param("x_min_m", -40.0)),
        "x_max_m": float(_param("x_max_m", 40.0)),
        "y_abs_m": float(_param("y_abs_m", 40.0)),
        "z_min_m": float(_param("z_min_m", -2.5)),
        "z_max_m": float(_param("z_max_m", 3.0)),

        # Ground Removal
        # 이 높이 이하의 점을 바닥으로 판단하여 제거
        "ground_remove_enabled": _bool_param("ground_remove_enabled", True),
        "ground_z_threshold_m": float(
            _param("ground_z_threshold_m", -1.5)
        ),
        "vertical_support_enabled": _bool_param(
            "vertical_support_enabled", True
        ),
        "vertical_support_filter_cloud": _bool_param(
            "vertical_support_filter_cloud", True
        ),
        "vertical_support_radius_m": float(
            _param("vertical_support_radius_m", 0.65)
        ),
        "vertical_support_min_height_m": float(
            _param("vertical_support_min_height_m", 0.05)
        ),

        "min_distance_m": float(_param("min_distance_m", 0.2)),
        "fast_nearest_enabled": _bool_param("fast_nearest_enabled", True),
        "nearest_publish_interval_s": float(
            _param("nearest_publish_interval_s", 0.05)
        ),
        "nearest_hold_s": float(_param("nearest_hold_s", 0.20)),
        "nearest_min_points": int(_param("nearest_min_points", 2)),
        "nearest_x_min_m": float(_param("nearest_x_min_m", 1.0)),
        "nearest_x_max_m": float(_param("nearest_x_max_m", 40.0)),
        "nearest_y_abs_m": float(_param("nearest_y_abs_m", 3.0)),
        "nearest_z_min_m": float(_param("nearest_z_min_m", -1.4)),
        "nearest_z_max_m": float(_param("nearest_z_max_m", 2.5)),
        "cluster_enabled": _bool_param("cluster_enabled", True),
        "cluster_tolerance_m": float(_param("cluster_tolerance_m", 0.8)),
        "cluster_min_points": int(_param("cluster_min_points", 3)),
        "cluster_min_height_m": float(_param("cluster_min_height_m", 0.15)),
        "cluster_hold_s": float(_param("cluster_hold_s", 0.15)),
        "marker_lifetime_s": float(_param("marker_lifetime_s", 0.20)),
        "cluster_max_clusters": int(_param("cluster_max_clusters", 8)),
        "cluster_max_input_points": int(_param("cluster_max_input_points", 5000)),
        "cluster_x_min_m": float(_param("cluster_x_min_m", 1.0)),
        "cluster_x_max_m": float(_param("cluster_x_max_m", 40.0)),
        "cluster_y_abs_m": float(_param("cluster_y_abs_m", 5.0)),
        "cluster_z_min_m": float(_param("cluster_z_min_m", -1.4)),
        "cluster_z_max_m": float(_param("cluster_z_max_m", 2.5)),
        "merge_gap_enabled": _bool_param("merge_gap_enabled", True),
        "ego_size_x_m": float(_param("ego_size_x_m", 4.635)),
        "ego_size_y_m": float(_param("ego_size_y_m", 1.892)),
        "ego_size_z_m": float(_param("ego_size_z_m", 2.434)),
        "adjacent_lane_width_m": float(
            _param("adjacent_lane_width_m", 3.5)
        ),
        "gap_lane_lateral_allowance_m": float(
            _param("gap_lane_lateral_allowance_m", 0.4)
        ),
        "gap_longitudinal_margin_m": float(
            _param("gap_longitudinal_margin_m", 1.0)
        ),
        "gap_lateral_margin_m": float(
            _param("gap_lateral_margin_m", 0.2)
        ),
        "gap_detection_range_m": float(
            _param("gap_detection_range_m", 40.0)
        ),
        "gap_confirmation_scans": int(
            _param("gap_confirmation_scans", 3)
        ),
        "gap_log_interval_s": float(_param("gap_log_interval_s", 1.0)),
        "gap_max_clusters": int(_param("gap_max_clusters", 24)),
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
    if params["deskew_pose_source"] not in (
        "sensor_udp",
        "fused_pose_udp",
        "ros_odometry",
    ):
        raise ValueError(
            "deskew_pose_source must be 'sensor_udp', 'fused_pose_udp', "
            "or 'ros_odometry'"
        )
    for sensor_port in (params["gps_port"], params["imu_port"]):
        if not 1 <= sensor_port <= 65535:
            raise ValueError("GPS and IMU ports must be between 1 and 65535")
    if (
        params["deskew_enabled"]
        and params["deskew_pose_source"] == "sensor_udp"
        and len(
            {
                params["destination_port"],
                params["gps_port"],
                params["imu_port"],
            }
        )
        != 3
    ):
        raise ValueError("LiDAR, GPS and IMU receive ports must be distinct")
    if not 1 <= params["pose_port"] <= 65535:
        raise ValueError("pose_port must be between 1 and 65535")
    if params["pose_buffer_s"] <= 0.0:
        raise ValueError("pose_buffer_s must be positive")
    if params["pose_extrapolation_limit_s"] < 0.0:
        raise ValueError("pose_extrapolation_limit_s cannot be negative")
    if params["fov_left_deg"] < 0.0 or params["fov_right_deg"] < 0.0:
        raise ValueError("FOV limits cannot be negative")
    if params["fov_left_deg"] > 180.0 or params["fov_right_deg"] > 180.0:
        raise ValueError("FOV limits cannot exceed 180 degrees")
    if params["rear_blind_deg"] < 0.0 or params["rear_blind_deg"] > 180.0:
        raise ValueError("rear blind sector must be between 0 and 180 degrees")
    if params["vertical_support_radius_m"] <= 0.0:
        raise ValueError("vertical_support_radius_m must be positive")
    if params["vertical_support_min_height_m"] < 0.0:
        raise ValueError("vertical_support_min_height_m cannot be negative")
    if params["nearest_publish_interval_s"] <= 0.0:
        raise ValueError("nearest_publish_interval_s must be positive")
    if params["nearest_hold_s"] < 0.0:
        raise ValueError("nearest_hold_s cannot be negative")
    if params["nearest_min_points"] < 1:
        raise ValueError("nearest_min_points must be at least 1")
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
    if params["cluster_min_height_m"] < 0.0:
        raise ValueError("cluster_min_height_m cannot be negative")
    if params["cluster_hold_s"] < 0.0:
        raise ValueError("cluster_hold_s cannot be negative")
    if params["marker_lifetime_s"] <= 0.0:
        raise ValueError("marker_lifetime_s must be positive")
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
    if min(
        params["ego_size_x_m"],
        params["ego_size_y_m"],
        params["ego_size_z_m"],
    ) <= 0.0:
        raise ValueError("ego xyz dimensions must be positive")
    if params["adjacent_lane_width_m"] <= 0.0:
        raise ValueError("adjacent_lane_width_m must be positive")
    if params["gap_lane_lateral_allowance_m"] < 0.0:
        raise ValueError("gap_lane_lateral_allowance_m cannot be negative")
    if params["gap_longitudinal_margin_m"] < 0.0:
        raise ValueError("gap_longitudinal_margin_m cannot be negative")
    if params["gap_lateral_margin_m"] < 0.0:
        raise ValueError("gap_lateral_margin_m cannot be negative")
    if params["gap_detection_range_m"] <= 0.5 * params["ego_size_x_m"]:
        raise ValueError("gap_detection_range_m is too short for ego length")
    if params["gap_confirmation_scans"] < 1:
        raise ValueError("gap_confirmation_scans must be at least 1")
    if params["gap_log_interval_s"] <= 0.0:
        raise ValueError("gap_log_interval_s must be positive")
    if params["gap_max_clusters"] < 1:
        raise ValueError("gap_max_clusters must be at least 1")
    if params["merge_gap_enabled"]:
        gap_range = params["gap_detection_range_m"]
        lane_tolerance = (
            0.5
            * max(
                0.0,
                params["adjacent_lane_width_m"] - params["ego_size_y_m"],
            )
            + params["gap_lane_lateral_allowance_m"]
        )
        if params["x_min_m"] > -gap_range or params["x_max_m"] < gap_range:
            raise ValueError(
                "merge gap check requires x_min_m <= -gap_detection_range_m "
                "and x_max_m >= gap_detection_range_m"
            )
        if params["y_abs_m"] < params["adjacent_lane_width_m"] + lane_tolerance:
            raise ValueError("sample y_abs_m does not cover adjacent lane gap ROI")
        if (
            params["fov_left_deg"] < 180.0
            or params["fov_right_deg"] < 180.0
            or params["rear_blind_deg"] > 0.0
        ):
            raise ValueError(
                "merge gap check requires full 360-degree sampling: "
                "fov_left_deg=180, fov_right_deg=180, rear_blind_deg=0"
            )

    if params["rolling_clouds"] > 1:
        rospy.logwarn(
            "rolling_clouds=%d combines scans expressed in different ego frames; "
            "use 1 while driving because deskew compensates motion only within "
            "each rotation",
            params["rolling_clouds"],
        )

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

    pose_samples = deque()
    pose_stats = {"received": 0}
    pose_subscriber = None
    pose_socket = None
    direct_sensor_sockets = []
    direct_pose_estimator = None
    if params["deskew_enabled"]:
        try:
            if params["deskew_pose_source"] == "ros_odometry":
                pose_subscriber = rospy.Subscriber(
                    params["odometry_topic"],
                    Odometry,
                    lambda message: _append_odometry_pose(
                        message,
                        pose_samples,
                        params["pose_buffer_s"],
                        pose_stats,
                    ),
                    queue_size=20,
                )
            elif params["deskew_pose_source"] == "fused_pose_udp":
                pose_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                pose_socket.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_RCVBUF,
                    1024 * 1024,
                )
                pose_socket.bind((params["pose_bind_ip"], params["pose_port"]))
                pose_socket.setblocking(False)
            else:
                direct_pose_estimator = DirectGpsImuPoseEstimator()
                for sensor_name, sensor_port in (
                    ("gps", params["gps_port"]),
                    ("imu", params["imu_port"]),
                ):
                    sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sensor_socket.setsockopt(
                        socket.SOL_SOCKET,
                        socket.SO_RCVBUF,
                        1024 * 1024,
                    )
                    try:
                        sensor_socket.bind((params["bind_ip"], sensor_port))
                    except OSError:
                        sensor_socket.close()
                        raise
                    sensor_socket.setblocking(False)
                    direct_sensor_sockets.append((sensor_name, sensor_socket))
        except OSError:
            if pose_socket is not None:
                pose_socket.close()
            for _sensor_name, sensor_socket in direct_sensor_sockets:
                sensor_socket.close()
            udp_socket.close()
            raise

    if not params["deskew_enabled"]:
        pose_input_description = "disabled"
    else:
        if params["deskew_pose_source"] == "ros_odometry":
            pose_input_description = "ros_odometry={}".format(
                params["odometry_topic"]
            )
        elif params["deskew_pose_source"] == "fused_pose_udp":
            pose_input_description = "fused_pose_udp=%s:%d" % (
                params["pose_bind_ip"],
                params["pose_port"],
            )
        else:
            pose_input_description = "sensor_udp gps=%s:%d imu=%s:%d" % (
                params["bind_ip"],
                params["gps_port"],
                params["bind_ip"],
                params["imu_port"],
            )
    rospy.loginfo(
        "MORAI LiDAR PointCloud2 UDP: source *:%d -> %s:%d, topic=%s, "
        "display_topic=%s, nearest_distance_topic=%s, frame=%s, "
        "FOV=-%.1f..+%.1f deg, rear_blind=%.1f deg, "
        "rolling_clouds=%d, display_rolling_clouds=%d, display_history=%.3fs, "
        "max_cloud_age=%.3fs, deskew=%s pose_source=%s",
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
        "on" if params["deskew_enabled"] else "off",
        pose_input_description,
    )
    if params["merge_gap_enabled"]:
        rospy.loginfo(
            "Merge gap geometry check enabled: ego xyz=(%.3f, %.3f, %.3f)m, "
            "lane_width=%.2fm, required_longitudinal_gap=%.3fm, "
            "lateral_clearance=%.3fm, confirmation=%d scans",
            params["ego_size_x_m"],
            params["ego_size_y_m"],
            params["ego_size_z_m"],
            params["adjacent_lane_width_m"],
            params["ego_size_x_m"]
            + 2.0 * params["gap_longitudinal_margin_m"],
            0.5
            * (params["adjacent_lane_width_m"] - params["ego_size_y_m"]),
            params["gap_confirmation_scans"],
        )

    rolling_clouds = deque(maxlen=params["rolling_clouds"])
    display_rolling_clouds = deque(maxlen=params["display_rolling_clouds"])
    merge_gap_tracker = MergeGapTracker(params["gap_confirmation_scans"])

    try:
        packet_batches = []
        bad_packets = 0
        bad_pose_packets = 0
        pose_packets = 0
        packets = 0
        prev_azimuth = None
        cloud_started_at = time.monotonic()
        nearest_observations = deque()
        last_nearest_publish_at = 0.0
        last_detected_clusters = []
        last_cluster_seen_at = None

        while not rospy.is_shutdown():

            try:
                packet, sender = udp_socket.recvfrom(65535)
            except socket.timeout:
                rospy.logwarn_throttle(
                    2.0,
                    "No LiDAR UDP packet received within %.1fs",
                    params["socket_timeout_s"],
                )
                continue

            packet_received_at = time.monotonic()
            if pose_socket is not None:
                received_poses, invalid_poses = _drain_pose_socket(
                    pose_socket,
                    pose_samples,
                    packet_received_at,
                    params["pose_buffer_s"],
                )
                pose_packets += received_poses
                bad_pose_packets += invalid_poses
            elif direct_sensor_sockets:
                received_poses, invalid_poses = _drain_direct_sensor_sockets(
                    direct_sensor_sockets,
                    direct_pose_estimator,
                    pose_samples,
                    packet_received_at,
                    params["pose_buffer_s"],
                )
                pose_packets += received_poses
                bad_pose_packets += invalid_poses
            elif pose_subscriber is not None:
                pose_packets = pose_stats["received"]

            if sender[1] != params["host_port"]:
                rospy.logwarn_throttle(
                    5.0,
                    "LiDAR UDP sender source port %d, expected %d",
                    sender[1],
                    params["host_port"],
                )

            try:
                lidar_packet = parse_lidar_intensity_packet(packet)
            except (LidarPacketError, ValueError) as error:
                bad_packets += 1
                rospy.logwarn_throttle(2.0, "Bad LiDAR packet: %s", error)
                continue

            packets += 1

            now = time.monotonic()
            packet_points = _sample_points(
                lidar_packet.points,
                params,
            )

            if params["fast_nearest_enabled"]:
                for distance in _nearest_candidate_distances(packet_points, params):
                    nearest_observations.append((now, distance))

                nearest_cutoff = now - params["nearest_hold_s"]
                while (
                    nearest_observations
                    and nearest_observations[0][0] < nearest_cutoff
                ):
                    nearest_observations.popleft()

                if (
                    now - last_nearest_publish_at
                    >= params["nearest_publish_interval_s"]
                ):
                    if len(nearest_observations) >= params["nearest_min_points"]:
                        fast_nearest_distance = min(
                            distance for _, distance in nearest_observations
                        )
                    else:
                        fast_nearest_distance = float("nan")
                    nearest_distance_publisher.publish(
                        Float32(data=fast_nearest_distance)
                    )
                    last_nearest_publish_at = now

            current_azimuth = lidar_packet.points[0].azimuth_deg

            publish_cloud = should_publish_cloud(
                prev_azimuth,
                current_azimuth,
                len(packet_batches),
                now - cloud_started_at,
                params["packets_per_cloud"],
                params["max_cloud_age_s"],
            )

            if publish_cloud:
                if params["deskew_enabled"]:
                    synchronized_batches = [
                        (
                            points,
                            pose_at(
                                pose_samples,
                                acquisition_time,
                                params["pose_extrapolation_limit_s"],
                            ),
                        )
                        for points, acquisition_time in packet_batches
                    ]
                    reference_pose = pose_at(
                        pose_samples,
                        packet_batches[-1][1],
                        params["pose_extrapolation_limit_s"],
                    )
                else:
                    synchronized_batches = [
                        (points, None) for points, _acquisition_time in packet_batches
                    ]
                    reference_pose = None
                scan_points, scan_deskewed = deskew_scan(
                    synchronized_batches,
                    reference_pose,
                )
                if params["deskew_enabled"] and not scan_deskewed:
                    rospy.logwarn_throttle(
                        2.0,
                        "LiDAR deskew fallback: no synchronized pose from %s",
                        pose_input_description,
                    )

                unfiltered_scan_point_count = len(scan_points)
                if (
                    params["vertical_support_enabled"]
                    and params["vertical_support_filter_cloud"]
                ):
                    scan_points = _vertical_support_candidates(
                        scan_points,
                        params,
                    )
                rejected_arc_point_count = (
                    unfiltered_scan_point_count - len(scan_points)
                )

                rolling_clouds.append(scan_points)
                display_rolling_clouds.append(
                    (time.monotonic(), scan_points)
                )

                accumulated_points = [
                    point
                    for cloud in rolling_clouds
                    for point in cloud
                ]

                while (
                    display_rolling_clouds
                    and params["display_history_s"] > 0.0
                    and now - display_rolling_clouds[0][0]
                    > params["display_history_s"]
                ):
                    display_rolling_clouds.popleft()

                display_points = [
                    point
                    for _, cloud in display_rolling_clouds
                    for point in cloud
                ]

                clusters = []
                publisher.publish(
                    _make_cloud(
                        accumulated_points,
                        params["frame_id"],
                    )
                )
                display_publisher.publish(
                    _make_cloud(
                        display_points,
                        params["frame_id"],
                    )
                )

                if accumulated_points:
                    nearest_distance = _nearest_distance(
                        accumulated_points,
                        params,
                    )

                    if (
                        not params["fast_nearest_enabled"]
                        and nearest_distance is not None
                    ):
                        nearest_distance_publisher.publish(
                            Float32(data=nearest_distance)
                        )

                if params["cluster_enabled"]:
                    detected_clusters = _format_clusters(
                        _cluster_candidate_points(
                            accumulated_points,
                            params,
                        )
                    )

                    if detected_clusters:
                        clusters = detected_clusters
                        last_detected_clusters = detected_clusters
                        last_cluster_seen_at = now
                    elif (
                        last_detected_clusters
                        and last_cluster_seen_at is not None
                        and now - last_cluster_seen_at
                        <= params["cluster_hold_s"]
                    ):
                        clusters = last_detected_clusters
                    else:
                        clusters = []
                        last_detected_clusters = []
                        last_cluster_seen_at = None

                    obstacle_publisher.publish(
                        String(
                            data=_clusters_to_json(clusters)
                        )
                    )

                    obstacle_marker_publisher.publish(
                        _make_obstacle_markers(
                            clusters,
                            params["frame_id"],
                            params["cluster_max_clusters"],
                            params["marker_lifetime_s"],
                        )
                    )

                if params["merge_gap_enabled"]:
                    merge_gap_clusters = _merge_gap_candidate_clusters(
                        accumulated_points,
                        params,
                    )
                    merge_gap_assessments = assess_merge_gaps(
                        merge_gap_clusters,
                        params["ego_size_x_m"],
                        params["ego_size_y_m"],
                        params["ego_size_z_m"],
                        params["adjacent_lane_width_m"],
                        params["gap_lane_lateral_allowance_m"],
                        params["gap_longitudinal_margin_m"],
                        params["gap_lateral_margin_m"],
                        params["gap_detection_range_m"],
                    )
                    (
                        merge_gap_assessments,
                        available_sides,
                        unavailable_sides,
                    ) = merge_gap_tracker.update(merge_gap_assessments)
                    left_gap_text = format_merge_gap_status(
                        merge_gap_assessments["left"]
                    )
                    right_gap_text = format_merge_gap_status(
                        merge_gap_assessments["right"]
                    )
                    rospy.loginfo_throttle(
                        params["gap_log_interval_s"],
                        "MERGE_GAP GEOMETRY_ONLY | %s | %s",
                        left_gap_text,
                        right_gap_text,
                    )
                    for side in available_sides:
                        rospy.loginfo(
                            "MERGE_GAP AVAILABLE (GEOMETRY_ONLY): %s",
                            format_merge_gap_status(
                                merge_gap_assessments[side]
                            ),
                        )
                    for side in unavailable_sides:
                        rospy.logwarn(
                            "MERGE_GAP LOST (GEOMETRY_ONLY): %s",
                            format_merge_gap_status(
                                merge_gap_assessments[side]
                            ),
                        )

                nearest_text = _nearest_distance(
                    accumulated_points,
                    params,
                )

                rospy.loginfo_throttle(
                    1.0,
                    "LiDAR cloud: packets=%d bad=%d pose=%d pose_bad=%d deskew=%s "
                    "arc_removed=%d live_points=%d display_points=%d "
                    "nearest=%s clusters=%d",
                    packets,
                    bad_packets,
                    pose_packets,
                    bad_pose_packets,
                    (
                        "on"
                        if scan_deskewed
                        else "fallback"
                        if params["deskew_enabled"]
                        else "off"
                    ),
                    rejected_arc_point_count,
                    len(accumulated_points),
                    len(display_points),
                    "n/a"
                    if nearest_text is None
                    else "{:.2f}m".format(nearest_text),
                    len(clusters),
                )

                packet_batches = []
                packets = 0
                pose_packets = 0
                bad_pose_packets = 0
                cloud_started_at = time.monotonic()

            packet_batches.append((packet_points, packet_received_at))

            prev_azimuth = current_azimuth

    finally:
        if pose_socket is not None:
            pose_socket.close()
        for _sensor_name, sensor_socket in direct_sensor_sockets:
            sensor_socket.close()
        udp_socket.close()

if __name__ == "__main__":
    main()
