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
import threading
import time
from collections import deque
from collections import defaultdict

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Header, String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from path_planning.morai_competition_config import (
        BIND_IP,
        COMPETITION_STATUS_HOST_PORT,
        COMPETITION_STATUS_PORT,
    )
except ImportError:
    BIND_IP = "0.0.0.0"
    COMPETITION_STATUS_HOST_PORT = 9080
    COMPETITION_STATUS_PORT = 9081

try:
    from path_planning.morai_competition_config import LIDAR_HOST_PORT, LIDAR_PORT
except ImportError:
    LIDAR_HOST_PORT = 2000
    LIDAR_PORT = 2001

from path_planning.morai_udp_lidar import (
    LidarPacketError,
    parse_lidar_intensity_packet,
)
from path_planning.morai_udp_competition_status import (
    CompetitionStatusPacketError,
    parse_competition_vehicle_status,
)


POINT_FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("distance_m", 12, PointField.FLOAT32, 1),
    PointField("intensity", 16, PointField.FLOAT32, 1),
    PointField("ring", 20, PointField.FLOAT32, 1),
    PointField("bearing_deg", 24, PointField.FLOAT32, 1),
    PointField("age_s", 28, PointField.FLOAT32, 1),
]

POINT_X = 0
POINT_Y = 1
POINT_Z = 2
POINT_DISTANCE = 3
POINT_INTENSITY = 4
POINT_RING = 5
POINT_BEARING = 6
POINT_AGE = 7


class MotionState:
    """Latest ego motion used to deskew one LiDAR cloud.

    MORAI's VLP-16 UDP packets arrive over multiple datagrams per cloud.  When
    the car moves quickly, old packets are already measured in a slightly older
    ego frame.  This state stores the latest Competition Vehicle Status speed
    and yaw rate so every packet can be transformed to the cloud publish time.
    """

    def __init__(self, manual_speed_mps=0.0, manual_yaw_rate_radps=0.0):
        self._lock = threading.Lock()
        self._speed_mps = float(manual_speed_mps)
        self._yaw_rate_radps = float(manual_yaw_rate_radps)
        self._updated_at = None
        self._ctrl_mode = None
        self._gear = None
        self._source = "manual"

    def update_from_status(self, status, updated_at):
        with self._lock:
            self._speed_mps = float(status.signed_velocity_kmh) / 3.6
            self._yaw_rate_radps = math.radians(float(status.angular_velocity_degps[2]))
            self._updated_at = float(updated_at)
            self._ctrl_mode = int(status.ctrl_mode)
            self._gear = int(status.gear)
            self._source = "competition"

    def snapshot(self, now, timeout_s, manual_speed_mps, manual_yaw_rate_radps):
        with self._lock:
            if self._updated_at is None or now - self._updated_at > timeout_s:
                return {
                    "speed_mps": float(manual_speed_mps),
                    "yaw_rate_radps": float(manual_yaw_rate_radps),
                    "status_age_s": None
                    if self._updated_at is None
                    else max(0.0, now - self._updated_at),
                    "fresh": False,
                    "ctrl_mode": self._ctrl_mode,
                    "gear": self._gear,
                    "source": "manual",
                }
            return {
                "speed_mps": self._speed_mps,
                "yaw_rate_radps": self._yaw_rate_radps,
                "status_age_s": max(0.0, now - self._updated_at),
                "fresh": True,
                "ctrl_mode": self._ctrl_mode,
                "gear": self._gear,
                "source": self._source,
            }


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


def _motion_compensate_xy(x_forward, y_left, point_age_s, motion, params):
    if not params["motion_compensation_enabled"]:
        return x_forward, y_left, 0.0

    dt = min(
        max(0.0, float(point_age_s)),
        max(0.0, params["motion_max_point_age_s"]),
    )
    if dt <= 0.0:
        return x_forward, y_left, 0.0

    speed_mps = float(motion["speed_mps"])
    yaw_rate_radps = (
        float(motion["yaw_rate_radps"]) * params["motion_yaw_rate_sign"]
    )
    if not math.isfinite(speed_mps) or not math.isfinite(yaw_rate_radps):
        return x_forward, y_left, 0.0
    if params["motion_use_abs_speed"]:
        speed_mps = abs(speed_mps)
    speed_mps = max(
        -params["motion_max_speed_mps"],
        min(params["motion_max_speed_mps"], speed_mps),
    )
    yaw_rate_radps = max(
        -params["motion_max_yaw_rate_radps"],
        min(params["motion_max_yaw_rate_radps"], yaw_rate_radps),
    )

    # Old point -> current ego frame.
    #
    # If the ego moved forward by v*dt after this packet was measured, a static
    # object should be closer in the current frame.  If the ego yawed by theta,
    # rotate the old measurement by -theta into the current vehicle axes.
    shifted_x = x_forward - speed_mps * dt
    theta = yaw_rate_radps * dt
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    compensated_x = cos_theta * shifted_x + sin_theta * y_left
    compensated_y = -sin_theta * shifted_x + cos_theta * y_left
    return compensated_x, compensated_y, dt


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


def _sample_points(points, params, point_age_s=0.0, motion=None):
    sampled = []
    if motion is None:
        motion = {
            "speed_mps": 0.0,
            "yaw_rate_radps": 0.0,
        }
    for point in points:
        if point.distance_m < params["min_distance_m"]:
            continue

        x_forward, y_left = _rotate_xy(
            point.x_m,
            point.y_m,
            params["lidar_yaw_offset_deg"],
        )
        z_up = point.z_m
        x_forward, y_left, compensated_age_s = _motion_compensate_xy(
            x_forward,
            y_left,
            point_age_s,
            motion,
            params,
        )
        bearing_deg = math.degrees(math.atan2(y_left, x_forward))

        if not _is_in_sampled_area(x_forward, y_left, bearing_deg, params):
            continue
        if not (params["z_min_m"] <= z_up <= params["z_max_m"]):
            continue

        compensated_distance_m = math.sqrt(
            x_forward * x_forward + y_left * y_left + z_up * z_up
        )

        sampled.append(
            (
                float(x_forward),
                float(y_left),
                float(z_up),
                float(compensated_distance_m),
                float(point.intensity),
                float(point.laser_id),
                float(bearing_deg),
                float(compensated_age_s),
            )
        )
    return sampled


def _competition_status_listener(params, motion_state):
    status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    try:
        status_socket.bind(
            (
                params["comp_status_bind_ip"],
                params["comp_status_destination_port"],
            )
        )
        status_socket.settimeout(0.2)
        rospy.loginfo(
            "LiDAR motion compensation status UDP: source *:%d -> %s:%d",
            params["comp_status_host_port"],
            params["comp_status_bind_ip"],
            params["comp_status_destination_port"],
        )
        while not rospy.is_shutdown():
            try:
                packet, sender = status_socket.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError as error:
                rospy.logwarn("Competition status socket closed: %s", error)
                return

            if sender[1] != params["comp_status_host_port"]:
                rospy.logwarn_throttle(
                    5.0,
                    "Competition Status UDP sender source port %d, expected %d",
                    sender[1],
                    params["comp_status_host_port"],
                )
            try:
                status = parse_competition_vehicle_status(packet)
            except CompetitionStatusPacketError as error:
                rospy.logwarn_throttle(
                    2.0,
                    "Bad Competition Status packet for LiDAR motion compensation: %s",
                    error,
                )
                continue
            motion_state.update_from_status(status, time.monotonic())
    except OSError as error:
        rospy.logwarn(
            "Cannot bind Competition Status UDP for LiDAR motion compensation "
            "on %s:%d: %s. Falling back to manual_speed_mps/manual_yaw_rate_radps.",
            params["comp_status_bind_ip"],
            params["comp_status_destination_port"],
            error,
        )
    finally:
        status_socket.close()


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


def _left_right_counts(points):
    left = sum(1 for point in points if point[POINT_Y] > 0.2)
    right = sum(1 for point in points if point[POINT_Y] < -0.2)
    center = max(0, len(points) - left - right)
    return left, center, right


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


def _make_obstacle_markers(clusters, frame_id, max_clusters):
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
        "packets_per_cloud": int(_param("packets_per_cloud", 15)),
        "rolling_clouds": int(_param("rolling_clouds", 1)),
        "display_rolling_clouds": int(_param("display_rolling_clouds", 120)),
        "display_max_packets": int(_param("display_max_packets", 120)),
        "display_history_s": float(_param("display_history_s", 1.50)),
        "max_cloud_age_s": float(_param("max_cloud_age_s", 0.05)),
        "socket_timeout_s": float(_param("socket_timeout_s", 1.0)),
        "motion_compensation_enabled": _bool_param(
            "motion_compensation_enabled",
            True,
        ),
        "use_comp_status_motion_compensation": _bool_param(
            "use_comp_status_motion_compensation",
            True,
        ),
        "comp_status_bind_ip": _param("comp_status_bind_ip", BIND_IP),
        "comp_status_host_port": int(
            _param("comp_status_host_port", COMPETITION_STATUS_HOST_PORT)
        ),
        "comp_status_destination_port": int(
            _param("comp_status_destination_port", COMPETITION_STATUS_PORT)
        ),
        "motion_status_timeout_s": float(_param("motion_status_timeout_s", 0.5)),
        "manual_speed_mps": float(_param("manual_speed_mps", 0.0)),
        "manual_yaw_rate_radps": float(_param("manual_yaw_rate_radps", 0.0)),
        "motion_use_abs_speed": _bool_param("motion_use_abs_speed", True),
        "motion_max_speed_mps": float(_param("motion_max_speed_mps", 35.0)),
        "motion_max_yaw_rate_radps": float(
            _param("motion_max_yaw_rate_radps", 2.5)
        ),
        "motion_yaw_rate_sign": float(_param("motion_yaw_rate_sign", 1.0)),
        "motion_max_point_age_s": float(_param("motion_max_point_age_s", 1.5)),
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
    if params["display_max_packets"] < 1:
        raise ValueError("~display_max_packets must be at least 1")
    if params["display_history_s"] < 0.0:
        raise ValueError("~display_history_s cannot be negative")
    if params["max_cloud_age_s"] < 0.0:
        raise ValueError("~max_cloud_age_s cannot be negative")
    if params["motion_status_timeout_s"] < 0.0:
        raise ValueError("~motion_status_timeout_s cannot be negative")
    if params["motion_max_point_age_s"] < 0.0:
        raise ValueError("~motion_max_point_age_s cannot be negative")
    if params["motion_max_speed_mps"] < 0.0:
        raise ValueError("~motion_max_speed_mps cannot be negative")
    if params["motion_max_yaw_rate_radps"] < 0.0:
        raise ValueError("~motion_max_yaw_rate_radps cannot be negative")
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
        "rolling_clouds=%d, display_max_packets=%d, display_history=%.3fs, "
        "max_cloud_age=%.3fs, motion_compensation=%s",
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
        params["display_max_packets"],
        params["display_history_s"],
        params["max_cloud_age_s"],
        "on" if params["motion_compensation_enabled"] else "off",
    )

    rolling_clouds = deque(maxlen=params["rolling_clouds"])
    display_packet_batches = deque(maxlen=params["display_max_packets"])
    motion_state = MotionState(
        params["manual_speed_mps"],
        params["manual_yaw_rate_radps"],
    )
    if (
        params["motion_compensation_enabled"]
        and params["use_comp_status_motion_compensation"]
    ):
        status_thread = threading.Thread(
            target=_competition_status_listener,
            args=(params, motion_state),
            daemon=True,
        )
        status_thread.start()

    try:
        while not rospy.is_shutdown():
            packet_batches = []
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
                packet_received_at = time.monotonic()
                try:
                    lidar_packet = parse_lidar_intensity_packet(packet)
                except (LidarPacketError, ValueError) as error:
                    bad_packets += 1
                    rospy.logwarn_throttle(2.0, "Bad LiDAR packet: %s", error)
                    continue

                packet_batches.append((packet_received_at, lidar_packet.points))

            publish_started_at = time.monotonic()
            motion = motion_state.snapshot(
                publish_started_at,
                params["motion_status_timeout_s"],
                params["manual_speed_mps"],
                params["manual_yaw_rate_radps"],
            )
            if (
                params["motion_compensation_enabled"]
                and params["use_comp_status_motion_compensation"]
                and not motion["fresh"]
            ):
                rospy.logwarn_throttle(
                    2.0,
                    "Competition Status not ready/stale for LiDAR motion "
                    "compensation; using manual motion speed=%.2fm/s yaw_rate=%.3frad/s",
                    motion["speed_mps"],
                    motion["yaw_rate_radps"],
                )
            cloud_points = []
            for packet_received_at, points in packet_batches:
                point_age_s = publish_started_at - packet_received_at
                cloud_points.extend(
                    _sample_points(
                        points,
                        params,
                        point_age_s=point_age_s,
                        motion=motion,
                    )
                )
                display_packet_batches.append((packet_received_at, points))
            if cloud_points:
                rolling_clouds.append(cloud_points)
            accumulated_points = [
                point
                for cloud in rolling_clouds
                for point in cloud
            ]
            now = time.monotonic()
            while (
                display_packet_batches
                and params["display_history_s"] > 0.0
                and now - display_packet_batches[0][0] > params["display_history_s"]
            ):
                display_packet_batches.popleft()
            display_points = []
            for packet_received_at, points in display_packet_batches:
                display_points.extend(
                    _sample_points(
                        points,
                        params,
                        point_age_s=now - packet_received_at,
                        motion=motion,
                    )
                )
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
                        _make_obstacle_markers(
                            clusters,
                            params["frame_id"],
                            params["cluster_max_clusters"],
                        )
                    )
            if display_points:
                display_publisher.publish(_make_cloud(display_points, params["frame_id"]))
            nearest_text = _nearest_distance(accumulated_points, params)
            live_left, live_center, live_right = _left_right_counts(accumulated_points)
            display_left, display_center, display_right = _left_right_counts(display_points)
            rospy.loginfo_throttle(
                1.0,
                "LiDAR cloud: packets=%d bad=%d live_points=%d display_points=%d "
                "live_l/c/r=%d/%d/%d display_l/c/r=%d/%d/%d "
                "nearest=%s clusters=%d age=%.3fs motion=%s fresh=%s "
                "speed=%.2fm/s yaw_rate=%.3frad/s status_age=%s",
                packets,
                bad_packets,
                len(accumulated_points),
                len(display_points),
                live_left,
                live_center,
                live_right,
                display_left,
                display_center,
                display_right,
                "n/a" if nearest_text is None else "{:.2f}m".format(nearest_text),
                len(clusters),
                time.monotonic() - cloud_started_at,
                "on" if params["motion_compensation_enabled"] else "off",
                motion["fresh"],
                motion["speed_mps"],
                motion["yaw_rate_radps"],
                "n/a"
                if motion["status_age_s"] is None
                else "{:.3f}s".format(motion["status_age_s"]),
            )
    finally:
        udp_socket.close()


if __name__ == "__main__":
    main()
