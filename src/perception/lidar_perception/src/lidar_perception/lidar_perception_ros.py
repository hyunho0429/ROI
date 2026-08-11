#!/usr/bin/env python3
"""ROS/RViz adapters for standalone UDP LiDAR perception demos.

MORAI sensor data is received by ``morai_lidar_pointcloud_udp_ground.py`` via
UDP.  The ROS topics used here are local-only transport to the algorithms and
RViz; no MORAI ROS sensor topic is consumed.
"""

import colorsys
import json
import math
import struct

import rospy
from geometry_msgs.msg import Point
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from lidar_perception.lidar_bounding_box import bounding_boxes
from lidar_perception.lidar_euclidean_clustering import (
    euclidean_cluster_indices,
    select_roi,
)
from lidar_perception.lidar_kalman_hungarian import KalmanHungarianTracker


CLUSTER_FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("rgb", 12, PointField.FLOAT32, 1),
]


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _cluster_color(identifier, alpha=1.0):
    hue = (int(identifier) * 0.61803398875) % 1.0
    red, green, blue = colorsys.hsv_to_rgb(hue, 0.85, 1.0)
    return red, green, blue, float(alpha)


def _packed_rgb_float(red, green, blue):
    packed = (
        (int(max(0.0, min(1.0, red)) * 255.0) << 16)
        | (int(max(0.0, min(1.0, green)) * 255.0) << 8)
        | int(max(0.0, min(1.0, blue)) * 255.0)
    )
    return struct.unpack("f", struct.pack("I", packed))[0]


def _header(stamp, frame_id):
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header


def _cluster_cloud(points, cluster_indices, stamp, frame_id):
    colored_points = []
    for cluster_id, member_indices in enumerate(cluster_indices):
        red, green, blue, _alpha = _cluster_color(cluster_id)
        rgb = _packed_rgb_float(red, green, blue)
        for point_index in member_indices:
            x_m, y_m, z_m = points[point_index]
            colored_points.append((x_m, y_m, z_m, rgb))
    return point_cloud2.create_cloud(
        _header(stamp, frame_id),
        CLUSTER_FIELDS,
        colored_points,
    )


def _marker_base(stamp, frame_id, namespace, marker_id, marker_type, lifetime_s):
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.lifetime = rospy.Duration(float(lifetime_s))
    return marker


def _set_color(marker, color):
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color


def _centroid(points, member_indices):
    count = float(len(member_indices))
    return (
        sum(points[index][0] for index in member_indices) / count,
        sum(points[index][1] for index in member_indices) / count,
        sum(points[index][2] for index in member_indices) / count,
    )


def _cluster_markers(points, cluster_indices, stamp, frame_id, lifetime_s):
    marker_array = MarkerArray()
    for cluster_id, member_indices in enumerate(cluster_indices):
        center_x, center_y, center_z = _centroid(points, member_indices)
        color = _cluster_color(cluster_id)

        center = _marker_base(
            stamp,
            frame_id,
            "euclidean_centroids",
            cluster_id,
            Marker.SPHERE,
            lifetime_s,
        )
        center.pose.position.x = center_x
        center.pose.position.y = center_y
        center.pose.position.z = center_z
        center.scale.x = center.scale.y = center.scale.z = 0.28
        _set_color(center, color)
        marker_array.markers.append(center)

        label = _marker_base(
            stamp,
            frame_id,
            "euclidean_labels",
            cluster_id,
            Marker.TEXT_VIEW_FACING,
            lifetime_s,
        )
        label.pose.position.x = center_x
        label.pose.position.y = center_y
        label.pose.position.z = center_z + 0.55
        label.scale.z = 0.42
        _set_color(label, (1.0, 1.0, 1.0, 1.0))
        label.text = "C{} n={} d={:.1f}m".format(
            cluster_id,
            len(member_indices),
            math.hypot(center_x, center_y),
        )
        marker_array.markers.append(label)
    return marker_array


def _box_edge_points(box):
    half_x = max(0.025, 0.5 * float(box["size_x_m"]))
    half_y = max(0.025, 0.5 * float(box["size_y_m"]))
    half_z = max(0.025, 0.5 * float(box["size_z_m"]))
    center_x = float(box["center_x_m"])
    center_y = float(box["center_y_m"])
    center_z = float(box["center_z_m"])
    corners = [
        Point(center_x + sx * half_x, center_y + sy * half_y, center_z + sz * half_z)
        for sx in (-1.0, 1.0)
        for sy in (-1.0, 1.0)
        for sz in (-1.0, 1.0)
    ]
    edges = (
        (0, 1), (0, 2), (0, 4),
        (1, 3), (1, 5),
        (2, 3), (2, 6),
        (3, 7),
        (4, 5), (4, 6),
        (5, 7),
        (6, 7),
    )
    return [corners[index] for edge in edges for index in edge]


def _append_box_markers(
    marker_array,
    box,
    display_id,
    stamp,
    frame_id,
    lifetime_s,
    namespace_prefix,
    color,
    label_text,
):
    marker_id = int(display_id) * 3

    fill = _marker_base(
        stamp,
        frame_id,
        namespace_prefix + "_fill",
        marker_id,
        Marker.CUBE,
        lifetime_s,
    )
    fill.pose.position.x = float(box["center_x_m"])
    fill.pose.position.y = float(box["center_y_m"])
    fill.pose.position.z = float(box["center_z_m"])
    fill.scale.x = max(0.05, float(box["size_x_m"]))
    fill.scale.y = max(0.05, float(box["size_y_m"]))
    fill.scale.z = max(0.05, float(box["size_z_m"]))
    _set_color(fill, (color[0], color[1], color[2], 0.18))
    marker_array.markers.append(fill)

    edges = _marker_base(
        stamp,
        frame_id,
        namespace_prefix + "_edges",
        marker_id + 1,
        Marker.LINE_LIST,
        lifetime_s,
    )
    edges.scale.x = 0.08
    _set_color(edges, (color[0], color[1], color[2], 1.0))
    edges.points = _box_edge_points(box)
    marker_array.markers.append(edges)

    label = _marker_base(
        stamp,
        frame_id,
        namespace_prefix + "_labels",
        marker_id + 2,
        Marker.TEXT_VIEW_FACING,
        lifetime_s,
    )
    label.pose.position.x = float(box["center_x_m"])
    label.pose.position.y = float(box["center_y_m"])
    label.pose.position.z = (
        float(box["center_z_m"]) + 0.5 * float(box["size_z_m"]) + 0.45
    )
    label.scale.z = 0.40
    _set_color(label, (1.0, 1.0, 1.0, 1.0))
    label.text = label_text
    marker_array.markers.append(label)


def _box_markers(boxes, stamp, frame_id, lifetime_s):
    marker_array = MarkerArray()
    for box in boxes:
        cluster_id = int(box["cluster_id"])
        _append_box_markers(
            marker_array,
            box,
            cluster_id,
            stamp,
            frame_id,
            lifetime_s,
            "aabb",
            _cluster_color(cluster_id),
            "B{} {:.1f}m  {:.1f}x{:.1f}x{:.1f}".format(
                cluster_id,
                box["distance_m"],
                box["size_x_m"],
                box["size_y_m"],
                box["size_z_m"],
            ),
        )
    return marker_array


def _tracking_markers(tracks, stamp, frame_id, lifetime_s, velocity_scale_s):
    marker_array = MarkerArray()
    for track in tracks:
        track_id = int(track["track_id"])
        if track["confirmed"]:
            color = _cluster_color(track_id)
            state_text = "CONF"
        else:
            color = (1.0, 0.75, 0.1, 1.0)
            state_text = "TENT"

        _append_box_markers(
            marker_array,
            track,
            track_id,
            stamp,
            frame_id,
            lifetime_s,
            "tracks",
            color,
            "T{} {} v={:.1f}m/s miss={}".format(
                track_id,
                state_text,
                track["speed_mps"],
                track["misses"],
            ),
        )

        arrow = _marker_base(
            stamp,
            frame_id,
            "track_velocity",
            track_id,
            Marker.ARROW,
            lifetime_s,
        )
        start = Point(
            float(track["center_x_m"]),
            float(track["center_y_m"]),
            float(track["center_z_m"]),
        )
        end = Point(
            start.x + float(track["velocity_x_mps"]) * float(velocity_scale_s),
            start.y + float(track["velocity_y_mps"]) * float(velocity_scale_s),
            start.z,
        )
        arrow.points = [start, end]
        arrow.scale.x = 0.08
        arrow.scale.y = 0.18
        arrow.scale.z = 0.22
        _set_color(arrow, color)
        marker_array.markers.append(arrow)
    return marker_array


def _round_dict(values, keys):
    result = {}
    for key in keys:
        value = values[key]
        if isinstance(value, float):
            result[key] = round(value, 3)
        else:
            result[key] = value
    return result


class _ClusterProcessor:
    def __init__(self, result_prefix):
        self.input_topic = _param("input_topic", "/morai/lidar/live_points")
        self.cluster_cloud_topic = _param(
            "cluster_cloud_topic",
            "/morai/lidar/{}/clustered_points".format(result_prefix),
        )
        self.marker_topic = _param(
            "marker_topic",
            "/morai/lidar/{}/markers".format(result_prefix),
        )
        self.result_topic = _param(
            "result_topic",
            "/morai/lidar/{}/results".format(result_prefix),
        )
        self.tolerance_m = float(_param("cluster_tolerance_m", 0.8))
        self.min_points = int(_param("cluster_min_points", 3))
        self.max_points = int(_param("cluster_max_points", 0))
        self.min_height_m = float(_param("cluster_min_height_m", 0.10))
        self.max_clusters = int(_param("cluster_max_clusters", 32))
        self.max_input_points = int(_param("cluster_max_input_points", 5000))
        self.x_min_m = float(_param("x_min_m", -40.0))
        self.x_max_m = float(_param("x_max_m", 40.0))
        self.y_abs_m = float(_param("y_abs_m", 8.0))
        self.z_min_m = float(_param("z_min_m", -1.4))
        self.z_max_m = float(_param("z_max_m", 2.5))
        self.marker_lifetime_s = float(_param("marker_lifetime_s", 0.15))

        if self.max_input_points < 0:
            raise ValueError("cluster_max_input_points cannot be negative")
        if self.x_max_m <= self.x_min_m:
            raise ValueError("x range must satisfy min < max")
        if self.y_abs_m < 0.0:
            raise ValueError("y_abs_m cannot be negative")
        if self.z_max_m <= self.z_min_m:
            raise ValueError("z range must satisfy min < max")
        if self.marker_lifetime_s <= 0.0:
            raise ValueError("marker_lifetime_s must be positive")

        self.cluster_cloud_publisher = rospy.Publisher(
            self.cluster_cloud_topic,
            PointCloud2,
            queue_size=1,
        )
        self.marker_publisher = rospy.Publisher(
            self.marker_topic,
            MarkerArray,
            queue_size=1,
        )
        self.result_publisher = rospy.Publisher(
            self.result_topic,
            String,
            queue_size=1,
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self._callback,
            queue_size=1,
            buff_size=4 * 1024 * 1024,
        )

    def _extract_clusters(self, message):
        raw_points = point_cloud2.read_points(
            message,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        points = select_roi(
            raw_points,
            self.x_min_m,
            self.x_max_m,
            self.y_abs_m,
            self.z_min_m,
            self.z_max_m,
        )
        if self.max_input_points and len(points) > self.max_input_points:
            step = int(math.ceil(float(len(points)) / self.max_input_points))
            points = points[::step]
        cluster_indices = euclidean_cluster_indices(
            points,
            tolerance_m=self.tolerance_m,
            min_points=self.min_points,
            max_points=self.max_points,
            min_height_m=self.min_height_m,
            max_clusters=self.max_clusters,
        )
        stamp = message.header.stamp
        if stamp.to_sec() <= 0.0:
            stamp = rospy.Time.now()
        frame_id = message.header.frame_id or "morai_lidar"
        self.cluster_cloud_publisher.publish(
            _cluster_cloud(points, cluster_indices, stamp, frame_id)
        )
        return points, cluster_indices, stamp, frame_id


class EuclideanClusteringNode(_ClusterProcessor):
    def __init__(self):
        super().__init__("euclidean")
        rospy.loginfo(
            "Euclidean clustering: input=%s tolerance=%.2fm min_points=%d",
            self.input_topic,
            self.tolerance_m,
            self.min_points,
        )

    def _callback(self, message):
        points, cluster_indices, stamp, frame_id = self._extract_clusters(message)
        self.marker_publisher.publish(
            _cluster_markers(
                points,
                cluster_indices,
                stamp,
                frame_id,
                self.marker_lifetime_s,
            )
        )
        results = []
        for cluster_id, member_indices in enumerate(cluster_indices):
            center_x, center_y, center_z = _centroid(points, member_indices)
            results.append(
                {
                    "cluster_id": cluster_id,
                    "point_count": len(member_indices),
                    "center_x_m": round(center_x, 3),
                    "center_y_m": round(center_y, 3),
                    "center_z_m": round(center_z, 3),
                    "distance_m": round(math.hypot(center_x, center_y), 3),
                }
            )
        self.result_publisher.publish(String(data=json.dumps(results)))
        rospy.loginfo_throttle(
            1.0,
            "Euclidean: roi_points=%d clusters=%d",
            len(points),
            len(cluster_indices),
        )


class BoundingBoxNode(_ClusterProcessor):
    BOX_KEYS = (
        "cluster_id", "point_count",
        "center_x_m", "center_y_m", "center_z_m",
        "size_x_m", "size_y_m", "size_z_m",
        "distance_m", "bearing_deg",
    )

    def __init__(self):
        super().__init__("bbox")
        rospy.loginfo(
            "3D AABB: input=%s tolerance=%.2fm min_points=%d",
            self.input_topic,
            self.tolerance_m,
            self.min_points,
        )

    def _callback(self, message):
        points, cluster_indices, stamp, frame_id = self._extract_clusters(message)
        boxes = bounding_boxes(points, cluster_indices)
        self.marker_publisher.publish(
            _box_markers(boxes, stamp, frame_id, self.marker_lifetime_s)
        )
        results = [_round_dict(box, self.BOX_KEYS) for box in boxes]
        self.result_publisher.publish(String(data=json.dumps(results)))
        rospy.loginfo_throttle(
            1.0,
            "3D AABB: roi_points=%d boxes=%d",
            len(points),
            len(boxes),
        )


class KalmanHungarianNode(_ClusterProcessor):
    TRACK_KEYS = (
        "track_id", "confirmed", "hits", "misses", "point_count",
        "center_x_m", "center_y_m", "center_z_m",
        "size_x_m", "size_y_m", "size_z_m",
        "velocity_x_mps", "velocity_y_mps", "speed_mps",
    )

    def __init__(self):
        self.velocity_scale_s = float(_param("velocity_scale_s", 1.0))
        self.tracker = KalmanHungarianTracker(
            match_distance_m=float(_param("match_distance_m", 3.0)),
            max_missed=int(_param("max_missed", 6)),
            min_hits=int(_param("min_hits", 3)),
            process_accel_std_mps2=float(
                _param("process_accel_std_mps2", 4.0)
            ),
            measurement_noise_m=float(_param("measurement_noise_m", 0.35)),
        )
        super().__init__("tracking")
        rospy.loginfo(
            "Kalman+Hungarian: input=%s gate=%.2fm min_hits=%d max_missed=%d",
            self.input_topic,
            self.tracker.match_distance_m,
            self.tracker.min_hits,
            self.tracker.max_missed,
        )
        rospy.logwarn(
            "Tracker velocities are ego-relative in morai_lidar frame; "
            "the UDP source deskews each scan but does not transform tracks "
            "to a world frame between scans."
        )

    def _callback(self, message):
        points, cluster_indices, stamp, frame_id = self._extract_clusters(message)
        detections = bounding_boxes(points, cluster_indices)
        tracks = self.tracker.update(detections, stamp.to_sec())
        self.marker_publisher.publish(
            _tracking_markers(
                tracks,
                stamp,
                frame_id,
                self.marker_lifetime_s,
                self.velocity_scale_s,
            )
        )
        results = [_round_dict(track, self.TRACK_KEYS) for track in tracks]
        self.result_publisher.publish(String(data=json.dumps(results)))
        confirmed_count = sum(1 for track in tracks if track["confirmed"])
        rospy.loginfo_throttle(
            1.0,
            "Kalman+Hungarian: detections=%d tracks=%d confirmed=%d",
            len(detections),
            len(tracks),
            confirmed_count,
        )


def run_euclidean_clustering():
    rospy.init_node("morai_lidar_euclidean_clustering")
    EuclideanClusteringNode()
    rospy.spin()


def run_bounding_box():
    rospy.init_node("morai_lidar_3d_bounding_box")
    BoundingBoxNode()
    rospy.spin()


def run_kalman_hungarian():
    rospy.init_node("morai_lidar_kalman_hungarian")
    KalmanHungarianNode()
    rospy.spin()
