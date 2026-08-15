#!/usr/bin/env python3
"""Map-frame helpers for tracked LiDAR obstacles."""

from collections import deque
import math

import numpy as np


def wrap_angle(angle_rad):
    """Wrap an angle to [-pi, pi)."""

    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(x, y, z, w):
    """Extract ROS ENU yaw from a quaternion."""

    sin_yaw = 2.0 * (float(w) * float(z) + float(x) * float(y))
    cos_yaw = 1.0 - 2.0 * (float(y) ** 2 + float(z) ** 2)
    return math.atan2(sin_yaw, cos_yaw)


class Pose2DHistory:
    """Small timestamped pose buffer with wrap-safe yaw interpolation."""

    def __init__(self, history_s=2.0):
        self.history_s = float(history_s)
        if self.history_s <= 0.0:
            raise ValueError("history_s must be positive")
        self._samples = deque()

    def append(self, timestamp_s, x_m, y_m, yaw_rad):
        sample = (
            float(timestamp_s),
            float(x_m),
            float(y_m),
            wrap_angle(yaw_rad),
        )
        if self._samples and sample[0] < self._samples[-1][0]:
            return False
        self._samples.append(sample)
        cutoff = sample[0] - self.history_s
        while len(self._samples) > 2 and self._samples[1][0] < cutoff:
            self._samples.popleft()
        return True

    def pose_at(self, timestamp_s, maximum_age_s=0.25):
        if not self._samples:
            return None
        timestamp = float(timestamp_s)
        maximum_age = float(maximum_age_s)
        if maximum_age < 0.0:
            raise ValueError("maximum_age_s cannot be negative")

        first = self._samples[0]
        last = self._samples[-1]
        if timestamp <= first[0]:
            return first[1:] if first[0] - timestamp <= maximum_age else None
        if timestamp >= last[0]:
            return last[1:] if timestamp - last[0] <= maximum_age else None

        for left, right in zip(self._samples, list(self._samples)[1:]):
            if left[0] <= timestamp <= right[0]:
                span = right[0] - left[0]
                ratio = 0.0 if span <= 1.0e-9 else (timestamp - left[0]) / span
                yaw_delta = wrap_angle(right[3] - left[3])
                return (
                    left[1] + ratio * (right[1] - left[1]),
                    left[2] + ratio * (right[2] - left[2]),
                    wrap_angle(left[3] + ratio * yaw_delta),
                )
        return None


def oriented_bounding_box_2d(points, cluster_id=0):
    """Return a PCA-oriented 2D rectangle for one non-empty point cluster.

    The rectangle yaw describes its long axis. A rectangle alone cannot
    distinguish front from rear, so this yaw is initially defined modulo pi.
    """

    point_array = np.asarray(list(points), dtype=float)
    if point_array.ndim != 2 or point_array.shape[0] == 0 or point_array.shape[1] < 3:
        raise ValueError("an oriented bounding box requires xyz points")
    if not np.all(np.isfinite(point_array[:, :3])):
        raise ValueError("oriented bounding box points must be finite")

    xy = point_array[:, :2]
    mean_xy = np.mean(xy, axis=0)
    centered = xy - mean_xy
    if len(xy) >= 2 and np.any(np.abs(centered) > 1.0e-9):
        covariance = centered.T.dot(centered) / float(len(xy))
        values, vectors = np.linalg.eigh(covariance)
        long_axis = vectors[:, int(np.argmax(values))]
    else:
        long_axis = np.array([1.0, 0.0], dtype=float)
    short_axis = np.array([-long_axis[1], long_axis[0]], dtype=float)

    projected_long = centered.dot(long_axis)
    projected_short = centered.dot(short_axis)
    long_min, long_max = float(np.min(projected_long)), float(np.max(projected_long))
    short_min, short_max = float(np.min(projected_short)), float(np.max(projected_short))
    length_m = long_max - long_min
    width_m = short_max - short_min
    if width_m > length_m:
        long_axis, short_axis = short_axis, -long_axis
        projected_long, projected_short = projected_short, -projected_long
        long_min, long_max = float(np.min(projected_long)), float(np.max(projected_long))
        short_min, short_max = float(np.min(projected_short)), float(np.max(projected_short))
        length_m, width_m = long_max - long_min, short_max - short_min

    center_xy = (
        mean_xy
        + 0.5 * (long_min + long_max) * long_axis
        + 0.5 * (short_min + short_max) * short_axis
    )
    zs = point_array[:, 2]
    yaw_rad = wrap_angle(math.atan2(long_axis[1], long_axis[0]))
    if yaw_rad >= 0.5 * math.pi:
        yaw_rad -= math.pi
    elif yaw_rad < -0.5 * math.pi:
        yaw_rad += math.pi

    return {
        "cluster_id": int(cluster_id),
        "point_count": int(len(point_array)),
        "center_x_m": float(center_xy[0]),
        "center_y_m": float(center_xy[1]),
        "center_z_m": 0.5 * (float(np.min(zs)) + float(np.max(zs))),
        "length_m": float(length_m),
        "width_m": float(width_m),
        "height_m": float(np.max(zs) - np.min(zs)),
        "yaw_rad": float(yaw_rad),
    }


def oriented_bounding_boxes(points, cluster_indices):
    point_list = list(points)
    return [
        oriented_bounding_box_2d(
            (point_list[index] for index in member_indices),
            cluster_id=cluster_id,
        )
        for cluster_id, member_indices in enumerate(cluster_indices)
    ]


def transform_box_to_map(
    box,
    ego_x_m,
    ego_y_m,
    ego_yaw_rad,
    lidar_x_m=0.0,
    lidar_y_m=0.0,
    lidar_yaw_rad=0.0,
):
    """Transform one LiDAR-frame oriented box into the map frame."""

    lidar_cos = math.cos(float(lidar_yaw_rad))
    lidar_sin = math.sin(float(lidar_yaw_rad))
    base_x = (
        float(lidar_x_m)
        + lidar_cos * float(box["center_x_m"])
        - lidar_sin * float(box["center_y_m"])
    )
    base_y = (
        float(lidar_y_m)
        + lidar_sin * float(box["center_x_m"])
        + lidar_cos * float(box["center_y_m"])
    )
    ego_cos = math.cos(float(ego_yaw_rad))
    ego_sin = math.sin(float(ego_yaw_rad))
    result = dict(box)
    result.update(
        {
            "center_x_m": float(ego_x_m) + ego_cos * base_x - ego_sin * base_y,
            "center_y_m": float(ego_y_m) + ego_sin * base_x + ego_cos * base_y,
            "yaw_rad": wrap_angle(
                float(ego_yaw_rad)
                + float(lidar_yaw_rad)
                + float(box["yaw_rad"])
            ),
            "size_x_m": float(box["length_m"]),
            "size_y_m": float(box["width_m"]),
            "size_z_m": float(box["height_m"]),
        }
    )
    return result


def resolve_box_heading(yaw_rad, velocity_x_mps, velocity_y_mps, minimum_speed_mps=0.5):
    """Resolve the pi ambiguity with motion direction when speed is sufficient."""

    yaw = wrap_angle(yaw_rad)
    vx = float(velocity_x_mps)
    vy = float(velocity_y_mps)
    if math.hypot(vx, vy) < float(minimum_speed_mps):
        return yaw
    motion_yaw = math.atan2(vy, vx)
    if math.cos(yaw - motion_yaw) < 0.0:
        yaw = wrap_angle(yaw + math.pi)
    return yaw
