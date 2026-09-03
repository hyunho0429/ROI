#!/usr/bin/env python3
"""Axis-aligned 3D bounding boxes for clustered LiDAR points."""

import math


def axis_aligned_bounding_box(points, cluster_id=0):
    """Summarize one non-empty cluster as an ego-frame 3D AABB."""

    point_list = list(points)
    if not point_list:
        raise ValueError("a bounding box requires at least one point")

    xs = [float(point[0]) for point in point_list]
    ys = [float(point[1]) for point in point_list]
    zs = [float(point[2]) for point in point_list]
    min_x_m, max_x_m = min(xs), max(xs)
    min_y_m, max_y_m = min(ys), max(ys)
    min_z_m, max_z_m = min(zs), max(zs)
    center_x_m = 0.5 * (min_x_m + max_x_m)
    center_y_m = 0.5 * (min_y_m + max_y_m)
    center_z_m = 0.5 * (min_z_m + max_z_m)

    return {
        "cluster_id": int(cluster_id),
        "point_count": len(point_list),
        "center_x_m": center_x_m,
        "center_y_m": center_y_m,
        "center_z_m": center_z_m,
        "size_x_m": max_x_m - min_x_m,
        "size_y_m": max_y_m - min_y_m,
        "size_z_m": max_z_m - min_z_m,
        "min_x_m": min_x_m,
        "max_x_m": max_x_m,
        "min_y_m": min_y_m,
        "max_y_m": max_y_m,
        "min_z_m": min_z_m,
        "max_z_m": max_z_m,
        "distance_m": math.hypot(center_x_m, center_y_m),
        "bearing_deg": math.degrees(math.atan2(center_y_m, center_x_m)),
    }


def bounding_boxes(points, cluster_indices):
    """Build an AABB for each cluster-index group."""

    point_list = list(points)
    return [
        axis_aligned_bounding_box(
            (point_list[index] for index in member_indices),
            cluster_id=cluster_id,
        )
        for cluster_id, member_indices in enumerate(cluster_indices)
    ]


def box_corners(box):
    """Return the eight corners of an axis-aligned box."""

    return [
        (x_m, y_m, z_m)
        for x_m in (box["min_x_m"], box["max_x_m"])
        for y_m in (box["min_y_m"], box["max_y_m"])
        for z_m in (box["min_z_m"], box["max_z_m"])
    ]
