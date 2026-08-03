"""Geometry filters that reject unsupported LiDAR scan-line arcs."""

import math
from collections import defaultdict


POINT_X = 0
POINT_Y = 1
POINT_Z = 2
POINT_DISTANCE = 3
POINT_RING = 5
POINT_BEARING = 6


def filter_vertical_support(points, radius_m, minimum_height_m):
    """Keep points supported by a vertically separated return from another ring.

    Flat-road scan arcs are normally produced by one laser ring at a nearly
    constant height.  A physical obstacle instead receives returns from two or
    more vertical channels at almost the same horizontal location.
    """
    points = list(points)
    if len(points) < 2:
        return []

    radius = float(radius_m)
    minimum_height = float(minimum_height_m)
    if radius <= 0.0:
        raise ValueError("vertical support radius must be positive")
    if minimum_height < 0.0:
        raise ValueError("vertical support minimum height cannot be negative")

    grid = defaultdict(list)
    for index, point in enumerate(points):
        key = (
            int(math.floor(point[POINT_X] / radius)),
            int(math.floor(point[POINT_Y] / radius)),
        )
        grid[key].append(index)

    radius_sq = radius * radius
    supported = [False] * len(points)
    for index, point in enumerate(points):
        cell_x = int(math.floor(point[POINT_X] / radius))
        cell_y = int(math.floor(point[POINT_Y] / radius))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for neighbor_index in grid.get((cell_x + dx, cell_y + dy), ()):
                    if neighbor_index == index:
                        continue
                    neighbor = points[neighbor_index]
                    if int(point[POINT_RING]) == int(neighbor[POINT_RING]):
                        continue
                    horizontal_distance_sq = (
                        (point[POINT_X] - neighbor[POINT_X]) ** 2
                        + (point[POINT_Y] - neighbor[POINT_Y]) ** 2
                    )
                    if horizontal_distance_sq > radius_sq:
                        continue
                    if abs(point[POINT_Z] - neighbor[POINT_Z]) < minimum_height:
                        continue
                    supported[index] = True
                    supported[neighbor_index] = True
                    break
                if supported[index]:
                    break
            if supported[index]:
                break

    return [point for index, point in enumerate(points) if supported[index]]


def compact_single_ring_groups(
    points,
    minimum_points,
    maximum_width_m,
    maximum_angle_gap_deg=1.5,
    maximum_range_jump_m=0.5,
):
    """Return short same-ring runs that can represent a low obstacle.

    A ground return normally forms a long circular run. A low object that is
    hit by only one vertical laser produces a short run separated by an angle
    or range discontinuity. Temporal confirmation is intentionally handled by
    the caller before these groups become published obstacles.
    """
    minimum_count = int(minimum_points)
    maximum_width = float(maximum_width_m)
    maximum_angle_gap = float(maximum_angle_gap_deg)
    maximum_range_jump = float(maximum_range_jump_m)
    if minimum_count < 2:
        raise ValueError("compact group minimum points must be at least 2")
    if maximum_width <= 0.0:
        raise ValueError("compact group maximum width must be positive")
    if maximum_angle_gap <= 0.0 or maximum_range_jump < 0.0:
        raise ValueError("compact group continuity thresholds are invalid")

    by_ring = defaultdict(list)
    for point in points:
        by_ring[int(point[POINT_RING])].append(point)

    groups = []
    for ring_points in by_ring.values():
        ring_points.sort(key=lambda point: point[POINT_BEARING])
        runs = []
        run = []
        for point in ring_points:
            if run:
                previous = run[-1]
                if (
                    point[POINT_BEARING] - previous[POINT_BEARING]
                    > maximum_angle_gap
                    or abs(point[POINT_DISTANCE] - previous[POINT_DISTANCE])
                    > maximum_range_jump
                ):
                    runs.append(run)
                    run = []
            run.append(point)
        if run:
            runs.append(run)

        for run in runs:
            if len(run) < minimum_count:
                continue
            xs = [point[POINT_X] for point in run]
            ys = [point[POINT_Y] for point in run]
            width = max(max(xs) - min(xs), max(ys) - min(ys))
            if width <= maximum_width:
                groups.append(run)
    return groups


def confirm_cluster_centroids(current_clusters, previous_clusters, distance_m):
    """Keep candidates that repeat near a candidate from the previous scan."""
    maximum_distance_sq = float(distance_m) ** 2
    confirmed = []
    for current in current_clusters:
        for previous in previous_clusters:
            distance_sq = (
                current["centroid_x_m"] - previous["centroid_x_m"]
            ) ** 2 + (
                current["centroid_y_m"] - previous["centroid_y_m"]
            ) ** 2
            if distance_sq <= maximum_distance_sq:
                confirmed.append(current)
                break
    return confirmed
