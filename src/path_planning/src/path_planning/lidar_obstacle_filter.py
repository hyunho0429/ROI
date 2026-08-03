"""Geometry filters that reject unsupported LiDAR scan-line arcs."""

import math
from collections import defaultdict


POINT_X = 0
POINT_Y = 1
POINT_Z = 2
POINT_DISTANCE = 3
POINT_RING = 5
POINT_BEARING = 6


def _vertical_support_mask(points, radius_m, minimum_height_m):
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

    return supported


def filter_vertical_support(points, radius_m, minimum_height_m):
    """Keep only vertically supported points; retained for strict filtering."""
    points = list(points)
    if len(points) < 2:
        return []
    supported = _vertical_support_mask(points, radius_m, minimum_height_m)
    return [point for index, point in enumerate(points) if supported[index]]


def filter_scan_line_arcs(
    points,
    support_radius_m,
    support_minimum_height_m,
    arc_minimum_points,
    arc_minimum_angle_deg,
    arc_minimum_length_m,
    arc_maximum_radial_thickness_m,
    neighbor_maximum_angle_gap_deg=1.5,
    neighbor_maximum_range_jump_m=0.5,
):
    """Reject long, thin unsupported ring arcs while retaining compact targets.

    Multi-ring vertical returns are always retained. Unsupported returns are
    rejected only when a same-ring run is long, angularly continuous and has
    the nearly constant radius characteristic of a flat-ground scan arc.
    """
    points = list(points)
    if len(points) < 2:
        return points

    minimum_points = int(arc_minimum_points)
    minimum_angle = float(arc_minimum_angle_deg)
    minimum_length = float(arc_minimum_length_m)
    maximum_radial_thickness = float(arc_maximum_radial_thickness_m)
    maximum_angle_gap = float(neighbor_maximum_angle_gap_deg)
    maximum_range_jump = float(neighbor_maximum_range_jump_m)
    if minimum_points < 2:
        raise ValueError("arc minimum points must be at least 2")
    if minimum_angle < 0.0 or minimum_length < 0.0:
        raise ValueError("arc angle and length thresholds cannot be negative")
    if maximum_radial_thickness < 0.0:
        raise ValueError("arc radial thickness cannot be negative")
    if maximum_angle_gap <= 0.0 or maximum_range_jump < 0.0:
        raise ValueError("arc neighbor thresholds are invalid")

    vertically_supported = _vertical_support_mask(
        points,
        support_radius_m,
        support_minimum_height_m,
    )
    unsupported_by_ring = defaultdict(list)
    for index, point in enumerate(points):
        if not vertically_supported[index]:
            unsupported_by_ring[int(point[POINT_RING])].append(index)

    rejected = [False] * len(points)
    for indices in unsupported_by_ring.values():
        indices.sort(key=lambda index: points[index][POINT_BEARING])
        runs = []
        run = []
        for index in indices:
            if run:
                previous = points[run[-1]]
                current = points[index]
                angle_gap = current[POINT_BEARING] - previous[POINT_BEARING]
                range_jump = abs(
                    current[POINT_DISTANCE] - previous[POINT_DISTANCE]
                )
                if (
                    angle_gap > maximum_angle_gap
                    or range_jump > maximum_range_jump
                ):
                    runs.append(run)
                    run = []
            run.append(index)
        if run:
            runs.append(run)

        for run in runs:
            if len(run) < minimum_points:
                continue
            run_points = [points[index] for index in run]
            angle_span = (
                run_points[-1][POINT_BEARING]
                - run_points[0][POINT_BEARING]
            )
            distances = [point[POINT_DISTANCE] for point in run_points]
            radial_thickness = max(distances) - min(distances)
            first = run_points[0]
            last = run_points[-1]
            chord_length = math.hypot(
                last[POINT_X] - first[POINT_X],
                last[POINT_Y] - first[POINT_Y],
            )
            if (
                angle_span >= minimum_angle
                and chord_length >= minimum_length
                and radial_thickness <= maximum_radial_thickness
            ):
                for index in run:
                    rejected[index] = True

    return [point for index, point in enumerate(points) if not rejected[index]]


def is_obstacle_cluster_geometry(
    points,
    minimum_height_m,
    small_object_minimum_points,
    small_object_maximum_width_m,
):
    """Accept either a vertically tall cluster or a compact small target."""
    points = list(points)
    if not points:
        return False
    xs = [point[POINT_X] for point in points]
    ys = [point[POINT_Y] for point in points]
    zs = [point[POINT_Z] for point in points]
    height = max(zs) - min(zs)
    width = max(max(xs) - min(xs), max(ys) - min(ys))
    compact_small_object = (
        len(points) >= int(small_object_minimum_points)
        and width <= float(small_object_maximum_width_m)
    )
    return height >= float(minimum_height_m) or compact_small_object
