#!/usr/bin/env python3
"""Dependency-free Euclidean clustering for small MORAI LiDAR point sets.

The spatial hash keeps neighbour lookup close to O(N) for the sparse VLP-16
clouds used by this project.  Points may contain extra fields; only xyz at
indices 0, 1 and 2 are used for the distance test.
"""

import math
from collections import defaultdict


def euclidean_cluster_indices(
    points,
    tolerance_m=0.8,
    min_points=3,
    max_points=0,
    min_height_m=0.0,
    max_clusters=0,
):
    """Return point-index groups connected by the Euclidean tolerance.

    ``max_points`` is the maximum accepted size of one cluster.  A value of
    zero disables that limit.  ``max_clusters`` keeps the nearest clusters
    after extraction; zero keeps every cluster.
    """

    tolerance_m = float(tolerance_m)
    min_points = int(min_points)
    max_points = int(max_points)
    min_height_m = float(min_height_m)
    max_clusters = int(max_clusters)

    if tolerance_m <= 0.0:
        raise ValueError("tolerance_m must be positive")
    if min_points < 1:
        raise ValueError("min_points must be at least 1")
    if max_points < 0:
        raise ValueError("max_points cannot be negative")
    if min_height_m < 0.0:
        raise ValueError("min_height_m cannot be negative")
    if max_clusters < 0:
        raise ValueError("max_clusters cannot be negative")

    point_list = list(points)
    if not point_list:
        return []

    cell_size = tolerance_m
    tolerance_sq = tolerance_m * tolerance_m
    grid = defaultdict(list)
    point_cells = []

    for index, point in enumerate(point_list):
        cell = (
            int(math.floor(float(point[0]) / cell_size)),
            int(math.floor(float(point[1]) / cell_size)),
            int(math.floor(float(point[2]) / cell_size)),
        )
        point_cells.append(cell)
        grid[cell].append(index)

    visited = [False] * len(point_list)
    clusters = []

    for seed_index in range(len(point_list)):
        if visited[seed_index]:
            continue

        visited[seed_index] = True
        queue = [seed_index]
        member_indices = []

        while queue:
            current_index = queue.pop()
            member_indices.append(current_index)
            current = point_list[current_index]
            cell_x, cell_y, cell_z = point_cells[current_index]

            for offset_x in (-1, 0, 1):
                for offset_y in (-1, 0, 1):
                    for offset_z in (-1, 0, 1):
                        neighbour_cell = (
                            cell_x + offset_x,
                            cell_y + offset_y,
                            cell_z + offset_z,
                        )
                        for neighbour_index in grid.get(neighbour_cell, ()):
                            if visited[neighbour_index]:
                                continue
                            neighbour = point_list[neighbour_index]
                            distance_sq = (
                                (float(current[0]) - float(neighbour[0])) ** 2
                                + (float(current[1]) - float(neighbour[1])) ** 2
                                + (float(current[2]) - float(neighbour[2])) ** 2
                            )
                            if distance_sq <= tolerance_sq:
                                visited[neighbour_index] = True
                                queue.append(neighbour_index)

        if len(member_indices) < min_points:
            continue
        if max_points and len(member_indices) > max_points:
            continue

        z_values = [float(point_list[index][2]) for index in member_indices]
        if max(z_values) - min(z_values) < min_height_m:
            continue

        centroid_x = sum(
            float(point_list[index][0]) for index in member_indices
        ) / len(member_indices)
        centroid_y = sum(
            float(point_list[index][1]) for index in member_indices
        ) / len(member_indices)
        clusters.append(
            (
                math.hypot(centroid_x, centroid_y),
                member_indices,
            )
        )

    clusters.sort(key=lambda item: item[0])
    if max_clusters:
        clusters = clusters[:max_clusters]
    return [member_indices for _distance, member_indices in clusters]


def select_roi(points, x_min_m, x_max_m, y_abs_m, z_min_m, z_max_m):
    """Return finite xyz points inside the requested ego-local ROI."""

    selected = []
    for point in points:
        x_m = float(point[0])
        y_m = float(point[1])
        z_m = float(point[2])
        if not all(math.isfinite(value) for value in (x_m, y_m, z_m)):
            continue
        if not float(x_min_m) <= x_m <= float(x_max_m):
            continue
        if abs(y_m) > float(y_abs_m):
            continue
        if not float(z_min_m) <= z_m <= float(z_max_m):
            continue
        selected.append((x_m, y_m, z_m))
    return selected
