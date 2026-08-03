"""Geometry filters that reject unsupported LiDAR scan-line arcs."""

import math
from collections import defaultdict


POINT_X = 0
POINT_Y = 1
POINT_Z = 2
POINT_RING = 5


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
