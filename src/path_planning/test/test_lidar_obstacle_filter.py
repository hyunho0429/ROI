#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from path_planning.lidar_obstacle_filter import (
    compact_single_ring_groups,
    confirm_cluster_centroids,
    filter_vertical_support,
)


def _point(x, y, z, ring):
    return (
        x,
        y,
        z,
        math.sqrt(x * x + y * y + z * z),
        1.0,
        ring,
        math.degrees(math.atan2(y, x)),
    )


class LidarObstacleFilterTest(unittest.TestCase):
    def test_rejects_single_ring_circular_arc(self):
        arc = [
            _point(
                8.0 * math.cos(math.radians(angle)),
                8.0 * math.sin(math.radians(angle)),
                -1.2,
                3,
            )
            for angle in range(-20, 21)
        ]

        filtered = filter_vertical_support(arc, 0.65, 0.05)

        self.assertEqual(filtered, [])

    def test_rejects_nearby_flat_returns_from_different_rings(self):
        flat_returns = [
            _point(8.0, 0.0, -1.20, 2),
            _point(8.2, 0.0, -1.19, 4),
        ]

        filtered = filter_vertical_support(flat_returns, 0.65, 0.05)

        self.assertEqual(filtered, [])

    def test_keeps_vertically_supported_obstacle_returns(self):
        obstacle = [
            _point(8.0, 0.0, -0.8, 2),
            _point(8.1, 0.1, -0.2, 4),
            _point(8.0, -0.1, 0.4, 6),
        ]

        filtered = filter_vertical_support(obstacle, 0.65, 0.05)

        self.assertEqual(filtered, obstacle)

    def test_rejects_different_rings_that_are_too_far_apart(self):
        separated_ground_rings = [
            _point(6.0, 0.0, -1.2, 2),
            _point(7.0, 0.0, -1.0, 4),
        ]

        filtered = filter_vertical_support(separated_ground_rings, 0.65, 0.05)

        self.assertEqual(filtered, [])

    def test_low_object_path_keeps_compact_single_ring_return(self):
        low_object = [
            _point(10.0, -0.15, -1.1, 3),
            _point(10.0, 0.0, -1.1, 3),
            _point(10.0, 0.15, -1.1, 3),
        ]

        groups = compact_single_ring_groups(low_object, 2, 1.2)

        self.assertEqual(groups, [low_object])

    def test_low_object_path_rejects_long_ground_arc(self):
        arc = [
            _point(
                8.0 * math.cos(math.radians(angle)),
                8.0 * math.sin(math.radians(angle)),
                -1.2,
                3,
            )
            for angle in range(-20, 21)
        ]

        groups = compact_single_ring_groups(arc, 2, 1.2)

        self.assertEqual(groups, [])

    def test_low_object_requires_two_nearby_scans(self):
        previous = [{"centroid_x_m": 10.0, "centroid_y_m": 0.0}]
        repeated = [{"centroid_x_m": 9.7, "centroid_y_m": 0.2}]
        unrelated = [{"centroid_x_m": 7.0, "centroid_y_m": 3.0}]

        self.assertEqual(
            confirm_cluster_centroids(repeated, previous, 1.0),
            repeated,
        )
        self.assertEqual(
            confirm_cluster_centroids(unrelated, previous, 1.0),
            [],
        )


if __name__ == "__main__":
    unittest.main()
