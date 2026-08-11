#!/usr/bin/env python3

import os
import sys
import unittest

import numpy as np


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from lidar_perception.lidar_bounding_box import (
    axis_aligned_bounding_box,
    bounding_boxes,
)
from lidar_perception.lidar_euclidean_clustering import (
    euclidean_cluster_indices,
    select_roi,
)
from lidar_perception.lidar_kalman_hungarian import (
    KalmanHungarianTracker,
    hungarian_assignment,
)


def _detection(x_m, y_m, cluster_id=0):
    return {
        "cluster_id": cluster_id,
        "point_count": 8,
        "center_x_m": float(x_m),
        "center_y_m": float(y_m),
        "center_z_m": 0.5,
        "size_x_m": 4.0,
        "size_y_m": 1.8,
        "size_z_m": 1.5,
        "min_x_m": float(x_m) - 2.0,
        "max_x_m": float(x_m) + 2.0,
        "min_y_m": float(y_m) - 0.9,
        "max_y_m": float(y_m) + 0.9,
        "min_z_m": -0.25,
        "max_z_m": 1.25,
        "distance_m": float(np.hypot(x_m, y_m)),
        "bearing_deg": float(np.degrees(np.arctan2(y_m, x_m))),
    }


class EuclideanClusteringTest(unittest.TestCase):
    def test_separates_connected_groups_and_rejects_noise(self):
        points = [
            (0.0, 0.0, 0.0),
            (0.2, 0.0, 0.2),
            (0.4, 0.1, 0.4),
            (5.0, 0.0, 0.0),
            (5.2, 0.1, 0.3),
            (5.4, 0.0, 0.6),
            (20.0, 20.0, 0.0),
        ]

        clusters = euclidean_cluster_indices(
            points,
            tolerance_m=0.5,
            min_points=3,
            min_height_m=0.2,
        )

        self.assertEqual(len(clusters), 2)
        self.assertEqual(set(clusters[0]), {0, 1, 2})
        self.assertEqual(set(clusters[1]), {3, 4, 5})

    def test_roi_drops_nonfinite_and_outside_points(self):
        selected = select_roi(
            [(1.0, 0.0, 0.0), (3.0, 0.0, 0.0), (1.0, 2.0, 0.0), (np.nan, 0.0, 0.0)],
            x_min_m=0.0,
            x_max_m=2.0,
            y_abs_m=1.0,
            z_min_m=-1.0,
            z_max_m=1.0,
        )
        self.assertEqual(selected, [(1.0, 0.0, 0.0)])


class BoundingBoxTest(unittest.TestCase):
    def test_axis_aligned_box_geometry(self):
        box = axis_aligned_bounding_box(
            [(1.0, -2.0, -0.5), (5.0, 2.0, 1.5)],
            cluster_id=7,
        )

        self.assertEqual(box["cluster_id"], 7)
        self.assertAlmostEqual(box["center_x_m"], 3.0)
        self.assertAlmostEqual(box["center_y_m"], 0.0)
        self.assertAlmostEqual(box["center_z_m"], 0.5)
        self.assertAlmostEqual(box["size_x_m"], 4.0)
        self.assertAlmostEqual(box["size_y_m"], 4.0)
        self.assertAlmostEqual(box["size_z_m"], 2.0)

    def test_builds_one_box_per_cluster(self):
        points = [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (10.0, 0.0, 0.0)]
        boxes = bounding_boxes(points, [[0, 1], [2]])
        self.assertEqual(len(boxes), 2)
        self.assertEqual(boxes[0]["point_count"], 2)
        self.assertEqual(boxes[1]["point_count"], 1)


class KalmanHungarianTest(unittest.TestCase):
    def test_hungarian_finds_global_not_greedy_minimum(self):
        assignments = hungarian_assignment([[1.0, 2.0], [1.1, 100.0]])
        self.assertEqual(set(assignments), {(0, 1), (1, 0)})

    def test_tracker_keeps_ids_and_estimates_velocity(self):
        tracker = KalmanHungarianTracker(
            match_distance_m=2.0,
            min_hits=2,
            max_missed=2,
            process_accel_std_mps2=2.0,
            measurement_noise_m=0.1,
        )
        first = tracker.update([_detection(10.0, 0.0)], 0.0)
        second = tracker.update([_detection(10.1, 0.0)], 0.1)
        third = tracker.update([_detection(10.2, 0.0)], 0.2)

        self.assertEqual(first[0]["track_id"], second[0]["track_id"])
        self.assertEqual(second[0]["track_id"], third[0]["track_id"])
        self.assertTrue(second[0]["confirmed"])
        self.assertGreater(third[0]["velocity_x_mps"], 0.0)

    def test_tracker_predicts_then_deletes_after_miss_limit(self):
        tracker = KalmanHungarianTracker(max_missed=1, min_hits=1)
        tracker.update([_detection(5.0, 0.0)], 0.0)

        predicted = tracker.update([], 0.1)
        deleted = tracker.update([], 0.2)

        self.assertEqual(len(predicted), 1)
        self.assertEqual(predicted[0]["misses"], 1)
        self.assertEqual(deleted, [])


if __name__ == "__main__":
    unittest.main()
