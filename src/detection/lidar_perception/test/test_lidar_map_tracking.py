#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from lidar_perception.lidar_map_tracking import (
    Pose2DHistory,
    oriented_bounding_box_2d,
    resolve_box_heading,
    transform_box_to_map,
)
from lidar_perception.lidar_kalman_hungarian import KalmanHungarianTracker


def _rotated_rectangle(center_x, center_y, length, width, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    points = []
    for longitudinal in (-0.5 * length, 0.5 * length):
        for lateral in (-0.5 * width, 0.5 * width):
            points.append(
                (
                    center_x + cosine * longitudinal - sine * lateral,
                    center_y + sine * longitudinal + cosine * lateral,
                    0.5,
                )
            )
    return points


class OrientedBoundingBoxTest(unittest.TestCase):
    def test_recovers_rotated_rectangle_geometry(self):
        box = oriented_bounding_box_2d(
            _rotated_rectangle(4.0, -2.0, 4.6, 1.9, math.radians(30.0)),
            cluster_id=9,
        )

        self.assertEqual(box["cluster_id"], 9)
        self.assertAlmostEqual(box["center_x_m"], 4.0, places=6)
        self.assertAlmostEqual(box["center_y_m"], -2.0, places=6)
        self.assertAlmostEqual(box["length_m"], 4.6, places=6)
        self.assertAlmostEqual(box["width_m"], 1.9, places=6)
        self.assertAlmostEqual(box["yaw_rad"], math.radians(30.0), places=6)

    def test_transforms_center_and_yaw_to_map(self):
        local = oriented_bounding_box_2d(
            _rotated_rectangle(2.0, 0.0, 4.0, 2.0, 0.0)
        )
        mapped = transform_box_to_map(
            local,
            ego_x_m=10.0,
            ego_y_m=20.0,
            ego_yaw_rad=0.5 * math.pi,
        )

        self.assertAlmostEqual(mapped["center_x_m"], 10.0, places=6)
        self.assertAlmostEqual(mapped["center_y_m"], 22.0, places=6)
        self.assertAlmostEqual(mapped["yaw_rad"], 0.5 * math.pi, places=6)

    def test_velocity_resolves_front_rear_ambiguity(self):
        resolved = resolve_box_heading(0.0, -3.0, 0.0, minimum_speed_mps=0.5)
        self.assertAlmostEqual(abs(resolved), math.pi, places=6)


class PoseHistoryTest(unittest.TestCase):
    def test_interpolates_position_and_yaw_across_wrap(self):
        history = Pose2DHistory(history_s=2.0)
        history.append(1.0, 0.0, 0.0, math.radians(179.0))
        history.append(2.0, 2.0, 4.0, math.radians(-179.0))

        x_m, y_m, yaw_rad = history.pose_at(1.5)
        self.assertAlmostEqual(x_m, 1.0)
        self.assertAlmostEqual(y_m, 2.0)
        self.assertAlmostEqual(abs(yaw_rad), math.pi, places=6)

    def test_rejects_pose_outside_maximum_age(self):
        history = Pose2DHistory()
        history.append(10.0, 1.0, 2.0, 0.0)
        self.assertIsNone(history.pose_at(10.5, maximum_age_s=0.25))


class MapTrackingTest(unittest.TestCase):
    def test_stationary_obstacle_has_zero_map_velocity_while_ego_moves(self):
        tracker = KalmanHungarianTracker(
            match_distance_m=2.0,
            min_hits=2,
            max_missed=2,
            process_accel_std_mps2=2.0,
            measurement_noise_m=0.05,
        )
        results = []
        for index, ego_x_m in enumerate((0.0, 0.5, 1.0, 1.5)):
            local_box = oriented_bounding_box_2d(
                _rotated_rectangle(10.0 - ego_x_m, 0.0, 4.0, 2.0, 0.0)
            )
            map_detection = transform_box_to_map(
                local_box,
                ego_x_m=ego_x_m,
                ego_y_m=0.0,
                ego_yaw_rad=0.0,
            )
            results = tracker.update([map_detection], 0.1 * index)

        self.assertTrue(results[0]["confirmed"])
        self.assertAlmostEqual(results[0]["center_x_m"], 10.0, places=6)
        self.assertAlmostEqual(results[0]["velocity_x_mps"], 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
