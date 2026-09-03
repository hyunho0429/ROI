#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from path_planning.lidar_merge_gap import (
    MergeGapTracker,
    assess_merge_gaps,
    format_merge_gap_status,
)


def _cluster(min_x, max_x, center_y):
    return {
        "min_x_m": float(min_x),
        "max_x_m": float(max_x),
        "centroid_x_m": 0.5 * (float(min_x) + float(max_x)),
        "centroid_y_m": float(center_y),
    }


def _assess(clusters, lane_width=3.5):
    return assess_merge_gaps(
        clusters=clusters,
        vehicle_length_m=4.635,
        vehicle_width_m=1.892,
        vehicle_height_m=2.434,
        lane_width_m=lane_width,
        lane_lateral_allowance_m=0.4,
        longitudinal_margin_m=1.0,
        lateral_margin_m=0.2,
        detection_range_m=40.0,
    )


class LidarMergeGapTest(unittest.TestCase):
    def test_uses_xyz_vehicle_size_and_range_limits_for_open_lane(self):
        left = _assess([])["left"]

        self.assertTrue(left["available"])
        self.assertAlmostEqual(left["required_gap_m"], 6.635)
        self.assertAlmostEqual(left["visible_gap_m"], 80.0)
        self.assertAlmostEqual(left["front_clearance_m"], 37.6825)
        self.assertAlmostEqual(left["rear_clearance_m"], 37.6825)
        self.assertAlmostEqual(left["vehicle_height_m"], 2.434)
        self.assertEqual(left["front_boundary_source"], "range_limit")
        self.assertEqual(left["rear_boundary_source"], "range_limit")

    def test_accepts_obstacle_bounded_gap_that_fits_ego(self):
        left = _assess(
            [
                _cluster(8.0, 12.0, 3.5),
                _cluster(-12.0, -8.0, 3.5),
            ]
        )["left"]

        self.assertTrue(left["available"])
        self.assertAlmostEqual(left["visible_gap_m"], 16.0)
        self.assertAlmostEqual(left["front_clearance_m"], 5.6825)
        self.assertAlmostEqual(left["rear_clearance_m"], 5.6825)
        self.assertEqual(left["front_boundary_source"], "obstacle")
        self.assertEqual(left["rear_boundary_source"], "obstacle")

    def test_rejects_obstacle_alongside(self):
        right = _assess([_cluster(-1.0, 1.0, -3.5)])["right"]

        self.assertFalse(right["available"])
        self.assertEqual(right["reason"], "obstacle_alongside")
        self.assertEqual(right["visible_gap_m"], 0.0)

    def test_rejects_front_margin_short(self):
        left = _assess([_cluster(3.0, 7.0, 3.5)])["left"]

        self.assertFalse(left["available"])
        self.assertEqual(left["reason"], "front_margin_short")
        self.assertLess(left["front_clearance_m"], 1.0)

    def test_rejects_lane_that_is_too_narrow(self):
        left = _assess([], lane_width=2.0)["left"]

        self.assertFalse(left["available"])
        self.assertEqual(left["reason"], "lane_width_short")
        self.assertLess(left["lateral_clearance_m"], 0.2)

    def test_tracks_confirmation_and_formats_status(self):
        tracker = MergeGapTracker(confirmation_scans=3)
        clear = _assess([])

        first, became_available, became_unavailable = tracker.update(clear)
        self.assertEqual(became_available, [])
        self.assertEqual(became_unavailable, [])
        self.assertIn("LEFT=CHECKING(1/3)", format_merge_gap_status(first["left"]))

        tracker.update(clear)
        third, became_available, became_unavailable = tracker.update(clear)
        self.assertEqual(became_available, ["left", "right"])
        self.assertEqual(became_unavailable, [])
        self.assertIn("LEFT=AVAILABLE", format_merge_gap_status(third["left"]))

        blocked = _assess([_cluster(-1.0, 1.0, 3.5)])
        fourth, became_available, became_unavailable = tracker.update(blocked)
        self.assertEqual(became_available, [])
        self.assertEqual(became_unavailable, ["left"])
        self.assertIn("LEFT=BLOCKED", format_merge_gap_status(fourth["left"]))


if __name__ == "__main__":
    unittest.main()
