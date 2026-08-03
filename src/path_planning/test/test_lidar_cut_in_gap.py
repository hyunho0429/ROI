#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from path_planning.lidar_cut_in_gap import (
    GapAvailabilityTracker,
    assess_adjacent_lane_gaps,
)


def _cluster(min_x, max_x, center_y):
    return {
        "min_x_m": float(min_x),
        "max_x_m": float(max_x),
        "centroid_x_m": 0.5 * (float(min_x) + float(max_x)),
        "centroid_y_m": float(center_y),
    }


def _assess(clusters):
    return assess_adjacent_lane_gaps(
        clusters=clusters,
        ego_length_m=4.635,
        ego_width_m=1.892,
        lane_width_m=3.5,
        lateral_allowance_m=0.5,
        front_clearance_m=15.0,
        rear_clearance_m=15.0,
    )


class LidarCutInGapTest(unittest.TestCase):
    def test_accepts_gap_around_current_ego_position(self):
        assessments = _assess(
            [
                _cluster(20.0, 24.0, 3.5),
                _cluster(-24.0, -20.0, 3.5),
            ]
        )

        left = assessments["left"]
        self.assertTrue(left["available"])
        self.assertEqual(left["reason"], "available")
        self.assertAlmostEqual(left["free_gap_m"], 40.0)
        self.assertAlmostEqual(left["required_gap_m"], 34.635)

    def test_rejects_rear_vehicle_inside_clearance(self):
        assessments = _assess(
            [
                _cluster(20.0, 24.0, -3.5),
                _cluster(-14.0, -10.0, -3.5),
            ]
        )

        right = assessments["right"]
        self.assertFalse(right["available"])
        self.assertEqual(right["reason"], "rear_clearance_short")
        self.assertLess(right["rear_clearance_m"], 15.0)

    def test_rejects_vehicle_alongside_ego(self):
        assessments = _assess(
            [
                _cluster(20.0, 24.0, 3.5),
                _cluster(-24.0, -20.0, 3.5),
                _cluster(-1.0, 1.0, 3.5),
            ]
        )

        left = assessments["left"]
        self.assertFalse(left["available"])
        self.assertEqual(left["reason"], "vehicle_alongside")

    def test_requires_both_front_and_rear_vehicle(self):
        assessments = _assess([_cluster(20.0, 24.0, 3.5)])

        left = assessments["left"]
        self.assertFalse(left["available"])
        self.assertEqual(left["reason"], "rear_vehicle_not_detected")

    def test_confirms_and_clears_availability_transitions(self):
        tracker = GapAvailabilityTracker(confirmation_scans=2)
        available = _assess(
            [
                _cluster(20.0, 24.0, 3.5),
                _cluster(-24.0, -20.0, 3.5),
            ]
        )

        first, secured, lost = tracker.update(available)
        self.assertFalse(first["left"]["confirmed_available"])
        self.assertEqual(secured, [])
        self.assertEqual(lost, [])

        second, secured, lost = tracker.update(available)
        self.assertTrue(second["left"]["confirmed_available"])
        self.assertEqual(secured, ["left"])
        self.assertEqual(lost, [])

        unavailable = _assess([])
        third, secured, lost = tracker.update(unavailable)
        self.assertFalse(third["left"]["confirmed_available"])
        self.assertEqual(secured, [])
        self.assertEqual(lost, ["left"])


if __name__ == "__main__":
    unittest.main()
