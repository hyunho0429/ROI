#!/usr/bin/env python3

import os
import sys
import unittest


TEST_DIR = os.path.dirname(__file__)
PACKAGE_SRC = os.path.abspath(os.path.join(TEST_DIR, "..", "src"))
REPOSITORY_ROOT = os.path.abspath(
    os.path.join(TEST_DIR, "..", "..", "..", "..")
)
for path in (PACKAGE_SRC, REPOSITORY_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

from camera_perception.highway_environment import (
    HighwayEnvironmentLatch,
    adjacent_left_lane_semantics,
    adjacent_left_lane_type,
    exclusive_highway_active,
)
from camera_perception.highway_vehicle import highway_vehicle_detected


class HighwayVehicleDetectionTest(unittest.TestCase):
    def test_only_unified_car_activates_vehicle_condition(self):
        self.assertTrue(highway_vehicle_detected({"car"}))
        for label in ("bus", "train", "truck", "motorcycle", "bicycle"):
            with self.subTest(label=label):
                self.assertFalse(highway_vehicle_detected({label}))

    def test_non_vehicle_classes_do_not_activate(self):
        self.assertFalse(highway_vehicle_detected({"person"}))

    def test_class_names_are_normalized(self):
        self.assertTrue(highway_vehicle_detected({" Car "}))


class HighwayEnvironmentLatchTest(unittest.TestCase):
    def test_once_active_remains_active(self):
        state = HighwayEnvironmentLatch(latch_once=True)

        self.assertFalse(state.update(False))
        self.assertTrue(state.update(True))
        self.assertTrue(state.update(False))

    def test_latch_can_be_disabled_for_previous_behavior(self):
        state = HighwayEnvironmentLatch(latch_once=False)

        self.assertTrue(state.update(True))
        self.assertFalse(state.update(False))


class AdjacentLeftLaneTest(unittest.TestCase):
    @staticmethod
    def lane_info(lane_type="white_dashed", y=1.75, status="FRESH", age=3):
        return {
            "lane_valid": True,
            "output_status": status,
            "left_lane": {
                "detected": True,
                "type": lane_type,
                "dashed": lane_type == "white_dashed",
                "age": age,
                "coef": [0.0, 0.0, y],
                "x_range_m": [5.0, 25.0],
            },
        }

    def test_nearest_adjacent_dashed_boundary_authorizes_merge(self):
        info = self.lane_info("white_dashed", y=1.75)
        self.assertEqual(adjacent_left_lane_type(info), "white_dashed")
        self.assertTrue(adjacent_left_lane_semantics(info)["dashed"])

    def test_nearest_adjacent_solid_boundary_blocks_dashed_signal(self):
        info = self.lane_info("white_solid", y=1.75)
        semantics = adjacent_left_lane_semantics(info)
        self.assertFalse(semantics["dashed"])
        self.assertTrue(semantics["solid"])

    def test_far_left_dashed_boundary_cannot_authorize_merge(self):
        info = self.lane_info("white_dashed", y=3.2)
        self.assertIsNone(adjacent_left_lane_type(info))

    def test_held_boundary_cannot_authorize_merge(self):
        info = self.lane_info("white_dashed", status="HELD")
        self.assertIsNone(adjacent_left_lane_type(info))

    def test_single_frame_boundary_cannot_authorize_merge(self):
        info = self.lane_info("white_dashed", age=1)
        self.assertIsNone(adjacent_left_lane_type(info))

    def test_right_dashed_does_not_override_left_solid(self):
        info = self.lane_info("white_solid", y=1.7)
        info["right_lane"] = {
            "detected": True,
            "type": "white_dashed",
            "dashed": True,
            "age": 5,
            "coef": [0.0, 0.0, -1.7],
            "x_range_m": [5.0, 25.0],
        }
        semantics = adjacent_left_lane_semantics(info)
        self.assertTrue(semantics["solid"])
        self.assertFalse(semantics["dashed"])


class SituationExclusionTest(unittest.TestCase):
    def test_intersection_overrides_highway(self):
        self.assertTrue(exclusive_highway_active(True, False))
        self.assertFalse(exclusive_highway_active(True, True))
        self.assertFalse(exclusive_highway_active(False, False))


if __name__ == "__main__":
    unittest.main()
