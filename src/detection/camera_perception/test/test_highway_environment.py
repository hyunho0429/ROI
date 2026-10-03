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
    AdjacentDashedHold,
    HighwayEnvironmentLatch,
    ConsecutiveLanePattern,
    adjacent_left_lane_semantics,
    adjacent_left_lane_type,
    exclusive_highway_active,
    multilane_highway_pattern,
)
from camera_perception.highway_vehicle import highway_vehicle_detected


class HighwayVehicleDetectionTest(unittest.TestCase):
    def test_car_bus_and_truck_activate_vehicle_condition(self):
        for label in ("car", "bus", "truck"):
            with self.subTest(label=label):
                self.assertTrue(highway_vehicle_detected({label}))

        for label in ("train", "motorcycle", "bicycle"):
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


class AdjacentDashedHoldTest(unittest.TestCase):
    def test_false_frame_does_not_cancel_recent_dashed_observation(self):
        state = AdjacentDashedHold(2.0)
        state.observe_dashed(True, 10.0)
        state.observe_dashed(False, 10.1)
        self.assertTrue(state.active(11.9))
        self.assertFalse(state.active(12.1))

    def test_positive_solid_cancels_dashed_hold_immediately(self):
        state = AdjacentDashedHold(2.0)
        state.observe_dashed(True, 10.0)
        state.observe_solid(True)
        self.assertFalse(state.active(10.1))


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


class MultilanePatternTest(unittest.TestCase):
    @staticmethod
    def info(outer_type="white_solid"):
        info = AdjacentLeftLaneTest.lane_info()
        info["lane_width_m"] = 3.5
        info["left_outer_lane"] = {
            "detected": True, "type": outer_type, "age": 3,
            "coef": [0.0, 0.0, 5.25], "x_range_m": [5.0, 25.0],
        }
        return info

    def test_distant_solid_is_not_the_requested_close_pair(self):
        self.assertIsNone(multilane_highway_pattern(self.info()))

    def test_close_dashed_and_solid_pair_is_highway_evidence(self):
        info = self.info()
        info["left_outer_lane"]["coef"] = [0, 0, 2.05]
        self.assertEqual(multilane_highway_pattern(info), "paired_dashed_solid")
        info["left_lane"]["type"] = "white_solid"
        info["left_outer_lane"]["type"] = "white_dashed"
        self.assertEqual(multilane_highway_pattern(info), "paired_dashed_solid")

    def test_double_dashed_is_distinct_weaker_pattern(self):
        self.assertEqual(multilane_highway_pattern(self.info("white_dashed")), "double_dashed")

    def test_nearest_solid_and_bad_geometry_are_rejected(self):
        info = self.info()
        info["left_lane"]["type"] = "white_solid"
        self.assertIsNone(multilane_highway_pattern(info))
        info = self.info()
        info["left_outer_lane"]["coef"] = [0, 0, 8.0]
        self.assertIsNone(multilane_highway_pattern(info))
        info = self.info()
        info["output_status"] = "HELD"
        self.assertIsNone(multilane_highway_pattern(info))

    def test_pattern_requires_distinct_consecutive_frames(self):
        state = ConsecutiveLanePattern(3)
        state.observe(1, "paired_dashed_solid")
        state.observe(1, "paired_dashed_solid")
        self.assertFalse(state.ready("paired_dashed_solid"))
        state.observe(2, "paired_dashed_solid")
        state.observe(3, "paired_dashed_solid")
        self.assertTrue(state.ready("paired_dashed_solid"))
        state.observe(4, None)
        self.assertFalse(state.ready("paired_dashed_solid"))


if __name__ == "__main__":
    unittest.main()
