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


class SituationExclusionTest(unittest.TestCase):
    def test_intersection_overrides_highway(self):
        self.assertTrue(exclusive_highway_active(True, False))
        self.assertFalse(exclusive_highway_active(True, True))
        self.assertFalse(exclusive_highway_active(False, False))


if __name__ == "__main__":
    unittest.main()
