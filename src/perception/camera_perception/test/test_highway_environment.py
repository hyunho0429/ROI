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

from camera_perception.highway_environment import HighwayEnvironmentLatch
from Sensor.highway_vehicle import highway_vehicle_detected


class HighwayVehicleDetectionTest(unittest.TestCase):
    def test_car_bus_and_truck_each_activate_vehicle_condition(self):
        for label in ("car", "bus", "truck"):
            with self.subTest(label=label):
                self.assertTrue(highway_vehicle_detected({label}))

    def test_non_vehicle_classes_do_not_activate(self):
        self.assertFalse(highway_vehicle_detected({"person", "bicycle"}))

    def test_class_names_are_normalized(self):
        self.assertTrue(highway_vehicle_detected({" Truck "}))


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


if __name__ == "__main__":
    unittest.main()
