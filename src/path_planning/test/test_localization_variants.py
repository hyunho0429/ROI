#!/usr/bin/env python3

import math
import os
import sys
import unittest

import numpy as np


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from path_planning.inertial_math import (
    apply_body_rotation,
    quaternion_from_rotation_vector,
    quaternion_yaw,
)
from path_planning.localization_dead_reckoning import SpeedAidedDeadReckoning
from path_planning.localization_ins import InsErrorStateEkf


IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)
STATIONARY_SPECIFIC_FORCE = (0.0, 0.0, 9.80665)


class InertialMathTest(unittest.TestCase):
    def test_body_rotation_updates_yaw(self):
        quaternion = apply_body_rotation(
            IDENTITY_QUATERNION,
            (0.0, 0.0, math.pi / 2.0),
        )
        self.assertAlmostEqual(quaternion_yaw(quaternion), math.pi / 2.0)

    def test_rotation_vector_quaternion_is_normalized(self):
        quaternion = quaternion_from_rotation_vector((0.1, -0.2, 0.3))
        self.assertAlmostEqual(float(np.linalg.norm(quaternion)), 1.0)


class InsErrorStateEkfTest(unittest.TestCase):
    def test_stationary_specific_force_does_not_fall_through_map(self):
        localizer = InsErrorStateEkf()
        localizer.add_imu(
            0.0,
            IDENTITY_QUATERNION,
            (0.0, 0.0, 0.0),
            STATIONARY_SPECIFIC_FORCE,
        )
        localizer.add_gps(0.0, 0.0, 0.0, 0.0, 0.0, 90.0)
        localizer.add_vehicle_speed(0.0, 0.0)
        state = localizer.state_at(5.0)
        self.assertAlmostEqual(state.x_m, 0.0, places=6)
        self.assertAlmostEqual(state.y_m, 0.0, places=6)
        self.assertAlmostEqual(state.z_m, 0.0, places=6)

    def test_vehicle_speed_aids_ins_during_gps_outage(self):
        localizer = InsErrorStateEkf()
        localizer.add_imu(
            0.0,
            IDENTITY_QUATERNION,
            (0.0, 0.0, 0.0),
            STATIONARY_SPECIFIC_FORCE,
        )
        localizer.add_gps(0.0, 0.0, 0.0, 0.0, 10.0, 90.0)
        localizer.add_vehicle_speed(0.0, 10.0)
        state = localizer.state_at(1.0)
        self.assertGreater(state.x_m, 9.5)
        self.assertAlmostEqual(state.y_m, 0.0, places=4)

    def test_stationary_speed_calibrates_imu_bias(self):
        localizer = InsErrorStateEkf()
        localizer.add_imu(
            0.0,
            IDENTITY_QUATERNION,
            (0.0, 0.0, 0.01),
            (0.0, 0.0, 9.90665),
        )
        localizer.add_gps(0.0, 0.0, 0.0, 0.0, 0.0, 90.0)
        localizer.add_vehicle_speed(0.0, 0.0)
        for index in range(1, 101):
            localizer.add_imu(
                index * 0.01,
                IDENTITY_QUATERNION,
                (0.0, 0.0, 0.01),
                (0.0, 0.0, 9.90665),
            )
        self.assertGreater(localizer.accel_bias[2], 0.05)
        self.assertGreater(localizer.gyro_bias[2], 0.005)


class SpeedAidedDeadReckoningTest(unittest.TestCase):
    def test_integrates_competition_speed_without_gps(self):
        localizer = SpeedAidedDeadReckoning()
        localizer.add_imu(
            0.0,
            IDENTITY_QUATERNION,
            (0.0, 0.0, 0.0),
            STATIONARY_SPECIFIC_FORCE,
        )
        localizer.add_gps(0.0, 0.0, 0.0, 0.0)
        localizer.add_vehicle_speed(0.0, 10.0)
        state = localizer.state_at(2.0)
        self.assertAlmostEqual(state.x_m, 20.0, places=5)
        self.assertAlmostEqual(state.y_m, 0.0, places=5)
        self.assertAlmostEqual(state.z_m, 0.0, places=5)

    def test_gps_reacquisition_corrects_accumulated_position(self):
        localizer = SpeedAidedDeadReckoning(position_drift_sigma_mps=2.0)
        localizer.add_imu(
            0.0,
            IDENTITY_QUATERNION,
            (0.0, 0.0, 0.0),
            STATIONARY_SPECIFIC_FORCE,
        )
        localizer.add_gps(0.0, 0.0, 0.0, 0.0)
        localizer.add_vehicle_speed(0.0, 5.0)
        predicted = localizer.state_at(2.0).x_m
        self.assertAlmostEqual(predicted, 10.0, places=5)
        self.assertTrue(localizer.add_gps(2.0, 8.0, 0.0, 0.0))
        corrected = localizer.state_at(2.0).x_m
        self.assertLess(corrected, predicted)
        self.assertGreater(corrected, 8.0)


if __name__ == "__main__":
    unittest.main()
