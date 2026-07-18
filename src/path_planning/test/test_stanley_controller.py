#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from path_planning.localization import PlanarGpsImuEkf
from path_planning.stanley_controller import PathPoint, StanleyController


class StanleyControllerTest(unittest.TestCase):
    def test_vehicle_left_of_eastbound_path_steers_right(self):
        controller = StanleyController(
            [PathPoint(0.0, 0.0), PathPoint(20.0, 0.0)], gain=1.0
        )
        result = controller.compute(5.0, 2.0, 0.0, 0.0, 5.0)
        self.assertGreater(result.cross_track_error_m, 0.0)
        self.assertLess(result.steering_rad, 0.0)

    def test_heading_error_steers_toward_path_heading(self):
        controller = StanleyController([PathPoint(0.0, 0.0), PathPoint(20.0, 0.0)])
        result = controller.compute(5.0, 0.0, 0.0, math.radians(10.0), 5.0)
        self.assertLess(result.heading_error_rad, 0.0)
        self.assertLess(result.steering_rad, 0.0)

    def test_planar_filter_requires_both_gps_and_imu(self):
        ekf = PlanarGpsImuEkf()
        ekf.add_gps(1.0, 10.0, 20.0, speed_mps=2.0, course_deg=90.0)
        self.assertIsNone(ekf.state_at(1.01))
        ekf.add_imu(1.02, 0.0, 0.0)
        state = ekf.state_at(1.03)
        self.assertIsNotNone(state)
        self.assertAlmostEqual(state.yaw_rad, 0.0, places=4)
        self.assertGreater(state.speed_mps, 0.0)

    def test_planar_filter_rejects_large_gps_jump(self):
        ekf = PlanarGpsImuEkf(gps_outlier_threshold_m=10.0)
        self.assertTrue(ekf.add_gps(1.0, 0.0, 0.0))
        self.assertFalse(ekf.add_gps(1.1, 1000.0, 1000.0))


if __name__ == "__main__":
    unittest.main()
