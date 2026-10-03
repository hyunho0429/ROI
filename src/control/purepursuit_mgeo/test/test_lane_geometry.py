import math
import unittest

from purepursuit_mgeo.lane_geometry import pose_at, reproject


class LaneGeometryTest(unittest.TestCase):
    def test_lane_observed_before_yaw_is_reprojected_into_current_ego_frame(self):
        observed = (0.0, 0.0, 0.0)
        current = (1.0, 0.0, math.radians(10.0))
        x, y = reproject([(10.0, 1.75)], observed, current)[0]
        self.assertAlmostEqual(x, 9.0*math.cos(current[2])+1.75*math.sin(current[2]))
        self.assertAlmostEqual(y, -9.0*math.sin(current[2])+1.75*math.cos(current[2]))
        self.assertLess(y, 1.75)

    def test_pose_interpolation_and_stale_rejection(self):
        history = [(10.0, (0.0, 0.0, 0.0)), (10.1, (1.0, 0.0, 0.2))]
        self.assertAlmostEqual(pose_at(history, 10.05)[0], 0.5)
        self.assertIsNone(pose_at(history, 9.7))
        self.assertIsNone(pose_at(history, 10.4))


if __name__ == "__main__":
    unittest.main()
