#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from camera_perception.stopline import stopline_requires_stop


class StoplineControlTest(unittest.TestCase):
    def test_stops_at_or_inside_one_meter(self):
        self.assertTrue(stopline_requires_stop(1.0))
        self.assertTrue(stopline_requires_stop(0.25))

    def test_does_not_stop_for_far_or_missing_line(self):
        self.assertFalse(stopline_requires_stop(1.01))
        self.assertFalse(stopline_requires_stop(None))
        self.assertFalse(stopline_requires_stop(float("nan")))

    def test_rejects_line_behind_ego(self):
        self.assertFalse(stopline_requires_stop(-0.1))

    def test_supports_configurable_distance(self):
        self.assertTrue(stopline_requires_stop(1.5, 2.0))
        self.assertFalse(stopline_requires_stop(2.1, 2.0))

    def test_rejects_invalid_threshold(self):
        with self.assertRaises(ValueError):
            stopline_requires_stop(0.5, 0.0)


if __name__ == "__main__":
    unittest.main()
