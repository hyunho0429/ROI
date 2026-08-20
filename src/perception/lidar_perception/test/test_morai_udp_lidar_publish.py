#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from lidar_perception.morai_udp_lidar import should_publish_cloud


class LidarCloudPublishTriggerTest(unittest.TestCase):
    def test_detects_wrap_when_fixed_angle_windows_are_skipped(self):
        self.assertTrue(
            should_publish_cloud(345.0, 15.0, 5, 0.01, 15, 0.05)
        )

    def test_ignores_small_reverse_azimuth_jitter(self):
        self.assertFalse(
            should_publish_cloud(120.0, 119.0, 5, 0.01, 15, 0.05)
        )

    def test_packet_limit_keeps_output_live(self):
        self.assertTrue(
            should_publish_cloud(120.0, 121.0, 15, 0.01, 15, 0.05)
        )

    def test_age_limit_keeps_output_live(self):
        self.assertTrue(
            should_publish_cloud(120.0, 121.0, 4, 0.05, 15, 0.05)
        )

    def test_never_publishes_empty_buffer(self):
        self.assertFalse(
            should_publish_cloud(345.0, 15.0, 0, 1.0, 15, 0.05)
        )


if __name__ == "__main__":
    unittest.main()
