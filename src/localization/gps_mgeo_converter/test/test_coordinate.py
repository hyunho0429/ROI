#!/usr/bin/env python3
import math
import unittest

from gps_mgeo_converter.coordinate import gps_to_mgeo, wgs84_to_utm52, wrap_angle


class CoordinateTest(unittest.TestCase):
    def test_utm_is_meter_coordinate(self):
        easting, northing = wgs84_to_utm52(37.0, 129.0)
        self.assertTrue(300000.0 < easting < 700000.0)
        self.assertTrue(4_000_000.0 < northing < 5_000_000.0)

    def test_local_origin_subtraction(self):
        east, north = wgs84_to_utm52(37.0, 129.0)
        x, y, z = gps_to_mgeo(37.0, 129.0, 30.0, [east, north, 0.0])
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, 30.0, places=6)

    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(3.0 * math.pi), math.pi, places=6)


if __name__ == "__main__":
    unittest.main()
