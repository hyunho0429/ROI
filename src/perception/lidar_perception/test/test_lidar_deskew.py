#!/usr/bin/env python3

import math
import os
import struct
import sys
import unittest
from collections import deque


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from lidar_perception.lidar_deskew import deskew_scan, pose_at
from lidar_perception.lidar_direct_localization import DirectGpsImuPoseEstimator
from lidar_perception.morai_udp_localization_pose import (
    LocalizationPose,
    LocalizationPosePacketError,
    decode_localization_pose,
    encode_localization_pose,
)


def _pose(timestamp, x=0.0, y=0.0, z=0.0, yaw=0.0, speed=0.0, yaw_rate=0.0):
    return LocalizationPose(timestamp, x, y, z, yaw, speed, yaw_rate)


def _point(x, y, z=0.0):
    return (x, y, z, math.sqrt(x * x + y * y + z * z), 5.0, 3.0, 0.0)


def _imu_packet(yaw_rad, yaw_rate_radps=0.0):
    packet = bytearray(107)
    packet[:9] = b"#IMUData$"
    struct.pack_into("<I", packet, 9, 80)
    struct.pack_into(
        "<10d",
        packet,
        25,
        math.cos(yaw_rad / 2.0),
        0.0,
        0.0,
        math.sin(yaw_rad / 2.0),
        0.0,
        0.0,
        yaw_rate_radps,
        0.0,
        0.0,
        9.81,
    )
    packet[-2:] = b"\r\n"
    return bytes(packet)


class LocalizationPoseUdpTest(unittest.TestCase):
    def test_round_trip(self):
        expected = _pose(123.5, 1.0, -2.0, 0.4, 0.7, 8.0, -0.2)

        actual = decode_localization_pose(encode_localization_pose(expected))

        self.assertEqual(actual, expected)

    def test_rejects_invalid_packet(self):
        with self.assertRaises(LocalizationPosePacketError):
            decode_localization_pose(b"not-a-pose")


class DirectGpsImuPoseEstimatorTest(unittest.TestCase):
    def test_builds_pose_without_stanley_pose_udp(self):
        estimator = DirectGpsImuPoseEstimator()
        gps_packet = (
            b"$GPRMC,123519,A,3723.2475,N,12658.3416,E,10.0,90.0,230394,,,A\r\n"
            b"$GPGGA,123519,3723.2475,N,12658.3416,E,1,08,0.9,42.5,M,0.0,M,,\r\n"
        )

        self.assertIsNone(estimator.add_gps_packet(gps_packet, 10.0))
        pose = estimator.add_imu_packet(
            _imu_packet(math.radians(45.0), 0.2),
            10.01,
        )

        self.assertIsNotNone(pose)
        self.assertGreater(pose.x_m, 0.0)
        self.assertAlmostEqual(pose.y_m, 0.0)
        self.assertAlmostEqual(pose.yaw_rad, math.radians(45.0))
        self.assertAlmostEqual(pose.yaw_rate_radps, 0.2)
        self.assertGreater(pose.speed_mps, 0.0)


class LidarDeskewTest(unittest.TestCase):
    def test_interpolates_yaw_across_wrap(self):
        samples = deque(
            (
                _pose(10.0, yaw=math.radians(179.0)),
                _pose(12.0, yaw=math.radians(-179.0)),
            )
        )

        interpolated = pose_at(samples, 11.0, 0.1)

        self.assertAlmostEqual(abs(interpolated.yaw_rad), math.pi, places=7)

    def test_extrapolates_with_speed_and_yaw_rate(self):
        samples = deque((_pose(10.0, speed=4.0, yaw_rate=1.0),))

        extrapolated = pose_at(samples, 10.1, 0.2)

        self.assertAlmostEqual(extrapolated.x_m, 0.4 * math.cos(0.05))
        self.assertAlmostEqual(extrapolated.y_m, 0.4 * math.sin(0.05))
        self.assertAlmostEqual(extrapolated.yaw_rad, 0.1)
        self.assertIsNone(pose_at(samples, 10.3, 0.2))

    def test_compensates_forward_translation(self):
        point = _point(10.0, 0.0)
        acquisition_pose = _pose(1.0, x=0.0)
        reference_pose = _pose(2.0, x=1.0)

        points, applied = deskew_scan(
            [([point], acquisition_pose)],
            reference_pose,
        )

        self.assertTrue(applied)
        self.assertAlmostEqual(points[0][0], 9.0)
        self.assertAlmostEqual(points[0][1], 0.0)
        self.assertAlmostEqual(points[0][3], 9.0)

    def test_compensates_turning_vehicle(self):
        point = _point(10.0, 0.0)
        acquisition_pose = _pose(1.0, yaw=0.0)
        reference_pose = _pose(2.0, yaw=math.pi / 2.0)

        points, applied = deskew_scan(
            [([point], acquisition_pose)],
            reference_pose,
        )

        self.assertTrue(applied)
        self.assertAlmostEqual(points[0][0], 0.0, places=7)
        self.assertAlmostEqual(points[0][1], -10.0, places=7)
        self.assertAlmostEqual(points[0][6], -90.0, places=7)

    def test_falls_back_when_any_packet_pose_is_missing(self):
        expected = [_point(3.0, 1.0), _point(4.0, 2.0)]

        points, applied = deskew_scan(
            [([expected[0]], _pose(1.0)), ([expected[1]], None)],
            _pose(2.0),
        )

        self.assertFalse(applied)
        self.assertEqual(points, expected)


if __name__ == "__main__":
    unittest.main()
