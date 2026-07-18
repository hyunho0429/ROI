#!/usr/bin/env python3

import math
import os
import struct
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from path_planning.morai_udp_gps import parse_nmea_datagram
from path_planning.morai_udp_imu import parse_imu_packet, quaternion_to_yaw


class MoraiUdpSensorsTest(unittest.TestCase):
    def test_merges_nmea_rmc_and_gga(self):
        packet = (
            b"$GPRMC,123519,A,3723.2475,N,12658.3416,E,10.0,90.0,230394,,,A\r\n"
            b"$GPGGA,123519,3723.2475,N,12658.3416,E,1,08,0.9,42.5,M,0.0,M,,\r\n"
        )
        measurement = parse_nmea_datagram(packet)
        self.assertTrue(measurement.fix_valid)
        self.assertAlmostEqual(measurement.latitude_deg, 37.0 + 23.2475 / 60.0)
        self.assertAlmostEqual(measurement.longitude_deg, 126.0 + 58.3416 / 60.0)
        self.assertAlmostEqual(measurement.altitude_m, 42.5)
        self.assertAlmostEqual(measurement.speed_mps, 5.1444444444)
        self.assertAlmostEqual(measurement.course_deg, 90.0)

    def test_parses_documented_wxyz_107_byte_imu_packet(self):
        yaw = math.radians(45.0)
        packet = bytearray(107)
        packet[:9] = b"#IMUData$"
        struct.pack_into("<I", packet, 9, 80)
        values = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0), 0.1, 0.2, 0.3, 1.0, 2.0, 3.0)
        struct.pack_into("<10d", packet, 25, *values)
        packet[-2:] = b"\r\n"
        measurement = parse_imu_packet(bytes(packet))
        self.assertAlmostEqual(quaternion_to_yaw(measurement.orientation_xyzw), yaw)
        self.assertIsNone(measurement.timestamp_sec)
        self.assertEqual(measurement.angular_velocity_radps, (0.1, 0.2, 0.3))
        self.assertEqual(measurement.linear_acceleration_mps2, (1.0, 2.0, 3.0))

    def test_parses_official_115_byte_imu_packet_with_timestamp(self):
        yaw = math.radians(45.0)
        packet = bytearray(115)
        packet[:9] = b"#IMUData$"
        struct.pack_into("<I", packet, 9, 80)
        values = (
            math.cos(yaw / 2.0),
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            0.1,
            0.2,
            0.3,
            1.0,
            2.0,
            3.0,
        )
        struct.pack_into("<II", packet, 25, 1234, 500000000)
        struct.pack_into("<10d", packet, 33, *values)
        packet[-2:] = b"\r\n"

        measurement = parse_imu_packet(bytes(packet))

        self.assertAlmostEqual(quaternion_to_yaw(measurement.orientation_xyzw), yaw)
        self.assertAlmostEqual(measurement.timestamp_sec, 1234.5)
        self.assertEqual(measurement.angular_velocity_radps, (0.1, 0.2, 0.3))
        self.assertEqual(measurement.linear_acceleration_mps2, (1.0, 2.0, 3.0))

    def test_accepts_25_01_115_byte_packet_reporting_length_88(self):
        yaw = math.radians(45.0)
        packet = bytearray(115)
        packet[:9] = b"#IMUData$"
        struct.pack_into("<I", packet, 9, 88)
        values = (
            math.cos(yaw / 2.0),
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            0.1,
            0.2,
            0.3,
            1.0,
            2.0,
            3.0,
        )
        struct.pack_into("<II", packet, 25, 1234, 500000000)
        struct.pack_into("<10d", packet, 33, *values)
        packet[-2:] = b"\r\n"

        measurement = parse_imu_packet(bytes(packet))

        self.assertAlmostEqual(quaternion_to_yaw(measurement.orientation_xyzw), yaw)
        self.assertAlmostEqual(measurement.timestamp_sec, 1234.5)
        self.assertEqual(measurement.angular_velocity_radps, (0.1, 0.2, 0.3))
        self.assertEqual(measurement.linear_acceleration_mps2, (1.0, 2.0, 3.0))


if __name__ == "__main__":
    unittest.main()
