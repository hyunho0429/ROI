#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import struct
import sys
from pathlib import Path
import unittest


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from morai_udp_bridge.protocol import (  # noqa: E402
    GPS_PACKET_SIZE,
    LEGACY_IMU_PACKET_SIZE,
    IMU_PACKET_SIZE,
    parse_gps_packet,
    parse_imu_packet,
)


def nmea_sentence(body: str) -> bytes:
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return f"${body}*{checksum:02X}\r\n".encode("ascii")


def gps_packet(sentence: bytes) -> bytes:
    return sentence[:6] + sentence[6:] + b"\x00" * (GPS_PACKET_SIZE - len(sentence))


class ProtocolTest(unittest.TestCase):
    def test_gga_packet(self):
        sentence = nmea_sentence(
            "GPGGA,123519,3723.2475,N,12701.3416,E,1,08,0.9,545.4,M,46.9,M,,"
        )
        measurement = parse_gps_packet(gps_packet(sentence))
        self.assertEqual(measurement.sentence_type, "GPGGA")
        self.assertAlmostEqual(measurement.latitude, 37.3874583333, places=8)
        self.assertAlmostEqual(measurement.longitude, 127.02236, places=8)
        self.assertAlmostEqual(measurement.altitude, 545.4)
        self.assertEqual(measurement.status, 1)

    def test_rmc_packet_direction(self):
        sentence = nmea_sentence(
            "GPRMC,123519,A,3723.2475,S,12701.3416,W,000.5,054.7,191194,,,A"
        )
        measurement = parse_gps_packet(gps_packet(sentence))
        self.assertLess(measurement.latitude, 0.0)
        self.assertLess(measurement.longitude, 0.0)
        self.assertEqual(measurement.status, 1)

    def test_short_nmea_packet(self):
        sentence = nmea_sentence(
            "GPRMC,123519,A,3723.2475,N,12701.3416,E,000.5,054.7,191194,,,A"
        )
        measurement = parse_gps_packet(sentence)
        self.assertEqual(measurement.packet_length, len(sentence))
        self.assertEqual(measurement.status, 1)

    def test_networkmodule_imu_115(self):
        packet = struct.pack(
            "<9s i 3i i i 10d 2s",
            b"MORAIIMU\x00",
            80,
            1,
            2,
            3,
            1700000000,
            123,
            1.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.2,
            0.3,
            1.0,
            2.0,
            3.0,
            b"\r\n",
        )
        self.assertEqual(len(packet), IMU_PACKET_SIZE)
        measurement = parse_imu_packet(packet)
        self.assertEqual(measurement.layout, "networkmodule_115")
        self.assertEqual(measurement.auxiliary, (1, 2, 3))
        self.assertEqual(measurement.sec, 1700000000)
        self.assertEqual(measurement.linear_acceleration, (1.0, 2.0, 3.0))

    def test_legacy_imu_107(self):
        packet = struct.pack(
            "<9s 4i 10d 2s",
            b"MORAIIMU\x00",
            80,
            7,
            1700000000,
            123,
            1.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.2,
            0.3,
            1.0,
            2.0,
            3.0,
            b"\r\n",
        )
        self.assertEqual(len(packet), LEGACY_IMU_PACKET_SIZE)
        measurement = parse_imu_packet(packet)
        self.assertEqual(measurement.layout, "legacy_107")
        self.assertEqual(measurement.auxiliary, (7,))

    def test_networkmodule_imu_accepts_data_length_88(self):
        packet = struct.pack(
            "<9s i 3i i i 10d 2s",
            b"MORAIIMU\x00",
            88,
            1,
            2,
            3,
            1700000000,
            123,
            1.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.2,
            0.3,
            1.0,
            2.0,
            3.0,
            b"\r\n",
        )
        measurement = parse_imu_packet(packet)
        self.assertEqual(measurement.data_length, 88)
        self.assertEqual(measurement.linear_acceleration, (1.0, 2.0, 3.0))


if __name__ == "__main__":
    unittest.main()
