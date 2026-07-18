#!/usr/bin/env python3

import csv
import os
import struct
import sys
import tempfile
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from morai_global_csv_recorder import MoraiGlobalCsvRecorder, status_to_sample
from path_planning.morai_udp_ego_status import (
    PACKET_DATA_LENGTH,
    PACKET_HEADER,
    PACKET_SIZE,
    UdpPacketError,
    parse_ego_vehicle_status_24r1,
)


def make_packet():
    packet = bytearray(PACKET_SIZE)
    packet[:11] = PACKET_HEADER
    struct.pack_into("<I", packet, 11, PACKET_DATA_LENGTH)
    struct.pack_into("<II", packet, 27, 123, 500_000_000)
    struct.pack_into("<BB", packet, 35, 1, 4)
    struct.pack_into("<f", packet, 37, 36.0)
    struct.pack_into("<i", packet, 41, 10001)
    struct.pack_into("<ff", packet, 45, 0.2, 0.0)
    struct.pack_into("<fff", packet, 53, 1.8, 4.7, 1.6)
    struct.pack_into("<fff", packet, 65, 0.9, 2.7, 0.8)
    struct.pack_into("<fff", packet, 77, 10.0, 20.0, 3.0)
    struct.pack_into("<fff", packet, 89, 1.0, 2.0, 90.0)
    struct.pack_into("<fff", packet, 101, 36.0, 18.0, 3.6)
    struct.pack_into("<fff", packet, 113, 0.1, 0.2, 0.3)
    struct.pack_into("<fff", packet, 125, 1.0, 2.0, 3.0)
    struct.pack_into("<f", packet, 137, -5.0)
    packet[141:179] = b"A123".ljust(38, b"\x00")
    packet[179:181] = b"\r\n"
    return bytes(packet)


class MoraiUdpEgoStatusTest(unittest.TestCase):
    def test_parses_documented_24r1_packet_offsets(self):
        status = parse_ego_vehicle_status_24r1(make_packet())

        self.assertAlmostEqual(status.timestamp_sec, 123.5)
        self.assertEqual(status.ctrl_mode, 1)
        self.assertEqual(status.gear, 4)
        self.assertEqual(status.map_data_id, 10001)
        self.assertEqual(status.position_m, (10.0, 20.0, 3.0))
        self.assertEqual(status.rotation_deg, (1.0, 2.0, 90.0))
        self.assertAlmostEqual(status.wheelbase_m, 2.7, places=5)
        self.assertEqual(status.link_id, "A123")

    def test_converts_udp_kmh_velocity_to_csv_mps(self):
        status = parse_ego_vehicle_status_24r1(make_packet())
        sample = status_to_sample(status, receive_time_sec=200.0)

        self.assertEqual(sample["enu"], (10.0, 20.0, 3.0))
        self.assertAlmostEqual(sample["velocity"][0], 10.0)
        self.assertAlmostEqual(sample["velocity"][1], 5.0)
        self.assertAlmostEqual(sample["velocity"][2], 1.0)
        self.assertAlmostEqual(sample["signed_speed_mps"], 10.0)

    def test_rejects_other_packet_versions(self):
        with self.assertRaises(UdpPacketError):
            parse_ego_vehicle_status_24r1(make_packet() + b"extra")

        packet = bytearray(make_packet())
        struct.pack_into("<I", packet, 11, 216)
        with self.assertRaises(UdpPacketError):
            parse_ego_vehicle_status_24r1(bytes(packet))

    def test_recorder_writes_uniform_interpolated_3d_points(self):
        first_packet = bytearray(make_packet())
        second_packet = bytearray(make_packet())
        struct.pack_into("<fff", first_packet, 77, 0.0, 0.0, 0.0)
        struct.pack_into("<fff", second_packet, 77, 0.0, 0.0, 1.2)

        with tempfile.TemporaryDirectory() as directory:
            output_file = os.path.join(directory, "udp_path.csv")
            recorder = MoraiGlobalCsvRecorder(output_file, sample_distance=0.5)
            recorder.add_sample(
                status_to_sample(parse_ego_vehicle_status_24r1(bytes(first_packet)), 100.0)
            )
            recorder.add_sample(
                status_to_sample(parse_ego_vehicle_status_24r1(bytes(second_packet)), 101.0)
            )
            recorder.close()

            with open(output_file, newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))

        self.assertEqual(len(rows), 3)
        self.assertEqual([float(row["global_enu_z_m"]) for row in rows], [0.0, 0.5, 1.0])


if __name__ == "__main__":
    unittest.main()
