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
    EgoVehicleStatusPacketError,
    parse_ego_vehicle_status,
)
from path_planning.morai_udp_competition_status import (
    BASE_PACKET_DATA_LENGTH,
    BASE_PACKET_SIZE,
    EXTENDED_PACKET_DATA_LENGTH,
    EXTENDED_PACKET_SIZE,
    PACKET_DATA_LENGTH,
    PACKET_HEADER,
    PACKET_SIZE,
    CompetitionStatusPacketError,
    parse_competition_vehicle_status,
)


def make_packet(packet_size=EXTENDED_PACKET_SIZE):
    data_length = (
        BASE_PACKET_DATA_LENGTH
        if packet_size == BASE_PACKET_SIZE
        else EXTENDED_PACKET_DATA_LENGTH
    )
    packet = bytearray(packet_size)
    packet[:11] = PACKET_HEADER
    struct.pack_into("<I", packet, 11, data_length)
    struct.pack_into("<II", packet, 27, 123, 500_000_000)
    struct.pack_into("<bb", packet, 35, 2, 4)
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
    if packet_size == EXTENDED_PACKET_SIZE:
        struct.pack_into("<12f", packet, 179, *[float(index) for index in range(12)])
    packet[-2:] = b"\r\n"
    return bytes(packet)


class MoraiUdpCompetitionStatusTest(unittest.TestCase):
    def test_parses_documented_181_byte_ego_status(self):
        status = parse_ego_vehicle_status(make_packet(BASE_PACKET_SIZE))
        self.assertEqual(status.position_m, (10.0, 20.0, 3.0))

    def test_ego_status_rejects_competition_extension(self):
        with self.assertRaises(EgoVehicleStatusPacketError):
            parse_ego_vehicle_status(make_packet(EXTENDED_PACKET_SIZE))

    def test_parses_observed_181_byte_competition_layout(self):
        status = parse_competition_vehicle_status(make_packet(BASE_PACKET_SIZE))
        self.assertEqual(status.position_m, (10.0, 20.0, 3.0))
        self.assertEqual(status.rotation_deg, (1.0, 2.0, 90.0))
        self.assertEqual(status.link_id, "A123")
        self.assertEqual(status.tire_lateral_force, ())
        self.assertEqual(status.side_slip_angle, ())
        self.assertEqual(status.tire_cornering_stiffness, ())

    def test_parses_exact_229_byte_competition_layout(self):
        status = parse_competition_vehicle_status(make_packet())
        self.assertAlmostEqual(status.timestamp_sec, 123.5)
        self.assertEqual(status.ctrl_mode, 2)
        self.assertEqual(status.gear, 4)
        self.assertEqual(status.position_m, (10.0, 20.0, 3.0))
        self.assertEqual(status.rotation_deg, (1.0, 2.0, 90.0))
        self.assertAlmostEqual(status.wheelbase_m, 2.7, places=5)
        self.assertEqual(status.link_id, "A123")
        self.assertEqual(status.tire_lateral_force, (0.0, 1.0, 2.0, 3.0))
        self.assertEqual(status.side_slip_angle, (4.0, 5.0, 6.0, 7.0))
        self.assertEqual(status.tire_cornering_stiffness, (8.0, 9.0, 10.0, 11.0))

    def test_rejects_other_packet_versions(self):
        with self.assertRaises(CompetitionStatusPacketError):
            parse_competition_vehicle_status(make_packet()[:-48])

    def test_recorder_converts_units_and_samples_once_per_second(self):
        first_packet = bytearray(make_packet(BASE_PACKET_SIZE))
        middle_packet = bytearray(make_packet(BASE_PACKET_SIZE))
        second_packet = bytearray(make_packet(BASE_PACKET_SIZE))
        struct.pack_into("<fff", first_packet, 77, 0.0, 0.0, 0.0)
        struct.pack_into("<fff", middle_packet, 77, 0.0, 0.0, 0.6)
        struct.pack_into("<fff", second_packet, 77, 0.0, 0.0, 1.2)

        with tempfile.TemporaryDirectory() as directory:
            output_file = os.path.join(directory, "udp_path.csv")
            recorder = MoraiGlobalCsvRecorder(output_file, sample_period=1.0)
            for packet, receive_time in (
                (first_packet, 100.0),
                (middle_packet, 100.5),
                (second_packet, 101.0),
            ):
                status = parse_ego_vehicle_status(bytes(packet))
                recorder.add_sample(status_to_sample(status, receive_time))
            recorder.close()
            with open(output_file, newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))

        self.assertEqual(len(rows), 2)
        saved_z = [float(row["global_enu_z_m"]) for row in rows]
        self.assertAlmostEqual(saved_z[0], 0.0)
        self.assertAlmostEqual(saved_z[1], 1.2)
        self.assertAlmostEqual(float(rows[0]["velocity_x_mps"]), 10.0)

    def test_recorder_rejects_nonpositive_sample_period(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                MoraiGlobalCsvRecorder(
                    os.path.join(directory, "bad.csv"), sample_period=0.0
                )


if __name__ == "__main__":
    unittest.main()
