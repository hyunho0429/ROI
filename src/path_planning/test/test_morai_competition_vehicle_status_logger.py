#!/usr/bin/env python3

import os
import sys
import unittest
from types import SimpleNamespace


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from morai_competition_vehicle_status_logger import (
    _validate_arguments,
    format_status_snapshot,
)
from path_planning.morai_udp_competition_status import CompetitionVehicleStatus


def _status(extended=True):
    return CompetitionVehicleStatus(
        timestamp_sec=123.5,
        ctrl_mode=2,
        gear=4,
        signed_velocity_kmh=36.0,
        map_data_id=10001,
        accel_pedal=0.2,
        brake_pedal=0.0,
        size_m=(1.892, 4.635, 2.434),
        overhang_m=0.845,
        wheelbase_m=3.0,
        rear_overhang_m=0.79,
        position_m=(10.0, 20.0, 3.0),
        rotation_deg=(1.0, 2.0, 90.0),
        velocity_kmh=(36.0, 0.0, 0.0),
        angular_velocity_degps=(0.1, 0.2, 0.3),
        acceleration_mps2=(1.0, 2.0, 3.0),
        front_steer_deg=-5.0,
        link_id="A123",
        tire_lateral_force=(0.0, 1.0, 2.0, 3.0) if extended else (),
        side_slip_angle=(4.0, 5.0, 6.0, 7.0) if extended else (),
        tire_cornering_stiffness=(8.0, 9.0, 10.0, 11.0) if extended else (),
    )


class CompetitionVehicleStatusLoggerTest(unittest.TestCase):
    def test_formats_every_decoded_group_in_one_snapshot(self):
        snapshot = format_status_snapshot(
            _status(),
            sender=("192.168.0.10", 9080),
            packet_size=229,
            snapshot_index=7,
            received_at="2026-08-03T12:34:56.789+09:00",
        )

        expected_text = (
            "Competition Vehicle Status #000007",
            "sender=192.168.0.10:9080",
            "229 bytes (extended)",
            "ctrl_mode : 2 (AV-ExternalCtrl)",
            "gear      : 4 (D)",
            "accel=0.200, brake=0.000",
            "signed_speed : 36.000km/h (10.000m/s)",
            "position : xyz=(10.000, 20.000, 3.000)m",
            "map_data_id=10001, link_id='A123'",
            "size xyz       : (1.892, 4.635, 2.434)m",
            "wheelbase      : 3.000m",
            "lateral_force       : (0.000, 1.000, 2.000, 3.000)",
            "side_slip_angle     : (4.000, 5.000, 6.000, 7.000)",
            "cornering_stiffness : (8.000, 9.000, 10.000, 11.000)",
        )
        for text in expected_text:
            self.assertIn(text, snapshot)

    def test_marks_tire_fields_absent_for_base_packet(self):
        snapshot = format_status_snapshot(
            _status(extended=False),
            sender=("127.0.0.1", 9080),
            packet_size=181,
            snapshot_index=1,
            received_at="now",
        )

        self.assertIn("181 bytes (base)", snapshot)
        self.assertIn("not present in the 181-byte base packet", snapshot)

    def test_accepts_infinite_count_and_zero_interval(self):
        arguments = SimpleNamespace(
            host_port=9080,
            destination_port=9081,
            interval=0.0,
            count=0,
            timeout=15.0,
        )

        _validate_arguments(arguments)

    def test_rejects_negative_count(self):
        arguments = SimpleNamespace(
            host_port=9080,
            destination_port=9081,
            interval=1.0,
            count=-1,
            timeout=15.0,
        )

        with self.assertRaises(ValueError):
            _validate_arguments(arguments)


if __name__ == "__main__":
    unittest.main()
