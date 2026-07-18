#!/usr/bin/env python3

import os
import struct
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from path_planning.morai_udp_ctrl_cmd import (
    PACKET_DATA_LENGTH,
    PACKET_HEADER,
    PACKET_SIZE,
    EgoCtrlCommand26R1,
    encode_ego_ctrl_cmd_26r1,
    external_control_ready,
)


class MoraiUdpCtrlCmdTest(unittest.TestCase):
    def test_encodes_exact_competition_throttle_layout(self):
        self.assertEqual(PACKET_SIZE, 55)
        self.assertEqual(PACKET_DATA_LENGTH, 23)
        packet = encode_ego_ctrl_cmd_26r1(
            EgoCtrlCommand26R1(
                ctrl_mode=2,
                gear=4,
                long_cmd_type=1,
                accel=0.3,
                steering_normalized=-0.25,
            )
        )
        self.assertEqual(len(packet), PACKET_SIZE)
        self.assertEqual(packet[:14], PACKET_HEADER)
        self.assertEqual(struct.unpack_from("<I", packet, 14)[0], PACKET_DATA_LENGTH)
        self.assertEqual(packet[18:30], bytes(12))
        fields = struct.unpack_from("<BBBfffff", packet, 30)
        self.assertEqual(fields[:3], (2, 4, 1))
        self.assertAlmostEqual(fields[5], 0.3)
        self.assertAlmostEqual(fields[7], -0.25)
        self.assertEqual(packet[-2:], b"\r\n")

    def test_rejects_out_of_range_steering(self):
        with self.assertRaises(ValueError):
            encode_ego_ctrl_cmd_26r1(EgoCtrlCommand26R1(steering_normalized=1.1))

    def test_rejects_disallowed_velocity_control(self):
        with self.assertRaises(ValueError):
            encode_ego_ctrl_cmd_26r1(EgoCtrlCommand26R1(long_cmd_type=2))

    def test_external_control_requires_auto_mode_and_drive(self):
        self.assertTrue(external_control_ready(2, 4))
        self.assertFalse(external_control_ready(1, 4))
        self.assertFalse(external_control_ready(2, 1))


if __name__ == "__main__":
    unittest.main()
