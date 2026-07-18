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
    PACKET_DATA_LENGTH_26R1,
    PACKET_HEADER,
    PACKET_SIZE,
    PACKET_SIZE_26R1,
    EgoCtrlCommand,
    encode_ego_ctrl_cmd,
    encode_ego_ctrl_cmd_25s4,
    encode_ego_ctrl_cmd_26r1,
    external_control_ready,
)


class MoraiUdpCtrlCmdTest(unittest.TestCase):
    def test_encodes_exact_25s4_competition_throttle_layout(self):
        self.assertEqual(PACKET_SIZE, 55)
        self.assertEqual(PACKET_DATA_LENGTH, 23)
        packet = encode_ego_ctrl_cmd_25s4(
            EgoCtrlCommand(
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
        self.assertEqual(fields[3], 0.0)  # velocity command is unused in type 1
        self.assertEqual(fields[4], 0.0)  # acceleration command is unused in type 1
        self.assertAlmostEqual(fields[5], 0.3)
        self.assertAlmostEqual(fields[7], -0.25)
        self.assertEqual(packet[-2:], b"\r\n")

    def test_encodes_optional_26r1_rear_steer_layout(self):
        packet = encode_ego_ctrl_cmd_26r1(
            EgoCtrlCommand(accel=0.2, rear_steering_normalized=-0.1)
        )
        self.assertEqual(len(packet), PACKET_SIZE_26R1)
        self.assertEqual(
            struct.unpack_from("<I", packet, 14)[0], PACKET_DATA_LENGTH_26R1
        )
        fields = struct.unpack_from("<BBBffffff", packet, 30)
        self.assertAlmostEqual(fields[5], 0.2)
        self.assertAlmostEqual(fields[8], -0.1)

    def test_generic_encoder_defaults_to_25s4(self):
        self.assertEqual(len(encode_ego_ctrl_cmd(EgoCtrlCommand())), 55)

    def test_rejects_out_of_range_steering(self):
        with self.assertRaises(ValueError):
            encode_ego_ctrl_cmd_25s4(EgoCtrlCommand(steering_normalized=1.1))

    def test_rejects_disallowed_velocity_control(self):
        with self.assertRaises(ValueError):
            encode_ego_ctrl_cmd_25s4(EgoCtrlCommand(long_cmd_type=2))

    def test_external_control_requires_auto_mode_and_drive(self):
        self.assertTrue(external_control_ready(2, 4))
        self.assertFalse(external_control_ready(1, 4))
        self.assertFalse(external_control_ready(2, 1))


if __name__ == "__main__":
    unittest.main()
