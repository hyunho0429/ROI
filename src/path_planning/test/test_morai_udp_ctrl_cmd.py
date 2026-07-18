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
    EgoCtrlCommand24R1,
    encode_ego_ctrl_cmd_24r1,
)


class MoraiUdpCtrlCmdTest(unittest.TestCase):
    def test_encodes_exact_documented_24r1_layout(self):
        packet = encode_ego_ctrl_cmd_24r1(
            EgoCtrlCommand24R1(
                ctrl_mode=2,
                gear=4,
                long_cmd_type=2,
                velocity_kmh=25.0,
                steering_normalized=-0.25,
            )
        )
        self.assertEqual(len(packet), PACKET_SIZE)
        self.assertEqual(packet[:14], PACKET_HEADER)
        self.assertEqual(struct.unpack_from("<I", packet, 14)[0], PACKET_DATA_LENGTH)
        self.assertEqual(packet[18:30], bytes(12))
        fields = struct.unpack_from("<BBBfffff", packet, 30)
        self.assertEqual(fields[:3], (2, 4, 2))
        self.assertAlmostEqual(fields[3], 25.0)
        self.assertAlmostEqual(fields[7], -0.25)
        self.assertEqual(packet[-2:], b"\r\n")

    def test_rejects_out_of_range_steering(self):
        with self.assertRaises(ValueError):
            encode_ego_ctrl_cmd_24r1(EgoCtrlCommand24R1(steering_normalized=1.1))


if __name__ == "__main__":
    unittest.main()
