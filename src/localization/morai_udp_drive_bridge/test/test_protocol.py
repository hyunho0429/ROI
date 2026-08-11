import struct
import unittest

from morai_udp_drive_bridge.protocol import (
    EGO_CTRL_CMD_PACKET_SIZE,
    EGO_STATUS_PACKET_SIZE,
    build_ego_ctrl_cmd,
    parse_ego_vehicle_status,
)


class EgoProtocolTest(unittest.TestCase):
    def test_control_packet(self):
        packet = build_ego_ctrl_cmd(
            cmd_type=2,
            velocity_kmh=7.2,
            steer_normalized=0.5,
        )
        self.assertEqual(len(packet), EGO_CTRL_CMD_PACKET_SIZE)
        self.assertEqual(packet[:14], b"#MoraiCtrlCmd$")

    def test_status_packet(self):
        floats = [float(index) for index in range(24)]
        packet = struct.pack(
            "<11s i 3i i i b b f i 24f 38s 2s",
            b"#MoraiStatus",
            102,
            0,
            0,
            0,
            10,
            20,
            2,
            4,
            7.2,
            10000,
            *floats,
            b"LINK_1",
            b"\r\n",
        )
        self.assertEqual(len(packet), EGO_STATUS_PACKET_SIZE)
        status = parse_ego_vehicle_status(packet)
        self.assertEqual(status.pos_x_m, 8.0)
        self.assertEqual(status.heading_deg, 13.0)
        self.assertEqual(status.link_id, "LINK_1")


if __name__ == "__main__":
    unittest.main()
