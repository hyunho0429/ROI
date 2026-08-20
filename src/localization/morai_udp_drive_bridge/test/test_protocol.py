import struct
import unittest

from morai_udp_drive_bridge.protocol import (
    COMPETITION_STATUS_BASE_PACKET_SIZE,
    COMPETITION_STATUS_EXTENDED_PACKET_SIZE,
    COMPETITION_STATUS_HOST_PORT,
    COMPETITION_STATUS_MAX_PACKET_SIZE,
    COMPETITION_STATUS_PORT,
    EGO_CTRL_CMD_PACKET_SIZE,
    ProtocolError,
    build_ego_ctrl_cmd,
    parse_competition_vehicle_status,
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

    @staticmethod
    def make_competition_status_packet(packet_size):
        packet = bytearray(packet_size)
        packet[:11] = b"#MoraiInfo$"
        struct.pack_into(
            "<I",
            packet,
            11,
            152 if packet_size == COMPETITION_STATUS_BASE_PACKET_SIZE else 200,
        )
        struct.pack_into("<II", packet, 27, 10, 20)
        struct.pack_into("<bb", packet, 35, 2, 4)
        struct.pack_into("<f", packet, 37, 7.2)
        struct.pack_into("<i", packet, 41, 10000)
        struct.pack_into("<24f", packet, 45, *[float(index) for index in range(24)])
        packet[141:179] = b"LINK_1".ljust(38, b"\x00")
        if packet_size == COMPETITION_STATUS_EXTENDED_PACKET_SIZE:
            struct.pack_into("<12f", packet, 179, *[float(index) for index in range(12)])
        packet[-2:] = b"\r\n"
        return bytes(packet)

    def test_competition_status_defaults(self):
        self.assertEqual(COMPETITION_STATUS_HOST_PORT, 9080)
        self.assertEqual(COMPETITION_STATUS_PORT, 9081)
        self.assertEqual(
            COMPETITION_STATUS_MAX_PACKET_SIZE,
            COMPETITION_STATUS_EXTENDED_PACKET_SIZE,
        )

    def test_base_competition_status_packet(self):
        packet = self.make_competition_status_packet(
            COMPETITION_STATUS_BASE_PACKET_SIZE
        )
        status = parse_competition_vehicle_status(packet)
        self.assertEqual(status.position_m, (8.0, 9.0, 10.0))
        self.assertEqual(status.rotation_deg[2], 13.0)
        self.assertEqual(status.link_id, "LINK_1")
        self.assertEqual(status.tire_lateral_force, ())

    def test_extended_competition_status_packet(self):
        packet = self.make_competition_status_packet(
            COMPETITION_STATUS_EXTENDED_PACKET_SIZE
        )
        status = parse_competition_vehicle_status(packet)
        self.assertEqual(status.tire_lateral_force, (0.0, 1.0, 2.0, 3.0))

    def test_rejects_invalid_status_header(self):
        packet = bytearray(
            self.make_competition_status_packet(COMPETITION_STATUS_BASE_PACKET_SIZE)
        )
        packet[:11] = b"#MoraiStatus"
        with self.assertRaises(ProtocolError):
            parse_competition_vehicle_status(bytes(packet))


if __name__ == "__main__":
    unittest.main()
