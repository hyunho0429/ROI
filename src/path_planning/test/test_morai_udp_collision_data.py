#!/usr/bin/env python3

import os
import struct
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from path_planning.morai_udp_collision_data import (
    PACKET_DATA_LENGTH,
    PACKET_HEADER,
    PACKET_SIZE,
    CollisionPacketError,
    parse_collision_data_26r1,
)


def make_packet(collided=False):
    packet = bytearray(PACKET_SIZE)
    packet[:15] = PACKET_HEADER
    struct.pack_into("<I", packet, 15, PACKET_DATA_LENGTH)
    struct.pack_into("<II", packet, 31, 10, 250_000_000)
    struct.pack_into("<hhffffff", packet, 39, -1, 7, 0, 0, 0, 300000, 4100000, 0)
    if collided:
        struct.pack_into("<hhffffff", packet, 67, 1, 42, 1, 2, 3, 300000, 4100000, 0)
    packet[-2:] = b"\r\n"
    return bytes(packet)


def make_first_slot_collision_packet():
    packet = bytearray(PACKET_SIZE)
    packet[:15] = PACKET_HEADER
    struct.pack_into("<I", packet, 15, PACKET_DATA_LENGTH)
    struct.pack_into("<II", packet, 31, 10, 0)
    struct.pack_into(
        "<hhffffff", packet, 39, 1, 99, 1, 2, 3, 300000, 4100000, 0
    )
    packet[-2:] = b"\r\n"
    return bytes(packet)


class CollisionDataTest(unittest.TestCase):
    def test_detects_second_populated_object(self):
        collision = parse_collision_data_26r1(make_packet(collided=True))
        self.assertAlmostEqual(collision.timestamp_sec, 10.25)
        self.assertTrue(collision.collision_detected)
        self.assertEqual(collision.collided_objects[0].object_id, 42)

    def test_empty_slots_are_not_collision(self):
        self.assertFalse(parse_collision_data_26r1(make_packet()).collision_detected)

    def test_detects_non_ego_collision_in_first_slot(self):
        collision = parse_collision_data_26r1(make_first_slot_collision_packet())
        self.assertTrue(collision.collision_detected)
        self.assertEqual(collision.collided_objects[0].object_id, 99)

    def test_rejects_wrong_header(self):
        packet = bytearray(make_packet())
        packet[:15] = b"wrong".ljust(15, b"\x00")
        with self.assertRaises(CollisionPacketError):
            parse_collision_data_26r1(bytes(packet))


if __name__ == "__main__":
    unittest.main()
