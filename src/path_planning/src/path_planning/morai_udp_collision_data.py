"""Strict parser for MORAI 26.R1 CollisionData UDP packets."""

import math
import struct
from dataclasses import dataclass


PACKET_HEADER = b"#CollisionData$"
PACKET_DATA_LENGTH = 148
PACKET_SIZE = 181
PACKET_TAIL = b"\r\n"
OBJECT_COUNT = 5
OBJECT_SIZE = 28


class CollisionPacketError(ValueError):
    """Raised when a datagram does not match the expected collision layout."""


@dataclass(frozen=True)
class CollisionObject:
    object_type: int
    object_id: int
    position_m: tuple
    global_offset_m: tuple

    @property
    def populated(self):
        values = (*self.position_m, *self.global_offset_m)
        return self.object_id != 0 or any(abs(value) > 1e-6 for value in values)


@dataclass(frozen=True)
class CollisionData26R1:
    timestamp_sec: float
    objects: tuple

    @property
    def collided_objects(self):
        # In the competition layout element zero is Ego; collisions start at one.
        return tuple(item for item in self.objects[1:] if item.populated)

    @property
    def collision_detected(self):
        return bool(self.collided_objects)


def parse_collision_data_26r1(packet):
    if len(packet) != PACKET_SIZE:
        raise CollisionPacketError(
            "expected {} bytes, received {} (header={!r})".format(
                PACKET_SIZE, len(packet), packet[:15]
            )
        )
    if packet[:15] != PACKET_HEADER:
        raise CollisionPacketError(
            "unexpected header {!r}".format(packet[:15])
        )
    data_length = struct.unpack_from("<I", packet, 15)[0]
    if data_length != PACKET_DATA_LENGTH:
        raise CollisionPacketError(
            "expected data_length {}, received {}".format(
                PACKET_DATA_LENGTH, data_length
            )
        )
    if packet[-2:] != PACKET_TAIL:
        raise CollisionPacketError(
            "unexpected packet tail {!r}".format(packet[-2:])
        )

    seconds, nanoseconds = struct.unpack_from("<II", packet, 31)
    if nanoseconds >= 1_000_000_000:
        raise CollisionPacketError(
            "nanoseconds field is out of range: {}".format(nanoseconds)
        )

    objects = []
    for index in range(OBJECT_COUNT):
        offset = 39 + index * OBJECT_SIZE
        object_type, object_id, *values = struct.unpack_from(
            "<hhffffff", packet, offset
        )
        if object_type not in (-1, 0, 1, 2):
            raise CollisionPacketError(
                "object {} has invalid type {}".format(index, object_type)
            )
        if not all(math.isfinite(value) for value in values):
            raise CollisionPacketError(
                "object {} contains a non-finite value".format(index)
            )
        objects.append(
            CollisionObject(
                object_type=object_type,
                object_id=object_id,
                position_m=tuple(values[:3]),
                global_offset_m=tuple(values[3:]),
            )
        )
    return CollisionData26R1(
        timestamp_sec=seconds + nanoseconds * 1e-9,
        objects=tuple(objects),
    )
