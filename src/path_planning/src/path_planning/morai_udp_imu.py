"""Parser for the MORAI IMU UDP format used by SIM: Drive 26.R1."""

import math
import struct
from dataclasses import dataclass


PACKET_HEADER = b"#IMUData$"
PACKET_DATA_LENGTH = 80
PACKET_SIZE = 107
PACKET_TAIL = b"\r\n"


class ImuPacketError(ValueError):
    """Raised when a datagram is not the documented MORAI IMU packet."""


@dataclass(frozen=True)
class ImuMeasurement:
    orientation_xyzw: tuple
    angular_velocity_radps: tuple
    linear_acceleration_mps2: tuple


def parse_imu_packet(packet):
    """Parse the official 107-byte IMU datagram.

    MORAI puts quaternion components on the wire as w, x, y, z.  The returned
    tuple is deliberately converted to the conventional x, y, z, w order.
    """
    if len(packet) != PACKET_SIZE:
        raise ImuPacketError("expected {} bytes, received {}".format(PACKET_SIZE, len(packet)))
    if packet[:9] != PACKET_HEADER:
        raise ImuPacketError("unexpected IMU header {!r}".format(packet[:9]))
    data_length = struct.unpack_from("<I", packet, 9)[0]
    if data_length != PACKET_DATA_LENGTH:
        raise ImuPacketError("expected data_length 80, received {}".format(data_length))
    if packet[-2:] != PACKET_TAIL:
        raise ImuPacketError("unexpected IMU packet tail")

    values = struct.unpack_from("<10d", packet, 25)
    if not all(math.isfinite(value) for value in values):
        raise ImuPacketError("IMU packet contains a non-finite value")
    quaternion_norm = math.sqrt(sum(value * value for value in values[:4]))
    if quaternion_norm < 1e-8:
        raise ImuPacketError("IMU orientation quaternion has zero norm")
    orientation_wxyz = tuple(value / quaternion_norm for value in values[:4])
    w, x, y, z = orientation_wxyz
    return ImuMeasurement((x, y, z, w), values[4:7], values[7:10])


def quaternion_to_yaw(orientation_xyzw):
    """Return ENU yaw in radians (counter-clockwise from +X)."""
    x, y, z, w = orientation_xyzw
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
