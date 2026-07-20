"""Parser for MORAI SIM: Drive 107/115-byte IMU UDP packets."""

import math
import struct
from dataclasses import dataclass


PACKET_HEADER = b"#IMUData$"
PACKET_DATA_LENGTH = 80
PACKET_SIZE = 107
EXTENDED_PACKET_DATA_LENGTH = 88
EXTENDED_PACKET_SIZE = 115
PACKET_TAIL = b"\r\n"


class ImuPacketError(ValueError):
    """Raised when a datagram is not the documented MORAI IMU packet."""


@dataclass(frozen=True)
class ImuMeasurement:
    orientation_xyzw: tuple
    angular_velocity_radps: tuple
    linear_acceleration_mps2: tuple
    timestamp_sec: float = None

    # sensor_msgs/Imu-compatible semantic names used by beta_drive.
    @property
    def orientation(self):
        return self.orientation_xyzw

    @property
    def angular_velocity(self):
        return self.angular_velocity_radps

    @property
    def linear_acceleration(self):
        return self.linear_acceleration_mps2


def parse_imu_packet(packet):
    """Parse a timestamped 115-byte or public 107-byte IMU datagram.

    MORAI puts quaternion components on the wire as w, x, y, z.  The returned
    tuple is deliberately converted to the conventional x, y, z, w order.

    In the timestamped form, bytes 25-32 are uint32 seconds/nanoseconds and the ten doubles
    begin at byte 33.  The legacy 107-byte format has no timestamp and begins
    the doubles at byte 25.  Some 25.01 builds report data_length 88 instead of
    80 for the 115-byte packet, so that length is accepted for compatibility.
    """
    if len(packet) not in (PACKET_SIZE, EXTENDED_PACKET_SIZE):
        raise ImuPacketError(
            "expected {} or {} bytes, received {}".format(
                PACKET_SIZE, EXTENDED_PACKET_SIZE, len(packet)
            )
        )
    if packet[:9] != PACKET_HEADER:
        raise ImuPacketError("unexpected IMU header {!r}".format(packet[:9]))
    data_length = struct.unpack_from("<I", packet, 9)[0]
    expected_lengths = (
        (PACKET_DATA_LENGTH,)
        if len(packet) == PACKET_SIZE
        else (PACKET_DATA_LENGTH, EXTENDED_PACKET_DATA_LENGTH)
    )
    if data_length not in expected_lengths:
        raise ImuPacketError(
            "expected data_length {}, received {}".format(
                " or ".join(str(value) for value in expected_lengths), data_length
            )
        )
    if packet[-2:] != PACKET_TAIL:
        raise ImuPacketError("unexpected IMU packet tail")

    timestamp_sec = None
    if len(packet) == EXTENDED_PACKET_SIZE:
        seconds, nanoseconds = struct.unpack_from("<II", packet, 25)
        if nanoseconds >= 1_000_000_000:
            raise ImuPacketError(
                "nanoseconds field is out of range: {}".format(nanoseconds)
            )
        timestamp_sec = seconds + nanoseconds * 1e-9

    offsets = (25,) if len(packet) == PACKET_SIZE else (33,)
    candidates = []
    for offset in offsets:
        values = struct.unpack_from("<10d", packet, offset)
        if not all(math.isfinite(value) for value in values):
            continue
        quaternion_norm = math.sqrt(sum(value * value for value in values[:4]))
        if 0.8 <= quaternion_norm <= 1.2:
            candidates.append((abs(quaternion_norm - 1.0), values, quaternion_norm))
    if not candidates:
        raise ImuPacketError(
            "IMU packet does not contain a valid unit orientation quaternion"
        )

    _score, values, quaternion_norm = min(candidates, key=lambda item: item[0])
    orientation_wxyz = tuple(value / quaternion_norm for value in values[:4])
    w, x, y, z = orientation_wxyz
    return ImuMeasurement(
        (x, y, z, w), values[4:7], values[7:10], timestamp_sec
    )


def quaternion_to_yaw(orientation_xyzw):
    """Return ENU yaw in radians (counter-clockwise from +X)."""
    x, y, z, w = orientation_xyzw
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
