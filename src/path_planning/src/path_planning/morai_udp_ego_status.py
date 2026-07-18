"""Parser for MORAI SIM: Drive 24.R1 Ego Vehicle Status UDP packets."""

import math
import struct
from dataclasses import dataclass


PACKET_HEADER = b"#MoraiInfo$"
PACKET_DATA_LENGTH = 152
PACKET_SIZE = 181
PACKET_TAIL = b"\r\n"


class UdpPacketError(ValueError):
    """Raised when a datagram is not the documented 24.R1 packet."""


@dataclass(frozen=True)
class EgoVehicleStatus24R1:
    timestamp_sec: float
    ctrl_mode: int
    gear: int
    signed_velocity_kmh: float
    map_data_id: int
    accel_pedal: float
    brake_pedal: float
    size_m: tuple
    overhang_m: float
    wheelbase_m: float
    rear_overhang_m: float
    position_m: tuple
    rotation_deg: tuple
    velocity_kmh: tuple
    angular_velocity_degps: tuple
    acceleration_mps2: tuple
    steer_deg: float
    link_id: str


def _unpack_floats(packet, offset, count):
    return struct.unpack_from("<{}f".format(count), packet, offset)


def parse_ego_vehicle_status_24r1(packet):
    """Parse one exact 181-byte MORAI 24.R1 Ego Vehicle Status datagram."""
    if len(packet) != PACKET_SIZE:
        raise UdpPacketError("expected {} bytes, received {}".format(PACKET_SIZE, len(packet)))
    if packet[:11] != PACKET_HEADER:
        raise UdpPacketError("unexpected header {!r}".format(packet[:11]))

    data_length = struct.unpack_from("<I", packet, 11)[0]
    if data_length != PACKET_DATA_LENGTH:
        raise UdpPacketError(
            "expected data_length {}, received {}".format(PACKET_DATA_LENGTH, data_length)
        )
    if packet[-2:] != PACKET_TAIL:
        raise UdpPacketError("unexpected packet tail {!r}".format(packet[-2:]))

    seconds, nanoseconds = struct.unpack_from("<II", packet, 27)
    if nanoseconds >= 1_000_000_000:
        raise UdpPacketError("nanoseconds field is out of range: {}".format(nanoseconds))

    ctrl_mode, gear = struct.unpack_from("<BB", packet, 35)
    signed_velocity_kmh = struct.unpack_from("<f", packet, 37)[0]
    map_data_id = struct.unpack_from("<i", packet, 41)[0]
    accel_pedal, brake_pedal = _unpack_floats(packet, 45, 2)
    size_m = _unpack_floats(packet, 53, 3)
    overhang_m, wheelbase_m, rear_overhang_m = _unpack_floats(packet, 65, 3)
    position_m = _unpack_floats(packet, 77, 3)
    rotation_deg = _unpack_floats(packet, 89, 3)
    velocity_kmh = _unpack_floats(packet, 101, 3)
    angular_velocity_degps = _unpack_floats(packet, 113, 3)
    acceleration_mps2 = _unpack_floats(packet, 125, 3)
    steer_deg = struct.unpack_from("<f", packet, 137)[0]
    link_id = packet[141:179].split(b"\x00", 1)[0].decode("ascii", errors="replace").strip()

    numeric_values = (
        signed_velocity_kmh,
        accel_pedal,
        brake_pedal,
        *size_m,
        overhang_m,
        wheelbase_m,
        rear_overhang_m,
        *position_m,
        *rotation_deg,
        *velocity_kmh,
        *angular_velocity_degps,
        *acceleration_mps2,
        steer_deg,
    )
    if not all(math.isfinite(value) for value in numeric_values):
        raise UdpPacketError("packet contains a non-finite numeric value")

    return EgoVehicleStatus24R1(
        timestamp_sec=seconds + nanoseconds * 1e-9,
        ctrl_mode=ctrl_mode,
        gear=gear,
        signed_velocity_kmh=signed_velocity_kmh,
        map_data_id=map_data_id,
        accel_pedal=accel_pedal,
        brake_pedal=brake_pedal,
        size_m=size_m,
        overhang_m=overhang_m,
        wheelbase_m=wheelbase_m,
        rear_overhang_m=rear_overhang_m,
        position_m=position_m,
        rotation_deg=rotation_deg,
        velocity_kmh=velocity_kmh,
        angular_velocity_degps=angular_velocity_degps,
        acceleration_mps2=acceleration_mps2,
        steer_deg=steer_deg,
        link_id=link_id,
    )
