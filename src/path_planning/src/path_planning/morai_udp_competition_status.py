"""Strict parser for the MORAI Competition Vehicle Status UDP packet.

Competition Vehicle Status is a competition-only interface and is deliberately
kept separate from the public Ego Vehicle Status protocol.
"""

import math
import struct
from dataclasses import dataclass


PACKET_HEADER = b"#MoraiInfo$"
BASE_PACKET_DATA_LENGTH = 152
BASE_PACKET_SIZE = 181
EXTENDED_PACKET_DATA_LENGTH = 200
EXTENDED_PACKET_SIZE = 229
# Backward-compatible aliases for code that builds the extended test packet.
PACKET_DATA_LENGTH = EXTENDED_PACKET_DATA_LENGTH
PACKET_SIZE = EXTENDED_PACKET_SIZE
PACKET_TAIL = b"\r\n"
SUPPORTED_LAYOUTS = {
    BASE_PACKET_SIZE: BASE_PACKET_DATA_LENGTH,
    EXTENDED_PACKET_SIZE: EXTENDED_PACKET_DATA_LENGTH,
}


class CompetitionStatusPacketError(ValueError):
    """Raised when a datagram does not match Competition Vehicle Status."""


@dataclass(frozen=True)
class CompetitionVehicleStatus:
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
    front_steer_deg: float
    link_id: str
    tire_lateral_force: tuple
    side_slip_angle: tuple
    tire_cornering_stiffness: tuple


def _floats(packet, offset, count):
    return struct.unpack_from("<{}f".format(count), packet, offset)


def parse_competition_vehicle_status(packet):
    """Parse the observed 181-byte or extended 229-byte competition packet."""
    packet_size = len(packet)
    if packet_size not in SUPPORTED_LAYOUTS:
        raise CompetitionStatusPacketError(
            "expected 181 or 229 bytes, received {} (header={!r})".format(
                packet_size, packet[:15]
            )
        )
    if packet[:11] != PACKET_HEADER:
        raise CompetitionStatusPacketError(
            "unexpected header {!r}".format(packet[:11])
        )
    data_length = struct.unpack_from("<I", packet, 11)[0]
    expected_data_length = SUPPORTED_LAYOUTS[packet_size]
    if data_length != expected_data_length:
        raise CompetitionStatusPacketError(
            "expected data_length {}, received {}".format(
                expected_data_length, data_length
            )
        )
    if packet[-2:] != PACKET_TAIL:
        raise CompetitionStatusPacketError(
            "unexpected packet tail {!r}".format(packet[-2:])
        )

    seconds, nanoseconds = struct.unpack_from("<II", packet, 27)
    if nanoseconds >= 1_000_000_000:
        raise CompetitionStatusPacketError(
            "nanoseconds field is out of range: {}".format(nanoseconds)
        )

    ctrl_mode, gear = struct.unpack_from("<bb", packet, 35)
    signed_velocity_kmh = struct.unpack_from("<f", packet, 37)[0]
    map_data_id = struct.unpack_from("<i", packet, 41)[0]
    accel_pedal, brake_pedal = _floats(packet, 45, 2)
    size_m = _floats(packet, 53, 3)
    overhang_m, wheelbase_m, rear_overhang_m = _floats(packet, 65, 3)
    position_m = _floats(packet, 77, 3)
    rotation_deg = _floats(packet, 89, 3)
    velocity_kmh = _floats(packet, 101, 3)
    angular_velocity_degps = _floats(packet, 113, 3)
    acceleration_mps2 = _floats(packet, 125, 3)
    front_steer_deg = struct.unpack_from("<f", packet, 137)[0]
    link_id = (
        packet[141:179]
        .split(b"\x00", 1)[0]
        .decode("ascii", errors="replace")
        .strip()
    )
    if packet_size == EXTENDED_PACKET_SIZE:
        tire_lateral_force = _floats(packet, 179, 4)
        side_slip_angle = _floats(packet, 195, 4)
        tire_cornering_stiffness = _floats(packet, 211, 4)
    else:
        tire_lateral_force = ()
        side_slip_angle = ()
        tire_cornering_stiffness = ()

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
        front_steer_deg,
        *tire_lateral_force,
        *side_slip_angle,
        *tire_cornering_stiffness,
    )
    if not all(math.isfinite(value) for value in numeric_values):
        raise CompetitionStatusPacketError(
            "packet contains a non-finite numeric value"
        )

    return CompetitionVehicleStatus(
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
        front_steer_deg=front_steer_deg,
        link_id=link_id,
        tire_lateral_force=tire_lateral_force,
        side_slip_angle=side_slip_angle,
        tire_cornering_stiffness=tire_cornering_stiffness,
    )
