"""Competition-safe MORAI Ego Ctrl Cmd UDP encoder."""

import math
import struct
from dataclasses import dataclass


PACKET_HEADER = b"#MoraiCtrlCmd$"
PACKET_DATA_LENGTH = 27
PACKET_SIZE = 59
PACKET_TAIL = b"\r\n"


@dataclass(frozen=True)
class EgoCtrlCommand26R1:
    """26.R1 competition command; only longCmdType 1 is permitted."""

    ctrl_mode: int = 2
    gear: int = 4
    long_cmd_type: int = 1
    velocity_kmh: float = 0.0
    acceleration_mps2: float = 0.0
    accel: float = 0.0
    brake: float = 0.0
    steering_normalized: float = 0.0
    rear_steering_normalized: float = 0.0


def encode_ego_ctrl_cmd_26r1(command):
    """Encode one exact 59-byte 26.R1 command and enforce throttle mode."""
    if command.ctrl_mode not in (1, 2):
        raise ValueError("ctrl_mode must be 1 (keyboard) or 2 (auto)")
    if command.gear not in range(6):
        raise ValueError("gear must be between 0 and 5")
    if command.long_cmd_type != 1:
        raise ValueError("competition rules require long_cmd_type 1 (accel/brake)")

    values = (
        command.velocity_kmh,
        command.acceleration_mps2,
        command.accel,
        command.brake,
        command.steering_normalized,
        command.rear_steering_normalized,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("control command contains a non-finite value")
    if not 0.0 <= command.accel <= 1.0:
        raise ValueError("accel must be between 0 and 1")
    if not 0.0 <= command.brake <= 1.0:
        raise ValueError("brake must be between 0 and 1")
    if not -1.0 <= command.steering_normalized <= 1.0:
        raise ValueError("steering_normalized must be between -1 and 1")
    if not -1.0 <= command.rear_steering_normalized <= 1.0:
        raise ValueError("rear_steering_normalized must be between -1 and 1")

    packet = bytearray(PACKET_SIZE)
    packet[: len(PACKET_HEADER)] = PACKET_HEADER
    struct.pack_into("<I", packet, 14, PACKET_DATA_LENGTH)
    # bytes 18:30 are the documented 12-byte auxiliary field and remain zero.
    struct.pack_into(
        "<BBBffffff",
        packet,
        30,
        command.ctrl_mode,
        command.gear,
        command.long_cmd_type,
        *values,
    )
    packet[-2:] = PACKET_TAIL
    return bytes(packet)


def pedal_command(accel, brake, steering_normalized):
    """Build a legal longCmdType-1 accel/brake/steering command."""
    return EgoCtrlCommand26R1(
        accel=max(0.0, min(1.0, float(accel))),
        brake=max(0.0, min(1.0, float(brake))),
        steering_normalized=max(-1.0, min(1.0, float(steering_normalized))),
    )


def brake_command(brake=1.0):
    """Build a legal command that explicitly applies the brake."""
    return EgoCtrlCommand26R1(brake=max(0.0, min(1.0, float(brake))))
