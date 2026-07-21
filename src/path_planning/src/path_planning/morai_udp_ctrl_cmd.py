"""Competition-safe MORAI Ego Ctrl Cmd UDP encoder."""

import math
import struct
from dataclasses import dataclass


PACKET_HEADER = b"#MoraiCtrlCmd$"
CONTROL_PROTOCOL_25S4 = "25s4"
CONTROL_PROTOCOL_26R1 = "26r1"
CONTROL_PROTOCOLS = (CONTROL_PROTOCOL_25S4, CONTROL_PROTOCOL_26R1)

# MORAI 25.S4 follows the public 23/24-series command: three one-byte mode
# fields and five floats.  The 26.R1 example added a rear-steer float.
PACKET_DATA_LENGTH = 23
PACKET_SIZE = 55
PACKET_DATA_LENGTH_26R1 = 27
PACKET_SIZE_26R1 = 59
PACKET_TAIL = b"\r\n"
KEYBOARD_CTRL_MODE = 1
EXTERNAL_CTRL_MODE = 2
DEFAULT_CTRL_MODE = KEYBOARD_CTRL_MODE
DRIVE_GEAR = 4


@dataclass(frozen=True)
class EgoCtrlCommand:
    """MORAI control command; competition use is limited to longCmdType 1."""

    ctrl_mode: int = DEFAULT_CTRL_MODE
    gear: int = DRIVE_GEAR
    long_cmd_type: int = 1
    velocity_kmh: float = 0.0
    acceleration_mps2: float = 0.0
    accel: float = 0.0
    brake: float = 0.0
    steering_normalized: float = 0.0
    rear_steering_normalized: float = 0.0

    # beta_drive morai_msgs/CtrlCmd-compatible field names.  ctrl_mode and
    # gear belong to the UDP envelope and are intentionally kept separate.
    @property
    def longlCmdType(self):
        return self.long_cmd_type

    @property
    def velocity(self):
        return self.velocity_kmh

    @property
    def acceleration(self):
        return self.acceleration_mps2

    @property
    def steering(self):
        return self.steering_normalized


def _validated_values(command):
    if command.ctrl_mode not in (1, 2):
        raise ValueError("ctrl_mode must be 1 or 2")
    if command.gear not in range(6):
        raise ValueError("gear must be between 0 and 5")
    if command.longlCmdType != 1:
        raise ValueError("competition rules require long_cmd_type 1 (accel/brake)")

    values = (
        command.velocity,
        command.acceleration,
        command.accel,
        command.brake,
        command.steering,
        command.rear_steering_normalized,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("control command contains a non-finite value")
    if not 0.0 <= command.accel <= 1.0:
        raise ValueError("accel must be between 0 and 1")
    if not 0.0 <= command.brake <= 1.0:
        raise ValueError("brake must be between 0 and 1")
    if not -1.0 <= command.steering <= 1.0:
        raise ValueError("steering_normalized must be between -1 and 1")
    if not -1.0 <= command.rear_steering_normalized <= 1.0:
        raise ValueError("rear_steering_normalized must be between -1 and 1")
    return values


def encode_ego_ctrl_cmd(command, protocol=CONTROL_PROTOCOL_25S4):
    """Encode a versioned Ego Ctrl Cmd packet, defaulting to MORAI 25.S4."""
    values = _validated_values(command)
    if protocol == CONTROL_PROTOCOL_25S4:
        data_length = PACKET_DATA_LENGTH
        packet_size = PACKET_SIZE
        payload_format = "<BBBfffff"
        payload_values = values[:-1]
    elif protocol == CONTROL_PROTOCOL_26R1:
        data_length = PACKET_DATA_LENGTH_26R1
        packet_size = PACKET_SIZE_26R1
        payload_format = "<BBBffffff"
        payload_values = values
    else:
        raise ValueError(
            "control protocol must be one of {}".format(", ".join(CONTROL_PROTOCOLS))
        )

    packet = bytearray(packet_size)
    packet[: len(PACKET_HEADER)] = PACKET_HEADER
    struct.pack_into("<I", packet, 14, data_length)
    # bytes 18:30 are the documented 12-byte auxiliary field and remain zero.
    struct.pack_into(
        payload_format,
        packet,
        30,
        command.ctrl_mode,
        command.gear,
        command.longlCmdType,
        *payload_values,
    )
    packet[-2:] = PACKET_TAIL
    return bytes(packet)


def encode_ego_ctrl_cmd_25s4(command):
    """Encode the 55-byte MORAI 25.S4/public 23/24-series packet."""
    return encode_ego_ctrl_cmd(command, CONTROL_PROTOCOL_25S4)


def encode_ego_ctrl_cmd_26r1(command):
    """Encode the optional 59-byte 26.R1 packet with rear steering."""
    return encode_ego_ctrl_cmd(command, CONTROL_PROTOCOL_26R1)


def pedal_command(
    accel,
    brake,
    steering_normalized,
    ctrl_mode=DEFAULT_CTRL_MODE,
):
    """Build a legal longCmdType-1 accel/brake/steering command."""
    return EgoCtrlCommand(
        ctrl_mode=int(ctrl_mode),
        accel=max(0.0, min(1.0, float(accel))),
        brake=max(0.0, min(1.0, float(brake))),
        steering_normalized=max(-1.0, min(1.0, float(steering_normalized))),
    )


def brake_command(brake=1.0, ctrl_mode=DEFAULT_CTRL_MODE):
    """Build a legal command that explicitly applies the brake."""
    return EgoCtrlCommand(
        ctrl_mode=int(ctrl_mode),
        brake=max(0.0, min(1.0, float(brake))),
    )


def external_control_ready(
    ctrl_mode,
    gear,
    required_ctrl_mode=DEFAULT_CTRL_MODE,
):
    """Return whether status confirms the configured mode and Drive gear."""
    return ctrl_mode == required_ctrl_mode and gear == DRIVE_GEAR


# Keep the old public name import-compatible with earlier dev/stanley commits.
EgoCtrlCommand26R1 = EgoCtrlCommand
