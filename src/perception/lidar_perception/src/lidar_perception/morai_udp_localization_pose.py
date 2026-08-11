"""Compact UDP transport for sharing the driving EKF pose with LiDAR.

The Stanley runtime remains the sole owner of the MORAI GPS and IMU receive
ports.  It republishes only its fused pose on a separate UDP port so the LiDAR
process can deskew scans without competing for sensor datagrams.
"""

import math
import struct
from dataclasses import dataclass


PACKET_MAGIC = b"ROIPOSE1"
PACKET_STRUCT = struct.Struct("<8s7d")
PACKET_SIZE = PACKET_STRUCT.size


class LocalizationPosePacketError(ValueError):
    """Raised when an internal EKF pose datagram is invalid."""


@dataclass(frozen=True)
class LocalizationPose:
    timestamp_monotonic_s: float
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    speed_mps: float
    yaw_rate_radps: float


def encode_localization_pose(pose):
    values = (
        float(pose.timestamp_monotonic_s),
        float(pose.x_m),
        float(pose.y_m),
        float(pose.z_m),
        float(pose.yaw_rad),
        float(pose.speed_mps),
        float(pose.yaw_rate_radps),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("localization pose contains a non-finite value")
    return PACKET_STRUCT.pack(PACKET_MAGIC, *values)


def decode_localization_pose(packet):
    if len(packet) != PACKET_SIZE:
        raise LocalizationPosePacketError(
            "expected {} byte localization pose packet, got {}".format(
                PACKET_SIZE, len(packet)
            )
        )
    magic, *values = PACKET_STRUCT.unpack(packet)
    if magic != PACKET_MAGIC:
        raise LocalizationPosePacketError(
            "unexpected localization pose magic {!r}".format(magic)
        )
    if not all(math.isfinite(value) for value in values):
        raise LocalizationPosePacketError(
            "localization pose packet contains a non-finite value"
        )
    return LocalizationPose(*values)
