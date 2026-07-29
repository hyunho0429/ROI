#!/usr/bin/env python3
"""MORAI 3D LiDAR UDP point cloud helpers.

MORAI's sensor documentation describes intensity point clouds as repeated
4-byte values for ``x, y, z, intensity``.  The UDP stream is therefore handled
as a flat sequence of float32 XYZI points by default.
"""

import math
import struct
from dataclasses import dataclass


POINT_FIELD_COUNT = 4
FLOAT32_SIZE = 4
POINT_STRIDE_BYTES = POINT_FIELD_COUNT * FLOAT32_SIZE


class LidarPacketError(ValueError):
    """Raised when a LiDAR UDP payload cannot be decoded as XYZI points."""


@dataclass(frozen=True)
class LidarPoint:
    x_m: float
    y_m: float
    z_m: float
    intensity: float

    @property
    def distance_m(self):
        return math.sqrt(self.x_m * self.x_m + self.y_m * self.y_m + self.z_m * self.z_m)


@dataclass(frozen=True)
class LidarPacket:
    points: tuple
    header_bytes: bytes
    point_stride_bytes: int
    byte_order: str


def infer_header_bytes(payload_size, point_stride_bytes=POINT_STRIDE_BYTES):
    """Infer a plausible header length from packet length.

    Official sensor-data docs describe the point data itself, not a separate
    UDP header.  For the common no-header case this returns 0.  If the payload
    is not divisible by the point stride, the smallest leading byte count that
    makes the remaining data divisible is returned so the inspect tool can
    still decode and report the packet.
    """
    if point_stride_bytes <= 0:
        raise ValueError("point_stride_bytes must be positive")
    remainder = payload_size % point_stride_bytes
    return 0 if remainder == 0 else remainder


def parse_lidar_intensity_packet(
    packet,
    header_bytes=0,
    point_stride_bytes=POINT_STRIDE_BYTES,
    byte_order="<",
    max_points=None,
):
    """Decode a MORAI intensity LiDAR UDP payload.

    Args:
        packet: Raw UDP payload bytes.
        header_bytes: Number of leading bytes to keep as header and skip when
            decoding point data.
        point_stride_bytes: Bytes per point. MORAI intensity XYZI is 16 bytes.
        byte_order: ``"<"`` for little endian, ``">"`` for big endian.
        max_points: Optional cap for decoded points.
    """
    if byte_order not in ("<", ">"):
        raise ValueError("byte_order must be '<' or '>'")
    if header_bytes < 0:
        raise ValueError("header_bytes cannot be negative")
    if point_stride_bytes != POINT_STRIDE_BYTES:
        raise ValueError("only 16-byte float32 XYZI points are supported")
    if len(packet) < header_bytes:
        raise LidarPacketError(
            "payload is shorter than header_bytes: {} < {}".format(
                len(packet), header_bytes
            )
        )
    point_bytes = packet[header_bytes:]
    if len(point_bytes) % point_stride_bytes:
        raise LidarPacketError(
            "point payload size {} is not divisible by {} bytes/point".format(
                len(point_bytes), point_stride_bytes
            )
        )
    point_count = len(point_bytes) // point_stride_bytes
    if max_points is not None:
        point_count = min(point_count, int(max_points))
    fmt = byte_order + "ffff"
    points = []
    for index in range(point_count):
        offset = header_bytes + index * point_stride_bytes
        x_m, y_m, z_m, intensity = struct.unpack_from(fmt, packet, offset)
        points.append(LidarPoint(x_m, y_m, z_m, intensity))
    return LidarPacket(
        points=tuple(points),
        header_bytes=packet[:header_bytes],
        point_stride_bytes=point_stride_bytes,
        byte_order=byte_order,
    )


def summarize_lidar_points(points):
    """Return simple range/intensity statistics for decoded points."""
    if not points:
        return {
            "count": 0,
            "finite_count": 0,
            "distance_min_m": None,
            "distance_max_m": None,
            "x_range_m": None,
            "y_range_m": None,
            "z_range_m": None,
            "intensity_range": None,
        }
    finite_points = [
        point
        for point in points
        if all(
            math.isfinite(value)
            for value in (point.x_m, point.y_m, point.z_m, point.intensity)
        )
    ]
    if not finite_points:
        return {
            "count": len(points),
            "finite_count": 0,
            "distance_min_m": None,
            "distance_max_m": None,
            "x_range_m": None,
            "y_range_m": None,
            "z_range_m": None,
            "intensity_range": None,
        }
    distances = [point.distance_m for point in finite_points]
    return {
        "count": len(points),
        "finite_count": len(finite_points),
        "distance_min_m": min(distances),
        "distance_max_m": max(distances),
        "x_range_m": (
            min(point.x_m for point in finite_points),
            max(point.x_m for point in finite_points),
        ),
        "y_range_m": (
            min(point.y_m for point in finite_points),
            max(point.y_m for point in finite_points),
        ),
        "z_range_m": (
            min(point.z_m for point in finite_points),
            max(point.z_m for point in finite_points),
        ),
        "intensity_range": (
            min(point.intensity for point in finite_points),
            max(point.intensity for point in finite_points),
        ),
    }

