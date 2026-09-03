#!/usr/bin/env python3
"""MORAI 3D LiDAR UDP packet helpers.

MORAI's UDP 3D LiDAR stream follows the Velodyne packet layout shown in the
official sensor documentation:

* 12 data blocks x 100 bytes = 1200 bytes
* 4 byte timestamp + 2 byte factory/status = 6 bytes

The 42 byte UDP/IP/Ethernet header in the documentation is not normally
returned by Python's ``socket.recvfrom()``.  Most user-space UDP receivers see
the 1206 byte payload beginning with the Velodyne block flag ``0xffee``.
"""

import math
import struct
from dataclasses import dataclass


VELODYNE_BLOCK_FLAG = b"\xff\xee"
VELODYNE_BLOCK_BYTES = 100
VELODYNE_BLOCK_COUNT = 12
VELODYNE_CHANNELS_PER_BLOCK = 32
VELODYNE_CHANNEL_BYTES = 3
VELODYNE_TAIL_BYTES = 6
VELODYNE_PAYLOAD_BYTES = (
    VELODYNE_BLOCK_BYTES * VELODYNE_BLOCK_COUNT + VELODYNE_TAIL_BYTES
)
OPTIONAL_NETWORK_HEADER_BYTES = 42

DISTANCE_RESOLUTION_M = 0.002

# VLP-16 channel vertical angles in Velodyne firing order.
VLP16_VERTICAL_ANGLES_DEG = (
    -15.0,
    1.0,
    -13.0,
    3.0,
    -11.0,
    5.0,
    -9.0,
    7.0,
    -7.0,
    9.0,
    -5.0,
    11.0,
    -3.0,
    13.0,
    -1.0,
    15.0,
)

# Backward-compatible aliases used by the inspector.
POINT_STRIDE_BYTES = VELODYNE_CHANNEL_BYTES


class LidarPacketError(ValueError):
    """Raised when a LiDAR UDP payload cannot be decoded."""


@dataclass(frozen=True)
class LidarPoint:
    # Vehicle/local coordinates used by the inspector and obstacle filters.
    # x: forward, y: left, z: up.
    x_m: float
    y_m: float
    z_m: float

    # Raw MORAI/Velodyne coordinates from the official LiDAR sensor frame.
    # x: right, y: forward, z: up.
    raw_x_right_m: float
    raw_y_forward_m: float
    raw_z_up_m: float

    intensity: int
    distance_m: float
    azimuth_deg: float
    vertical_angle_deg: float
    laser_id: int
    block_index: int
    channel_index: int

    @property
    def range_m(self):
        return self.distance_m


@dataclass(frozen=True)
class LidarPacket:
    points: tuple
    header_bytes: bytes
    tail_bytes: bytes
    timestamp_us: int
    factory_bytes: bytes
    payload_offset_bytes: int
    point_stride_bytes: int
    byte_order: str


def _normalize_payload(packet, header_bytes="auto"):
    """Return ``(payload, skipped_header)`` for a Velodyne payload.

    ``socket.recvfrom()`` normally returns 1206 bytes starting at ``0xffee``.
    Some packet captures or tools may include the 42 byte link/IP/UDP header,
    so auto mode strips it if byte 42 starts with ``0xffee``.
    """
    if header_bytes == "auto":
        if len(packet) == VELODYNE_PAYLOAD_BYTES and packet[:2] == VELODYNE_BLOCK_FLAG:
            return packet, b""
        if (
            len(packet) == OPTIONAL_NETWORK_HEADER_BYTES + VELODYNE_PAYLOAD_BYTES
            and packet[OPTIONAL_NETWORK_HEADER_BYTES : OPTIONAL_NETWORK_HEADER_BYTES + 2]
            == VELODYNE_BLOCK_FLAG
        ):
            return packet[OPTIONAL_NETWORK_HEADER_BYTES:], packet[:OPTIONAL_NETWORK_HEADER_BYTES]
        if len(packet) >= 2 and packet[:2] == VELODYNE_BLOCK_FLAG:
            return packet, b""
        if len(packet) > OPTIONAL_NETWORK_HEADER_BYTES and packet[
            OPTIONAL_NETWORK_HEADER_BYTES : OPTIONAL_NETWORK_HEADER_BYTES + 2
        ] == VELODYNE_BLOCK_FLAG:
            return packet[OPTIONAL_NETWORK_HEADER_BYTES:], packet[:OPTIONAL_NETWORK_HEADER_BYTES]
        raise LidarPacketError(
            "cannot find Velodyne block flag 0xffee at byte 0 or byte 42"
        )

    header_count = int(header_bytes)
    if header_count < 0:
        raise ValueError("header_bytes cannot be negative")
    if len(packet) < header_count:
        raise LidarPacketError(
            "payload is shorter than header_bytes: {} < {}".format(
                len(packet), header_count
            )
        )
    return packet[header_count:], packet[:header_count]


def infer_header_bytes(payload_size, point_stride_bytes=POINT_STRIDE_BYTES):
    """Infer likely user-space header size.

    For MORAI LiDAR UDP received by Python, this should usually be 0 because
    the application payload itself is 1206 bytes.  If a full 1248 byte packet
    capture is passed in, 42 is returned.
    """
    if payload_size == VELODYNE_PAYLOAD_BYTES:
        return 0
    if payload_size == OPTIONAL_NETWORK_HEADER_BYTES + VELODYNE_PAYLOAD_BYTES:
        return OPTIONAL_NETWORK_HEADER_BYTES
    return "auto"


def _azimuth_delta_deg(current, next_value):
    delta = next_value - current
    if delta < 0.0:
        delta += 360.0
    return delta


def should_publish_cloud(
    previous_azimuth_deg,
    current_azimuth_deg,
    buffered_packet_count,
    cloud_age_s,
    packets_per_cloud,
    max_cloud_age_s,
):
    """Return whether buffered packets should be emitted as a new cloud.

    Detect the 360 -> 0 boundary using a large negative jump instead of a
    narrow angle window. Packet-count and age limits keep RViz updating even
    when packet loss skips the boundary or the stream uses unusual azimuths.
    """

    if int(buffered_packet_count) <= 0:
        return False

    wrapped = (
        previous_azimuth_deg is not None
        and float(previous_azimuth_deg) - float(current_azimuth_deg) > 180.0
    )
    packet_limit_reached = int(buffered_packet_count) >= int(packets_per_cloud)
    age_limit_reached = (
        float(max_cloud_age_s) > 0.0
        and float(cloud_age_s) >= float(max_cloud_age_s)
    )
    return wrapped or packet_limit_reached or age_limit_reached


def _point_from_polar(distance_m, azimuth_deg, vertical_angle_deg):
    azimuth_rad = math.radians(azimuth_deg)
    vertical_rad = math.radians(vertical_angle_deg)
    xy_distance = distance_m * math.cos(vertical_rad)

    # Official MORAI Velodyne LiDAR frame:
    #   +x: right, +y: forward, +z: up
    #
    # Velodyne azimuth 0 deg points along +y(forward), and positive azimuth
    # rotates toward +x(right).  This keeps raw coordinates consistent with
    # the UDP LiDAR sensor frame documented by MORAI.
    raw_x_right_m = xy_distance * math.sin(azimuth_rad)
    raw_y_forward_m = xy_distance * math.cos(azimuth_rad)
    raw_z_up_m = distance_m * math.sin(vertical_rad)

    # Vehicle/obstacle-detection frame used by the rest of this debug tool:
    #   +x: forward, +y: left, +z: up
    vehicle_x_forward_m = raw_y_forward_m
    vehicle_y_left_m = -raw_x_right_m
    vehicle_z_up_m = raw_z_up_m
    return (
        vehicle_x_forward_m,
        vehicle_y_left_m,
        vehicle_z_up_m,
        raw_x_right_m,
        raw_y_forward_m,
        raw_z_up_m,
    )


def parse_lidar_intensity_packet(
    packet,
    header_bytes="auto",
    point_stride_bytes=POINT_STRIDE_BYTES,
    byte_order="<",
    max_points=None,
):
    """Decode a MORAI/Velodyne 3D LiDAR UDP payload.

    Args:
        packet: Raw UDP payload bytes.
        header_bytes: ``"auto"``, ``0`` for normal UDP payloads, or ``42`` for
            full packet captures containing link/IP/UDP headers.
        point_stride_bytes: Kept for backward compatibility. Velodyne channel
            data is always 3 bytes: uint16 distance + uint8 reflectivity.
        byte_order: ``"<"`` or ``">"``. Velodyne UDP packets are little endian.
        max_points: Optional cap for decoded measurements.
    """
    if byte_order not in ("<", ">"):
        raise ValueError("byte_order must be '<' or '>'")
    if point_stride_bytes != VELODYNE_CHANNEL_BYTES:
        raise ValueError("Velodyne channel stride must be 3 bytes")

    payload, skipped_header = _normalize_payload(packet, header_bytes)
    if len(payload) != VELODYNE_PAYLOAD_BYTES:
        raise LidarPacketError(
            "expected {} byte Velodyne payload, got {} bytes".format(
                VELODYNE_PAYLOAD_BYTES, len(payload)
            )
        )

    block_azimuths = []
    for block_index in range(VELODYNE_BLOCK_COUNT):
        block_offset = block_index * VELODYNE_BLOCK_BYTES
        if payload[block_offset : block_offset + 2] != VELODYNE_BLOCK_FLAG:
            raise LidarPacketError(
                "invalid block flag at block {} byte {}".format(
                    block_index, block_offset
                )
            )
        azimuth_raw = struct.unpack_from(byte_order + "H", payload, block_offset + 2)[0]
        block_azimuths.append(azimuth_raw * 0.01)

    points = []
    for block_index in range(VELODYNE_BLOCK_COUNT):
        block_offset = block_index * VELODYNE_BLOCK_BYTES
        current_azimuth = block_azimuths[block_index]
        if block_index < VELODYNE_BLOCK_COUNT - 1:
            next_azimuth = block_azimuths[block_index + 1]
            azimuth_step = _azimuth_delta_deg(current_azimuth, next_azimuth)
        elif block_index > 0:
            azimuth_step = _azimuth_delta_deg(
                block_azimuths[block_index - 1], current_azimuth
            )
        else:
            azimuth_step = 0.0

        for channel_index in range(VELODYNE_CHANNELS_PER_BLOCK):
            data_offset = block_offset + 4 + channel_index * VELODYNE_CHANNEL_BYTES
            distance_raw = struct.unpack_from(byte_order + "H", payload, data_offset)[0]
            intensity = payload[data_offset + 2]
            distance_m = distance_raw * DISTANCE_RESOLUTION_M

            laser_id = channel_index % len(VLP16_VERTICAL_ANGLES_DEG)
            firing_group = channel_index // len(VLP16_VERTICAL_ANGLES_DEG)
            azimuth_deg = (current_azimuth + azimuth_step * 0.5 * firing_group) % 360.0
            vertical_angle_deg = VLP16_VERTICAL_ANGLES_DEG[laser_id]
            (
                x_m,
                y_m,
                z_m,
                raw_x_right_m,
                raw_y_forward_m,
                raw_z_up_m,
            ) = _point_from_polar(
                distance_m, azimuth_deg, vertical_angle_deg
            )

            points.append(
                LidarPoint(
                    x_m=x_m,
                    y_m=y_m,
                    z_m=z_m,
                    raw_x_right_m=raw_x_right_m,
                    raw_y_forward_m=raw_y_forward_m,
                    raw_z_up_m=raw_z_up_m,
                    intensity=intensity,
                    distance_m=distance_m,
                    azimuth_deg=azimuth_deg,
                    vertical_angle_deg=vertical_angle_deg,
                    laser_id=laser_id,
                    block_index=block_index,
                    channel_index=channel_index,
                )
            )
            if max_points is not None and len(points) >= int(max_points):
                break
        if max_points is not None and len(points) >= int(max_points):
            break

    tail_offset = VELODYNE_BLOCK_BYTES * VELODYNE_BLOCK_COUNT
    tail = payload[tail_offset:]
    timestamp_us = struct.unpack_from(byte_order + "I", tail, 0)[0]
    factory = tail[4:6]

    return LidarPacket(
        points=tuple(points),
        header_bytes=skipped_header,
        tail_bytes=tail,
        timestamp_us=timestamp_us,
        factory_bytes=factory,
        payload_offset_bytes=len(skipped_header),
        point_stride_bytes=VELODYNE_CHANNEL_BYTES,
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
