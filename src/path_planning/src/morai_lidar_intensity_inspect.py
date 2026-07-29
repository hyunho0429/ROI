#!/usr/bin/env python3
"""Inspect MORAI 3D LiDAR Intensity UDP packets.

The inspector is intentionally read-only: it binds the algorithm-side
Destination Port and prints packet/point-cloud statistics without sending any
control command.
"""

import argparse
import csv
import os
import socket
import time

from path_planning.morai_competition_config import (
    BIND_IP,
    LIDAR_HOST_PORT,
    LIDAR_PORT,
)
from path_planning.morai_udp_lidar import (
    POINT_STRIDE_BYTES,
    LidarPacketError,
    infer_header_bytes,
    parse_lidar_intensity_packet,
    summarize_lidar_points,
)


def argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Print sender, payload size, decoded XYZI point count and intensity "
            "statistics for MORAI 3D LiDAR Intensity UDP packets."
        )
    )
    parser.add_argument("--bind-ip", default=BIND_IP)
    parser.add_argument("--host-port", type=int, default=LIDAR_HOST_PORT)
    parser.add_argument("--destination-port", type=int, default=LIDAR_PORT)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--header-bytes",
        default="auto",
        help=(
            "leading bytes before XYZI point data; use 0 for documented raw "
            "float32 point stream or auto to infer from packet size"
        ),
    )
    parser.add_argument(
        "--byte-order",
        choices=("little", "big"),
        default="little",
        help="float byte order used by the point data",
    )
    parser.add_argument(
        "--sample-points",
        type=int,
        default=5,
        help="number of decoded points to print from the beginning of each packet",
    )
    parser.add_argument(
        "--hex-bytes",
        type=int,
        default=64,
        help="number of raw payload bytes to print as hexadecimal (0 prints all)",
    )
    parser.add_argument(
        "--dump-csv",
        default=None,
        help="optional CSV file path to append decoded points for offline checks",
    )
    parser.add_argument(
        "--dump-max-points",
        type=int,
        default=20000,
        help="maximum number of points written to --dump-csv across all packets",
    )
    return parser


def _hex(packet, limit):
    visible = packet if limit == 0 else packet[:limit]
    text = visible.hex(" ")
    if len(visible) < len(packet):
        text += " ..."
    return text


def _parse_header_bytes(value, payload_size):
    if str(value).lower() == "auto":
        return infer_header_bytes(payload_size, POINT_STRIDE_BYTES)
    header_bytes = int(value)
    if header_bytes < 0:
        raise ValueError("header-bytes cannot be negative")
    return header_bytes


def _open_csv(path):
    if not path:
        return None, None
    output_path = os.path.abspath(os.path.expanduser(path))
    output_dir = os.path.dirname(output_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    stream = open(output_path, "w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        stream,
        fieldnames=[
            "packet_index",
            "point_index",
            "x_m",
            "y_m",
            "z_m",
            "intensity",
            "distance_m",
        ],
    )
    writer.writeheader()
    return stream, writer


def _dump_points(writer, packet_index, points, remaining_budget):
    if writer is None or remaining_budget <= 0:
        return 0
    written = 0
    for point_index, point in enumerate(points):
        if written >= remaining_budget:
            break
        writer.writerow(
            {
                "packet_index": packet_index,
                "point_index": point_index,
                "x_m": "{:.6f}".format(point.x_m),
                "y_m": "{:.6f}".format(point.y_m),
                "z_m": "{:.6f}".format(point.z_m),
                "intensity": "{:.6f}".format(point.intensity),
                "distance_m": "{:.6f}".format(point.distance_m),
            }
        )
        written += 1
    return written


def _print_summary(summary):
    print(
        "  points: count={}, finite={}, distance={}..{} m".format(
            summary["count"],
            summary["finite_count"],
            "n/a"
            if summary["distance_min_m"] is None
            else "{:.3f}".format(summary["distance_min_m"]),
            "n/a"
            if summary["distance_max_m"] is None
            else "{:.3f}".format(summary["distance_max_m"]),
        )
    )
    print(
        "          x={}, y={}, z={}, intensity={}".format(
            _range_text(summary["x_range_m"], "m"),
            _range_text(summary["y_range_m"], "m"),
            _range_text(summary["z_range_m"], "m"),
            _range_text(summary["intensity_range"], ""),
        )
    )


def _range_text(value_range, unit):
    if value_range is None:
        return "n/a"
    suffix = "" if not unit else " {}".format(unit)
    return "{:.3f}..{:.3f}{}".format(value_range[0], value_range[1], suffix)


def run(arguments):
    for port in (arguments.host_port, arguments.destination_port):
        if not 1 <= port <= 65535:
            raise ValueError("UDP ports must be between 1 and 65535")
    if arguments.count < 1:
        raise ValueError("count must be at least 1")
    if arguments.timeout <= 0.0:
        raise ValueError("timeout must be positive")
    if arguments.sample_points < 0:
        raise ValueError("sample-points cannot be negative")
    if arguments.hex_bytes < 0:
        raise ValueError("hex-bytes cannot be negative")
    if arguments.dump_max_points < 0:
        raise ValueError("dump-max-points cannot be negative")

    byte_order = "<" if arguments.byte_order == "little" else ">"
    csv_stream, csv_writer = _open_csv(arguments.dump_csv)
    dumped_points = 0

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    udp_socket.bind((arguments.bind_ip, arguments.destination_port))
    udp_socket.settimeout(arguments.timeout)
    print(
        "Waiting for 3D LiDAR Intensity UDP: MORAI host/source *:{} -> "
        "destination {}:{} (XYZI float32, {} endian)".format(
            arguments.host_port,
            arguments.bind_ip,
            arguments.destination_port,
            arguments.byte_order,
        )
    )
    try:
        for packet_index in range(1, arguments.count + 1):
            try:
                packet, sender = udp_socket.recvfrom(65535)
            except socket.timeout:
                print("TIMEOUT: no packet received within {:.1f}s".format(arguments.timeout))
                return 2
            header_bytes = _parse_header_bytes(arguments.header_bytes, len(packet))
            point_payload_size = max(0, len(packet) - header_bytes)
            expected_points = (
                point_payload_size // POINT_STRIDE_BYTES
                if point_payload_size % POINT_STRIDE_BYTES == 0
                else None
            )
            print(
                "packet {}: sender={}:{}, payload={} bytes, header_bytes={}, "
                "point_payload={} bytes, expected_points={}".format(
                    packet_index,
                    sender[0],
                    sender[1],
                    len(packet),
                    header_bytes,
                    point_payload_size,
                    "n/a" if expected_points is None else expected_points,
                )
            )
            if sender[1] != arguments.host_port:
                print(
                    "  WARNING: sender source port {}, expected Host Port {}".format(
                        sender[1], arguments.host_port
                    )
                )
            print("  hex: {}".format(_hex(packet, arguments.hex_bytes)))
            try:
                lidar_packet = parse_lidar_intensity_packet(
                    packet,
                    header_bytes=header_bytes,
                    byte_order=byte_order,
                )
            except (LidarPacketError, ValueError) as error:
                print("  parser: INCOMPATIBLE ({})".format(error))
                continue

            summary = summarize_lidar_points(lidar_packet.points)
            _print_summary(summary)
            for point_index, point in enumerate(
                lidar_packet.points[: arguments.sample_points]
            ):
                print(
                    "  sample[{}]: x={:+.3f}m, y={:+.3f}m, z={:+.3f}m, "
                    "intensity={:.3f}, distance={:.3f}m".format(
                        point_index,
                        point.x_m,
                        point.y_m,
                        point.z_m,
                        point.intensity,
                        point.distance_m,
                    )
                )
            dumped_points += _dump_points(
                csv_writer,
                packet_index,
                lidar_packet.points,
                arguments.dump_max_points - dumped_points,
            )
            if csv_stream is not None:
                csv_stream.flush()
        if csv_stream is not None:
            print(
                "CSV dump complete: {} points -> {}".format(
                    dumped_points, os.path.abspath(arguments.dump_csv)
                )
            )
        return 0
    finally:
        udp_socket.close()
        if csv_stream is not None:
            csv_stream.close()


def main(argv=None):
    return run(argument_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())

