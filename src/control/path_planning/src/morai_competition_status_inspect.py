#!/usr/bin/env python3
"""Inspect raw Competition Vehicle Status UDP payloads without driving."""

import argparse
import socket
import struct

from path_planning.morai_competition_config import (
    BIND_IP,
    COMPETITION_STATUS_HOST_PORT,
    COMPETITION_STATUS_PORT,
)
from path_planning.morai_udp_competition_status import (
    CompetitionStatusPacketError,
    parse_competition_vehicle_status,
)


def argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Print sender, payload size, header, hex and decoded fields for "
            "Competition Vehicle Status UDP packets. Stop the driving node "
            "before binding the same Destination Port."
        )
    )
    parser.add_argument("--bind-ip", default=BIND_IP)
    parser.add_argument(
        "--host-port", type=int, default=COMPETITION_STATUS_HOST_PORT
    )
    parser.add_argument(
        "--destination-port", type=int, default=COMPETITION_STATUS_PORT
    )
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--hex-bytes",
        type=int,
        default=96,
        help="number of payload bytes to print as hexadecimal (0 prints all)",
    )
    return parser


def _hex(packet, limit):
    visible = packet if limit == 0 else packet[:limit]
    text = visible.hex(" ")
    if len(visible) < len(packet):
        text += " ..."
    return text


def _print_decoded(status):
    print(
        "  decoded: timestamp={:.9f}, ctrl_mode={}, gear={}, "
        "signed_velocity={:.3f} km/h, map_data_id={}".format(
            status.timestamp_sec,
            status.ctrl_mode,
            status.gear,
            status.signed_velocity_kmh,
            status.map_data_id,
        )
    )
    print(
        "           pedals=(accel={:.3f}, brake={:.3f}), "
        "front_steer={:.3f} deg, wheelbase={:.3f} m".format(
            status.accel_pedal,
            status.brake_pedal,
            status.front_steer_deg,
            status.wheelbase_m,
        )
    )
    print(
        "           size_m={}, overhangs_m=({:.3f}, {:.3f}), "
        "position_m={}, rotation_deg={}".format(
            status.size_m,
            status.overhang_m,
            status.rear_overhang_m,
            status.position_m,
            status.rotation_deg,
        )
    )
    print(
        "           velocity_kmh={}, angular_velocity_degps={}, "
        "acceleration_mps2={}, link_id={!r}".format(
            status.velocity_kmh,
            status.angular_velocity_degps,
            status.acceleration_mps2,
            status.link_id,
        )
    )
    if status.tire_lateral_force:
        print(
            "           tire_lateral_force={}, side_slip_angle={}, "
            "tire_cornering_stiffness={}".format(
                status.tire_lateral_force,
                status.side_slip_angle,
                status.tire_cornering_stiffness,
            )
        )


def run(arguments):
    for port in (arguments.host_port, arguments.destination_port):
        if not 1 <= port <= 65535:
            raise ValueError("UDP ports must be between 1 and 65535")
    if arguments.count < 1:
        raise ValueError("count must be at least 1")
    if arguments.timeout <= 0.0:
        raise ValueError("timeout must be positive")
    if arguments.hex_bytes < 0:
        raise ValueError("hex-bytes cannot be negative")

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((arguments.bind_ip, arguments.destination_port))
    udp_socket.settimeout(arguments.timeout)
    print(
        "Waiting for Competition Vehicle Status: MORAI host/source *:{} -> "
        "destination {}:{}".format(
            arguments.host_port,
            arguments.bind_ip,
            arguments.destination_port,
        )
    )
    try:
        for index in range(1, arguments.count + 1):
            try:
                packet, sender = udp_socket.recvfrom(65535)
            except socket.timeout:
                print("TIMEOUT: no packet received within {:.1f}s".format(arguments.timeout))
                return 2
            data_length = (
                struct.unpack_from("<I", packet, 11)[0]
                if len(packet) >= 15
                else None
            )
            print(
                "packet {}: sender={}:{}, payload={} bytes, header={!r}, "
                "data_length={}".format(
                    index,
                    sender[0],
                    sender[1],
                    len(packet),
                    packet[:11],
                    data_length,
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
                _print_decoded(parse_competition_vehicle_status(packet))
            except CompetitionStatusPacketError as error:
                print("  parser: INCOMPATIBLE ({})".format(error))
        return 0
    finally:
        udp_socket.close()


def main(argv=None):
    return run(argument_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
