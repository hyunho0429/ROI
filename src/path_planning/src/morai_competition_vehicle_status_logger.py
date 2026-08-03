#!/usr/bin/env python3
"""Continuously print decoded MORAI Competition Vehicle Status over UDP."""

import argparse
import datetime
import math
import socket
import time

from path_planning.morai_competition_config import (
    BIND_IP,
    COMPETITION_STATUS_HOST_PORT,
    COMPETITION_STATUS_PORT,
)
from path_planning.morai_udp_competition_status import (
    CompetitionStatusPacketError,
    parse_competition_vehicle_status,
)


CTRL_MODE_NAMES = {2: "AV-ExternalCtrl"}
GEAR_NAMES = {4: "D"}


def argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Receive Competition Vehicle Status directly over UDP and print "
            "every decoded vehicle field in one snapshot. Stop other nodes "
            "that bind the same Destination Port before running this logger."
        )
    )
    parser.add_argument("--bind-ip", default=BIND_IP)
    parser.add_argument(
        "--host-port",
        type=int,
        default=COMPETITION_STATUS_HOST_PORT,
        help="expected MORAI UDP source/Host Port",
    )
    parser.add_argument(
        "--destination-port",
        type=int,
        default=COMPETITION_STATUS_PORT,
        help="local UDP Destination Port to bind",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="seconds between snapshots; 0 prints every packet",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="number of snapshots to print; 0 runs until Ctrl+C",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    return parser


def _validate_arguments(arguments):
    for port in (arguments.host_port, arguments.destination_port):
        if not 1 <= port <= 65535:
            raise ValueError("UDP ports must be between 1 and 65535")
    if arguments.interval < 0.0:
        raise ValueError("interval cannot be negative")
    if arguments.count < 0:
        raise ValueError("count cannot be negative")
    if arguments.timeout <= 0.0:
        raise ValueError("timeout must be positive")


def _name(value, names):
    return names.get(value, "unknown")


def _vector(values, precision=3):
    return "(" + ", ".join(
        ("{:.%df}" % precision).format(value) for value in values
    ) + ")"


def _magnitude(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def format_status_snapshot(status, sender, packet_size, snapshot_index, received_at):
    """Return one readable block containing every decoded status field."""
    packet_layout = "extended" if status.tire_lateral_force else "base"
    lines = [
        "=" * 78,
        "Competition Vehicle Status #{:06d} | received={} | sender={}:{}".format(
            snapshot_index,
            received_at,
            sender[0],
            sender[1],
        ),
        "packet     : {} bytes ({}) | simulator_timestamp={:.9f}s".format(
            packet_size,
            packet_layout,
            status.timestamp_sec,
        ),
        "[CONTROL]",
        "  ctrl_mode : {} ({})".format(
            status.ctrl_mode,
            _name(status.ctrl_mode, CTRL_MODE_NAMES),
        ),
        "  gear      : {} ({})".format(
            status.gear,
            _name(status.gear, GEAR_NAMES),
        ),
        "  pedals    : accel={:.3f}, brake={:.3f}".format(
            status.accel_pedal,
            status.brake_pedal,
        ),
        "  steering  : front={:.3f}deg".format(status.front_steer_deg),
        "[MOTION]",
        "  signed_speed : {:.3f}km/h ({:.3f}m/s)".format(
            status.signed_velocity_kmh,
            status.signed_velocity_kmh / 3.6,
        ),
        "  velocity     : xyz={}km/h, magnitude={:.3f}km/h".format(
            _vector(status.velocity_kmh),
            _magnitude(status.velocity_kmh),
        ),
        "  angular_vel  : xyz={}deg/s".format(
            _vector(status.angular_velocity_degps)
        ),
        "  acceleration : xyz={}m/s^2".format(
            _vector(status.acceleration_mps2)
        ),
        "[POSE / MAP]",
        "  position : xyz={}m".format(_vector(status.position_m)),
        "  rotation : xyz={}deg".format(_vector(status.rotation_deg)),
        "  map      : map_data_id={}, link_id={!r}".format(
            status.map_data_id,
            status.link_id,
        ),
        "[VEHICLE GEOMETRY]",
        "  size xyz       : {}m".format(_vector(status.size_m)),
        "  front_overhang : {:.3f}m".format(status.overhang_m),
        "  wheelbase      : {:.3f}m".format(status.wheelbase_m),
        "  rear_overhang  : {:.3f}m".format(status.rear_overhang_m),
        "[TIRES: packet order]",
    ]
    if status.tire_lateral_force:
        lines.extend(
            [
                "  lateral_force       : {}".format(
                    _vector(status.tire_lateral_force)
                ),
                "  side_slip_angle     : {}".format(
                    _vector(status.side_slip_angle)
                ),
                "  cornering_stiffness : {}".format(
                    _vector(status.tire_cornering_stiffness)
                ),
            ]
        )
    else:
        lines.append("  not present in the 181-byte base packet")
    lines.append("=" * 78)
    return "\n".join(lines)


def run(arguments):
    _validate_arguments(arguments)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    udp_socket.bind((arguments.bind_ip, arguments.destination_port))
    udp_socket.settimeout(arguments.timeout)
    print(
        "Waiting for Competition Vehicle Status: MORAI *:{} -> {}:{}; "
        "interval={:.3f}s, count={}".format(
            arguments.host_port,
            arguments.bind_ip,
            arguments.destination_port,
            arguments.interval,
            "infinite" if arguments.count == 0 else arguments.count,
        ),
        flush=True,
    )
    print(
        "Do not run this together with another process using UDP port {}.".format(
            arguments.destination_port
        ),
        flush=True,
    )

    snapshot_count = 0
    last_snapshot_at = None
    warned_source_ports = set()
    try:
        while arguments.count == 0 or snapshot_count < arguments.count:
            try:
                packet, sender = udp_socket.recvfrom(65535)
            except socket.timeout:
                print(
                    "TIMEOUT: no Competition Vehicle Status packet for {:.1f}s".format(
                        arguments.timeout
                    ),
                    flush=True,
                )
                return 2

            if sender[1] != arguments.host_port and sender[1] not in warned_source_ports:
                warned_source_ports.add(sender[1])
                print(
                    "WARNING: sender source port {} differs from expected Host Port {}".format(
                        sender[1],
                        arguments.host_port,
                    ),
                    flush=True,
                )

            try:
                status = parse_competition_vehicle_status(packet)
            except CompetitionStatusPacketError as error:
                print("INCOMPATIBLE PACKET: {}".format(error), flush=True)
                continue

            now = time.monotonic()
            if (
                last_snapshot_at is not None
                and now - last_snapshot_at < arguments.interval
            ):
                continue
            snapshot_count += 1
            last_snapshot_at = now
            received_at = datetime.datetime.now().astimezone().isoformat(
                timespec="milliseconds"
            )
            print(
                format_status_snapshot(
                    status,
                    sender,
                    len(packet),
                    snapshot_count,
                    received_at,
                ),
                flush=True,
            )
        return 0
    finally:
        udp_socket.close()


def main(argv=None):
    arguments = argument_parser().parse_args(argv)
    try:
        return run(arguments)
    except KeyboardInterrupt:
        print("\nStopped Competition Vehicle Status logger.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
