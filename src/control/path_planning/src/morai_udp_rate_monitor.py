#!/usr/bin/env python3
"""Monitor MORAI UDP receive/transmit timing.

This utility is intentionally independent from the driving controller.  Use it
when the simulator appears connected but the vehicle does not move, sensors go
stale, or command feedback does not follow the command.

Default receive channels:
  * GPS destination 3001
  * IMU destination 4001
  * Competition Vehicle Status source 9080 -> destination 9081
  * CollisionData source 9091 -> destination 9092
  * 3D LiDAR Intensity source 2000 -> destination 2001
  * Camera sensor source/destination ports supplied by launch/CLI

Optional transmit check:
  * Ego Ctrl Cmd source 9094 -> MORAI host 9093

Run this instead of the main controller when checking receive ports.  Binding a
second process to the same UDP destination port can hide the real issue.
"""

import argparse
import math
import selectors
import socket
import sys
import time
from dataclasses import dataclass

from path_planning.morai_competition_config import (
    BIND_IP,
    COLLISION_HOST_PORT,
    COLLISION_PORT,
    COMPETITION_STATUS_HOST_PORT,
    COMPETITION_STATUS_PORT,
    CONTROL_DESTINATION_PORT,
    CONTROL_IP,
    CONTROL_PORT,
    GPS_PORT,
    IMU_PORT,
    LIDAR_HOST_PORT,
    LIDAR_PORT,
)
from path_planning.morai_udp_competition_status import (
    CompetitionStatusPacketError,
    parse_competition_vehicle_status,
)
from path_planning.morai_udp_ctrl_cmd import (
    CONTROL_PROTOCOL_25S4,
    CONTROL_PROTOCOLS,
    encode_ego_ctrl_cmd,
    pedal_command,
)


CAMERA_HOST_PORT = 0
CAMERA_PORT = 0


@dataclass(frozen=True)
class ReceiveChannel:
    name: str
    destination_port: int
    expected_source_port: int = None


class TimingStats:
    def __init__(self):
        self.count = 0
        self.bytes = 0
        self.first_time = None
        self.last_time = None
        self.last_dt = None
        self.min_dt = None
        self.max_dt = None
        self._mean_dt = 0.0
        self._m2_dt = 0.0

    def update(self, timestamp, byte_count=0):
        timestamp = float(timestamp)
        if self.first_time is None:
            self.first_time = timestamp
        if self.last_time is not None:
            dt = max(0.0, timestamp - self.last_time)
            self.last_dt = dt
            self.min_dt = dt if self.min_dt is None else min(self.min_dt, dt)
            self.max_dt = dt if self.max_dt is None else max(self.max_dt, dt)
            interval_index = self.count
            delta = dt - self._mean_dt
            self._mean_dt += delta / interval_index
            self._m2_dt += delta * (dt - self._mean_dt)
        self.last_time = timestamp
        self.count += 1
        self.bytes += int(byte_count)

    @property
    def interval_count(self):
        return max(0, self.count - 1)

    @property
    def mean_dt(self):
        return self._mean_dt if self.interval_count > 0 else None

    @property
    def std_dt(self):
        if self.interval_count <= 1:
            return None
        return math.sqrt(self._m2_dt / (self.interval_count - 1))

    @property
    def hz(self):
        if self.mean_dt is None or self.mean_dt <= 0.0:
            return None
        return 1.0 / self.mean_dt


class ChannelState:
    def __init__(self, channel):
        self.channel = channel
        self.stats = TimingStats()
        self.source_mismatch_count = 0
        self.invalid_packet_count = 0
        self.last_sender = None
        self.last_payload_size = 0
        self.last_note = ""

    def update_packet(self, packet, sender, now):
        self.stats.update(now, len(packet))
        self.last_sender = sender
        self.last_payload_size = len(packet)
        expected = self.channel.expected_source_port
        if expected is not None and sender[1] != expected:
            self.source_mismatch_count += 1
        self.last_note = self._packet_note(packet)

    def _packet_note(self, packet):
        if self.channel.name == "competition":
            try:
                status = parse_competition_vehicle_status(packet)
            except CompetitionStatusPacketError as error:
                self.invalid_packet_count += 1
                return "invalid_status={}".format(error)
            return (
                "mode={} gear={} speed={:+.2f}km/h "
                "pedal=({:.2f},{:.2f}) steer={:+.1f}deg"
            ).format(
                status.ctrl_mode,
                status.gear,
                status.signed_velocity_kmh,
                status.accel_pedal,
                status.brake_pedal,
                status.front_steer_deg,
            )
        if self.channel.name == "lidar":
            if len(packet) == 1206:
                return "velodyne_payload=1206B"
            if len(packet) == 1248:
                return "velodyne_with_network_header=1248B"
            return "payload={}B".format(len(packet))
        if self.channel.name == "camera":
            if packet.startswith(b"\xff\xd8"):
                return "jpeg_start payload={}B".format(len(packet))
            return "payload={}B".format(len(packet))
        return ""


def _fmt(value, fmt="{:.2f}", empty="n/a"):
    if value is None:
        return empty
    return fmt.format(value)


def _bool_arg(value):
    if isinstance(value, bool):
        return value
    value = str(value).strip().lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError("expected true/false, got {!r}".format(value))


def _open_receive_socket(bind_ip, port, rcvbuf_bytes):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, int(rcvbuf_bytes))
    sock.bind((bind_ip, int(port)))
    sock.setblocking(False)
    return sock


def _parse_extra_channel(spec):
    parts = spec.split(":")
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError(
            "--extra-channel must be name:destination_port or "
            "name:expected_source_port:destination_port"
        )
    if len(parts) == 2:
        name, destination = parts
        source = None
    else:
        name, source, destination = parts
        source = int(source)
    return ReceiveChannel(name=name, destination_port=int(destination), expected_source_port=source)


def _build_channels(args):
    channels = []
    if not args.no_gps:
        channels.append(ReceiveChannel("gps", args.gps_port))
    if not args.no_imu:
        channels.append(ReceiveChannel("imu", args.imu_port))
    if not args.no_camera:
        if args.camera_port > 0:
            channels.append(
                ReceiveChannel(
                    "camera",
                    args.camera_port,
                    args.camera_host_port if args.camera_host_port > 0 else None,
                )
            )
        else:
            print(
                "  camera RX skipped: set --camera-port/--camera-host-port "
                "to the MORAI Camera sensor ports",
                file=sys.stderr,
            )
    if not args.no_lidar:
        channels.append(
            ReceiveChannel("lidar", args.lidar_port, args.lidar_host_port)
        )
    if not args.no_competition:
        channels.append(
            ReceiveChannel(
                "competition",
                args.competition_status_port,
                args.competition_status_host_port,
            )
        )
    if not args.no_collision:
        channels.append(
            ReceiveChannel(
                "collision",
                args.collision_port,
                args.collision_host_port,
            )
        )
    channels.extend(args.extra_channel)
    return channels


def _print_header(args, channels):
    print("MORAI UDP rate monitor")
    print("  bind_ip={}".format(args.bind_ip))
    print("  default scope: GPS, IMU, Camera(if port set), 3D LiDAR, Competition, Collision")
    for channel in channels:
        if channel.expected_source_port is None:
            print(
                "  RX {:<12} destination {}:{}".format(
                    channel.name,
                    args.bind_ip,
                    channel.destination_port,
                )
            )
        else:
            print(
                "  RX {:<12} source *:{} -> destination {}:{}".format(
                    channel.name,
                    channel.expected_source_port,
                    args.bind_ip,
                    channel.destination_port,
                )
            )
    if args.send_control:
        print(
            "  TX control      source {}:{} -> MORAI {}:{} at {:.1f}Hz "
            "accel={:.2f} brake={:.2f} steer={:+.2f} protocol={}".format(
                args.bind_ip,
                args.control_source_port,
                args.control_ip,
                args.control_port,
                args.control_rate_hz,
                args.tx_accel,
                args.tx_brake,
                args.tx_steer,
                args.control_protocol,
            )
        )
    else:
        print("  TX control      disabled; use --send-control to check send timing")
    print("")


def _print_report(states, tx_stats, tx_enabled, stale_timeout_s, started_at):
    now = time.monotonic()
    elapsed = now - started_at
    print("---- {:.1f}s ----".format(elapsed))
    for state in states:
        stats = state.stats
        if stats.count == 0:
            print(
                "RX {:<12} count=0 hz=n/a stale=never".format(
                    state.channel.name,
                )
            )
            continue
        age = now - stats.last_time
        stale = age > stale_timeout_s
        sender = "{}:{}".format(state.last_sender[0], state.last_sender[1])
        print(
            "RX {:<12} count={:<6d} hz={:<7} dt(last/avg/min/max/std)="
            "{}/{}/{}/{}/{}ms age={:>6.1f}ms stale={} size={}B "
            "sender={} mismatch={} invalid={} {}".format(
                state.channel.name,
                stats.count,
                _fmt(stats.hz, "{:.1f}"),
                _fmt(stats.last_dt * 1000.0 if stats.last_dt is not None else None, "{:.1f}"),
                _fmt(stats.mean_dt * 1000.0 if stats.mean_dt is not None else None, "{:.1f}"),
                _fmt(stats.min_dt * 1000.0 if stats.min_dt is not None else None, "{:.1f}"),
                _fmt(stats.max_dt * 1000.0 if stats.max_dt is not None else None, "{:.1f}"),
                _fmt(stats.std_dt * 1000.0 if stats.std_dt is not None else None, "{:.1f}"),
                age * 1000.0,
                stale,
                state.last_payload_size,
                sender,
                state.source_mismatch_count,
                state.invalid_packet_count,
                state.last_note,
            )
        )
    if tx_enabled:
        print(
            "TX control      count={:<6d} hz={:<7} dt(last/avg/min/max/std)="
            "{}/{}/{}/{}/{}ms".format(
                tx_stats.count,
                _fmt(tx_stats.hz, "{:.1f}"),
                _fmt(tx_stats.last_dt * 1000.0 if tx_stats.last_dt is not None else None, "{:.1f}"),
                _fmt(tx_stats.mean_dt * 1000.0 if tx_stats.mean_dt is not None else None, "{:.1f}"),
                _fmt(tx_stats.min_dt * 1000.0 if tx_stats.min_dt is not None else None, "{:.1f}"),
                _fmt(tx_stats.max_dt * 1000.0 if tx_stats.max_dt is not None else None, "{:.1f}"),
                _fmt(tx_stats.std_dt * 1000.0 if tx_stats.std_dt is not None else None, "{:.1f}"),
            )
        )
    print("")
    sys.stdout.flush()


def _make_control_packet(args):
    command = pedal_command(args.tx_accel, args.tx_brake, args.tx_steer)
    return encode_ego_ctrl_cmd(command, args.control_protocol)


def run(args):
    channels = _build_channels(args)
    if not channels and not args.send_control:
        raise ValueError("at least one receive channel or --send-control is required")

    selector = selectors.DefaultSelector()
    sockets = []
    states = []
    tx_socket = None

    try:
        for channel in channels:
            sock = _open_receive_socket(
                args.bind_ip,
                channel.destination_port,
                args.rcvbuf_bytes,
            )
            state = ChannelState(channel)
            selector.register(sock, selectors.EVENT_READ, state)
            sockets.append(sock)
            states.append(state)

        control_packet = None
        control_destination = None
        tx_stats = TimingStats()
        next_tx_time = None
        if args.send_control:
            if args.control_rate_hz <= 0.0:
                raise ValueError("--control-rate-hz must be positive")
            tx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            tx_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            tx_socket.bind((args.bind_ip, args.control_source_port))
            control_packet = _make_control_packet(args)
            control_destination = (args.control_ip, args.control_port)
            next_tx_time = time.monotonic()

        _print_header(args, channels)
        started_at = time.monotonic()
        next_report = started_at
        end_time = None if args.duration <= 0.0 else started_at + args.duration

        while True:
            now = time.monotonic()
            if end_time is not None and now >= end_time:
                break

            if args.send_control and now >= next_tx_time:
                tx_socket.sendto(control_packet, control_destination)
                tx_stats.update(now, len(control_packet))
                next_tx_time += 1.0 / args.control_rate_hz
                if next_tx_time < now - 1.0:
                    next_tx_time = now

            if now >= next_report:
                _print_report(
                    states,
                    tx_stats,
                    args.send_control,
                    args.stale_timeout,
                    started_at,
                )
                next_report += args.report_interval

            timeout_candidates = [max(0.0, next_report - now)]
            if args.send_control:
                timeout_candidates.append(max(0.0, next_tx_time - now))
            if end_time is not None:
                timeout_candidates.append(max(0.0, end_time - now))
            timeout = min(timeout_candidates)

            for key, _mask in selector.select(timeout):
                try:
                    packet, sender = key.fileobj.recvfrom(65535)
                except BlockingIOError:
                    continue
                key.data.update_packet(packet, sender, time.monotonic())
    finally:
        selector.close()
        for sock in sockets:
            sock.close()
        if tx_socket is not None:
            tx_socket.close()


def argument_parser():
    parser = argparse.ArgumentParser(
        description="Measure MORAI UDP packet receive/send periods."
    )
    parser.add_argument("--bind-ip", default=BIND_IP)
    parser.add_argument("--duration", type=float, default=0.0, help="0 means forever")
    parser.add_argument("--report-interval", type=float, default=1.0)
    parser.add_argument("--stale-timeout", type=float, default=0.5)
    parser.add_argument("--rcvbuf-bytes", type=int, default=4 * 1024 * 1024)

    parser.add_argument("--gps-port", type=int, default=GPS_PORT)
    parser.add_argument("--imu-port", type=int, default=IMU_PORT)
    parser.add_argument("--camera-host-port", type=int, default=CAMERA_HOST_PORT)
    parser.add_argument(
        "--camera-port",
        type=int,
        default=CAMERA_PORT,
        help="Camera sensor destination port; 0 disables camera RX monitor",
    )
    parser.add_argument("--lidar-host-port", type=int, default=LIDAR_HOST_PORT)
    parser.add_argument("--lidar-port", type=int, default=LIDAR_PORT)
    parser.add_argument(
        "--competition-status-host-port",
        type=int,
        default=COMPETITION_STATUS_HOST_PORT,
    )
    parser.add_argument(
        "--competition-status-port",
        type=int,
        default=COMPETITION_STATUS_PORT,
    )
    parser.add_argument("--collision-host-port", type=int, default=COLLISION_HOST_PORT)
    parser.add_argument("--collision-port", type=int, default=COLLISION_PORT)

    parser.add_argument("--no-gps", action="store_true")
    parser.add_argument("--no-imu", action="store_true")
    parser.add_argument("--no-camera", action="store_true")
    parser.add_argument("--no-lidar", action="store_true")
    parser.add_argument("--no-competition", action="store_true")
    parser.add_argument("--no-collision", action="store_true")
    parser.add_argument(
        "--extra-channel",
        action="append",
        default=[],
        type=_parse_extra_channel,
        help=(
            "add RX monitor as name:destination_port or "
            "name:expected_source_port:destination_port; repeatable"
        ),
    )

    parser.add_argument(
        "--send-control",
        nargs="?",
        const=True,
        default=False,
        type=_bool_arg,
        help="true/false; when true, also send Ego Ctrl Cmd timing packets",
    )
    parser.add_argument("--control-ip", default=CONTROL_IP)
    parser.add_argument("--control-port", type=int, default=CONTROL_PORT)
    parser.add_argument(
        "--control-source-port",
        type=int,
        default=CONTROL_DESTINATION_PORT,
    )
    parser.add_argument("--control-rate-hz", type=float, default=30.0)
    parser.add_argument(
        "--control-protocol",
        choices=CONTROL_PROTOCOLS,
        default=CONTROL_PROTOCOL_25S4,
    )
    parser.add_argument("--tx-accel", type=float, default=0.0)
    parser.add_argument("--tx-brake", type=float, default=0.0)
    parser.add_argument("--tx-steer", type=float, default=0.0)
    return parser


def main():
    args = argument_parser().parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print("\nStopped UDP rate monitor")


if __name__ == "__main__":
    main()
