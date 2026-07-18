#!/usr/bin/env python3
"""Verify MORAI Ego Ctrl Cmd reception through Competition Vehicle Status."""

import argparse
import socket
import sys
import time

from path_planning.morai_competition_config import (
    BIND_IP,
    COMPETITION_STATUS_PORT,
    CONTROL_IP,
    CONTROL_PORT,
)
from path_planning.morai_udp_competition_status import (
    CompetitionStatusPacketError,
    parse_competition_vehicle_status,
)
from path_planning.morai_udp_ctrl_cmd import (
    CONTROL_PROTOCOLS,
    brake_command,
    encode_ego_ctrl_cmd,
    external_control_ready,
    pedal_command,
)


def argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Send a safe brake command and verify MORAI feedback. "
            "Use --drive-test only in a clear test area."
        )
    )
    parser.add_argument("--bind-ip", default=BIND_IP)
    parser.add_argument(
        "--competition-status-port", type=int, default=COMPETITION_STATUS_PORT
    )
    parser.add_argument("--control-ip", default=CONTROL_IP)
    parser.add_argument("--control-port", type=int, default=CONTROL_PORT)
    parser.add_argument(
        "--control-protocol", choices=CONTROL_PROTOCOLS, default="25s4"
    )
    parser.add_argument("--brake", type=float, default=0.25)
    parser.add_argument("--brake-test-seconds", type=float, default=2.0)
    parser.add_argument(
        "--drive-test",
        action="store_true",
        help="after brake feedback succeeds, command low acceleration briefly",
    )
    parser.add_argument("--drive-test-accel", type=float, default=0.10)
    parser.add_argument("--drive-test-seconds", type=float, default=1.0)
    return parser


def _validate(arguments):
    for port in (arguments.competition_status_port, arguments.control_port):
        if not 1 <= port <= 65535:
            raise ValueError("UDP ports must be between 1 and 65535")
    for name in ("brake", "drive_test_accel"):
        if not 0.0 <= getattr(arguments, name) <= 1.0:
            raise ValueError("{} must be between 0 and 1".format(name))
    for name in ("brake_test_seconds", "drive_test_seconds"):
        if getattr(arguments, name) <= 0.0:
            raise ValueError("{} must be positive".format(name))


def _receive_latest(status_socket, current):
    while True:
        try:
            packet, _sender = status_socket.recvfrom(65535)
        except (BlockingIOError, socket.timeout):
            return current
        try:
            current = parse_competition_vehicle_status(packet)
        except CompetitionStatusPacketError as error:
            print("Ignored incompatible Competition Status: {}".format(error), file=sys.stderr)


def _run_phase(
    label,
    command,
    duration,
    expected_feedback_name,
    expected_feedback_value,
    encoder,
    control_socket,
    destination,
    status_socket,
    latest_status,
):
    packet = encoder(command)
    deadline = time.monotonic() + duration
    next_send = next_log = 0.0
    feedback_confirmed = False
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send:
            control_socket.sendto(packet, destination)
            next_send = now + 0.05
        latest_status = _receive_latest(status_socket, latest_status)
        if latest_status is not None:
            feedback = getattr(latest_status, expected_feedback_name)
            feedback_confirmed = feedback_confirmed or abs(
                feedback - expected_feedback_value
            ) <= 0.08
            if now >= next_log:
                print(
                    "{}: mode={} gear={} speed={:.2f} km/h "
                    "feedback=(accel={:.2f}, steer={:+.2f} deg, brake={:.2f})".format(
                        label,
                        latest_status.ctrl_mode,
                        latest_status.gear,
                        latest_status.signed_velocity_kmh,
                        latest_status.accel_pedal,
                        latest_status.front_steer_deg,
                        latest_status.brake_pedal,
                    )
                )
                next_log = now + 0.5
        time.sleep(0.005)
    return latest_status, feedback_confirmed


def run(arguments):
    _validate(arguments)
    destination = (arguments.control_ip, arguments.control_port)
    encoder = lambda command: encode_ego_ctrl_cmd(
        command, arguments.control_protocol
    )

    status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        status_socket.bind(
            (arguments.bind_ip, arguments.competition_status_port)
        )
    except OSError as error:
        raise OSError(
            "cannot bind Competition Status {}:{}; verify MORAI Destination "
            "port and Linux permission for ports below 1024 ({})".format(
                arguments.bind_ip,
                arguments.competition_status_port,
                error,
            )
        ) from error
    status_socket.setblocking(False)
    control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("MORAI UDP control reception check")
    print(
        "  status: {}:{}; control: {}:{}; protocol={} ({} bytes)".format(
            arguments.bind_ip,
            arguments.competition_status_port,
            destination[0],
            destination[1],
            arguments.control_protocol,
            len(encoder(brake_command(arguments.brake))),
        )
    )
    latest_status = None
    try:
        latest_status, brake_confirmed = _run_phase(
            "BRAKE",
            brake_command(arguments.brake),
            arguments.brake_test_seconds,
            "brake_pedal",
            arguments.brake,
            encoder,
            control_socket,
            destination,
            status_socket,
            latest_status,
        )
        if latest_status is None:
            print(
                "FAIL: no valid Competition Vehicle Status packet was received",
                file=sys.stderr,
            )
            return 2
        if not external_control_ready(latest_status.ctrl_mode, latest_status.gear):
            print(
                "FAIL: status did not confirm ctrl_mode=2 and gear=4",
                file=sys.stderr,
            )
            return 3
        if not brake_confirmed:
            print(
                "FAIL: external mode is active but brake feedback did not follow the command; "
                "check Cmd Control Host IP/Port and control protocol",
                file=sys.stderr,
            )
            return 4
        print("PASS: MORAI reflected the longCmdType-1 brake command")

        if arguments.drive_test:
            latest_status, accel_confirmed = _run_phase(
                "ACCEL",
                pedal_command(arguments.drive_test_accel, 0.0, 0.0),
                arguments.drive_test_seconds,
                "accel_pedal",
                arguments.drive_test_accel,
                encoder,
                control_socket,
                destination,
                status_socket,
                latest_status,
            )
            if not accel_confirmed:
                print("FAIL: acceleration feedback did not follow the command", file=sys.stderr)
                return 5
            print("PASS: MORAI reflected the low acceleration command")
        return 0
    finally:
        stop_packet = encoder(brake_command())
        for _ in range(5):
            control_socket.sendto(stop_packet, destination)
            time.sleep(0.02)
        status_socket.close()
        control_socket.close()


def main(argv=None):
    return run(argument_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
