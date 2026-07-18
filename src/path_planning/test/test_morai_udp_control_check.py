#!/usr/bin/env python3

import os
import socket
import struct
import sys
import threading
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from morai_udp_control_check import argument_parser, run
from path_planning.morai_udp_competition_status import (
    BASE_PACKET_DATA_LENGTH,
    BASE_PACKET_SIZE,
    PACKET_HEADER,
)


def available_udp_port():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        udp_socket.bind(("127.0.0.1", 0))
        return udp_socket.getsockname()[1]
    finally:
        udp_socket.close()


def status_packet(accel, brake):
    packet = bytearray(BASE_PACKET_SIZE)
    packet[:11] = PACKET_HEADER
    struct.pack_into("<I", packet, 11, BASE_PACKET_DATA_LENGTH)
    struct.pack_into("<II", packet, 27, 10, 0)
    struct.pack_into("<bb", packet, 35, 2, 4)
    struct.pack_into("<ff", packet, 45, accel, brake)
    packet[-2:] = b"\r\n"
    return bytes(packet)


class MoraiUdpControlCheckTest(unittest.TestCase):
    def test_loopback_confirms_25s4_brake_feedback(self):
        status_port = available_udp_port()
        control_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        control_receiver.bind(("127.0.0.1", 0))
        control_port = control_receiver.getsockname()[1]
        control_receiver.settimeout(0.1)
        stop_event = threading.Event()
        observed_packet_sizes = []

        def fake_simulator():
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                while not stop_event.is_set():
                    try:
                        packet, _address = control_receiver.recvfrom(1024)
                    except socket.timeout:
                        continue
                    observed_packet_sizes.append(len(packet))
                    fields = struct.unpack_from("<BBBfffff", packet, 30)
                    sender.sendto(
                        status_packet(fields[5], fields[6]),
                        ("127.0.0.1", status_port),
                    )
            finally:
                sender.close()

        simulator_thread = threading.Thread(target=fake_simulator, daemon=True)
        simulator_thread.start()
        arguments = argument_parser().parse_args(
            [
                "--bind-ip",
                "127.0.0.1",
                "--competition-status-port",
                str(status_port),
                "--control-ip",
                "127.0.0.1",
                "--control-port",
                str(control_port),
                "--brake-test-seconds",
                "0.25",
            ]
        )
        try:
            self.assertEqual(run(arguments), 0)
        finally:
            stop_event.set()
            simulator_thread.join(timeout=1.0)
            control_receiver.close()

        self.assertTrue(observed_packet_sizes)
        self.assertEqual(set(observed_packet_sizes), {55})


if __name__ == "__main__":
    unittest.main()
