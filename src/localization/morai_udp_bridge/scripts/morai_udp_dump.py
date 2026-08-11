#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 없이 MORAI GPS·IMU UDP 패킷을 확인하는 독립 수신기."""

from __future__ import annotations

import argparse
from pathlib import Path
import select
import socket
import sys


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from morai_udp_bridge.protocol import (  # noqa: E402
    GPS_PACKET_SIZE,
    IMU_PACKET_SIZE,
    ProtocolError,
    parse_gps_packet,
    parse_imu_packet,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--gps-port", type=int, default=3001)
    parser.add_argument("--imu-port", type=int, default=4001)
    args = parser.parse_args()

    gps_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    imu_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sockets = (gps_socket, imu_socket)
    try:
        for sock in sockets:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        gps_socket.bind((args.bind_ip, args.gps_port))
        imu_socket.bind((args.bind_ip, args.imu_port))
        print(f"GPS bind: {args.bind_ip}:{args.gps_port}")
        print(f"IMU bind: {args.bind_ip}:{args.imu_port}")
        print("Ctrl+C로 종료합니다.")

        while True:
            readable, _, _ = select.select(list(sockets), [], [], 1.0)
            for sock in readable:
                packet, sender = sock.recvfrom(max(GPS_PACKET_SIZE, IMU_PACKET_SIZE))
                try:
                    if sock is gps_socket:
                        gps = parse_gps_packet(packet)
                        print(
                            f"GPS {gps.sentence_type} from={sender} "
                            f"lat={gps.latitude:.8f} lon={gps.longitude:.8f} "
                            f"alt={gps.altitude} status={gps.status} len={len(packet)}"
                        )
                    else:
                        imu = parse_imu_packet(packet)
                        print(
                            f"IMU {imu.layout} from={sender} "
                            f"q_wxyz={imu.quaternion} "
                            f"gyro={imu.angular_velocity} "
                            f"accel={imu.linear_acceleration} len={len(packet)}"
                        )
                except (ProtocolError, UnicodeError, ValueError) as exc:
                    print(f"폐기 packet from={sender} len={len(packet)}: {exc}")
    except KeyboardInterrupt:
        return 0
    finally:
        for sock in sockets:
            sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
