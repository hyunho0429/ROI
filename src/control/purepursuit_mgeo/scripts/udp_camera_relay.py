#!/usr/bin/env python3
"""Fan out one MORAI UDP camera stream to multiple local UDP ports.

The relay is intentionally transport-only: it does not decode, reorder, or
modify camera packets. This lets the camera team's existing consumers run
unchanged on separate local ports.
"""
from __future__ import annotations

import argparse
import socket
import time


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind-ip", default="0.0.0.0")
    ap.add_argument("--input-port", type=int, default=1101)
    ap.add_argument("--output-host", default="127.0.0.1")
    ap.add_argument("--output-ports", default="1102,1103")
    ap.add_argument("--recv-buffer-bytes", type=int, default=4 * 1024 * 1024)
    ap.add_argument("--stats-period-s", type=float, default=5.0)
    args, _unknown = ap.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    output_ports = []
    for raw in args.output_ports.split(","):
        raw = raw.strip()
        if not raw:
            continue
        port = int(raw)
        if port <= 0 or port > 65535:
            raise SystemExit(f"invalid output port: {port}")
        output_ports.append(port)
    if not output_ports:
        raise SystemExit("at least one output port is required")
    if args.input_port in output_ports:
        raise SystemExit("input port must differ from every output port")

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.recv_buffer_bytes)
    rx.bind((args.bind_ip, args.input_port))

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    destinations = [(args.output_host, port) for port in output_ports]
    print(
        "[camera_relay] listening %s:%d -> %s"
        % (
            args.bind_ip,
            args.input_port,
            ", ".join(f"{host}:{port}" for host, port in destinations),
        ),
        flush=True,
    )

    packet_count = 0
    byte_count = 0
    last_stats = time.monotonic()

    try:
        while True:
            data, _source = rx.recvfrom(65535)
            for destination in destinations:
                tx.sendto(data, destination)

            packet_count += 1
            byte_count += len(data)
            now = time.monotonic()
            elapsed = now - last_stats
            if args.stats_period_s > 0.0 and elapsed >= args.stats_period_s:
                pps = packet_count / elapsed
                mbps = (byte_count * 8.0) / elapsed / 1_000_000.0
                print(
                    f"[camera_relay] rx={pps:.1f} pkt/s {mbps:.2f} Mbit/s "
                    f"fanout={len(destinations)}",
                    flush=True,
                )
                packet_count = 0
                byte_count = 0
                last_stats = now
    except KeyboardInterrupt:
        pass
    finally:
        rx.close()
        tx.close()


if __name__ == "__main__":
    main()
