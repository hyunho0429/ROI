#!/usr/bin/env python3
"""Record MORAI Competition Vehicle Status ENU positions to CSV."""

import argparse
import datetime
import math
import os
import socket
import sys
import threading
import time

from path_planning.csv_path_writer import CsvPathWriter
from path_planning.morai_udp_competition_status import (
    CompetitionStatusPacketError,
    parse_competition_vehicle_status,
)


DEFAULT_OUTPUT_FILE = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "data", "morai_global_path.csv")
)


def status_to_sample(status, receive_time_sec):
    """Convert Competition Status packet units to an SI-unit sample."""
    return {
        "receive_time_sec": float(receive_time_sec),
        "message_time_sec": status.timestamp_sec,
        "enu": status.position_m,
        "roll_deg": status.rotation_deg[0],
        "pitch_deg": status.rotation_deg[1],
        "heading_deg": status.rotation_deg[2],
        "velocity": tuple(value / 3.6 for value in status.velocity_kmh),
        "signed_speed_mps": status.signed_velocity_kmh / 3.6,
        "ctrl_mode": status.ctrl_mode,
        "gear": status.gear,
        "map_data_id": status.map_data_id,
        "wheelbase_m": status.wheelbase_m,
        "overhang_m": status.overhang_m,
        "rear_overhang_m": status.rear_overhang_m,
        "link_id": status.link_id,
    }


class MoraiGlobalCsvRecorder:
    def __init__(self, output_file, sample_period=1.0, append=False):
        if not math.isfinite(sample_period) or sample_period <= 0.0:
            raise ValueError("sample_period must be a finite value greater than zero")

        self.output_file = os.path.abspath(os.path.expanduser(output_file))
        self.sample_period = float(sample_period)
        self._lock = threading.RLock()
        self._last_sample_clock = None
        self._sequence = 0
        self._closed = False
        self._writer = CsvPathWriter(self.output_file, append=append)

    @property
    def written_count(self):
        return self._sequence

    def add_sample(self, sample, sample_clock_sec=None):
        """Write the first sample immediately, then at fixed time intervals."""
        with self._lock:
            if self._closed:
                return False
            clock = (
                float(sample["receive_time_sec"])
                if sample_clock_sec is None
                else float(sample_clock_sec)
            )
            if not math.isfinite(clock):
                raise ValueError("sample clock must be finite")
            elapsed = (
                None
                if self._last_sample_clock is None
                else clock - self._last_sample_clock
            )
            if elapsed is not None and 0.0 <= elapsed < self.sample_period:
                return False
            self._write_sample(sample)
            self._last_sample_clock = clock
            return True

    def _write_sample(self, sample):
        enu = sample["enu"]
        velocity = sample["velocity"]
        self._sequence += 1
        self._writer.write(
            {
                "sequence": self._sequence,
                "recorded_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "receive_time_sec": "{:.9f}".format(sample["receive_time_sec"]),
                "message_time_sec": "{:.9f}".format(sample["message_time_sec"]),
                "global_enu_x_m": "{:.9f}".format(enu[0]),
                "global_enu_y_m": "{:.9f}".format(enu[1]),
                "global_enu_z_m": "{:.9f}".format(enu[2]),
                "roll_deg": "{:.6f}".format(sample["roll_deg"]),
                "pitch_deg": "{:.6f}".format(sample["pitch_deg"]),
                "heading_deg": "{:.6f}".format(sample["heading_deg"]),
                "velocity_x_mps": "{:.6f}".format(velocity[0]),
                "velocity_y_mps": "{:.6f}".format(velocity[1]),
                "velocity_z_mps": "{:.6f}".format(velocity[2]),
                "speed_mps": "{:.6f}".format(math.sqrt(sum(value * value for value in velocity))),
                "signed_speed_mps": "{:.6f}".format(sample["signed_speed_mps"]),
                "ctrl_mode": sample["ctrl_mode"],
                "gear": sample["gear"],
                "map_data_id": sample["map_data_id"],
                "wheelbase_m": "{:.6f}".format(sample["wheelbase_m"]),
                "overhang_m": "{:.6f}".format(sample["overhang_m"]),
                "rear_overhang_m": "{:.6f}".format(sample["rear_overhang_m"]),
                "link_id": sample["link_id"],
            }
        )

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._writer.close()


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Record MORAI Competition Vehicle Status to CSV."
    )
    parser.add_argument("--bind-ip", default="0.0.0.0", help="Local interface to bind")
    parser.add_argument("--port", type=int, default=3315, help="Competition Status destination port")
    parser.add_argument("--output", default=DEFAULT_OUTPUT_FILE, help="Output CSV path")
    parser.add_argument("--sample-period", type=float, default=1.0, help="CSV sampling period in seconds")
    parser.add_argument(
        "--sample-distance",
        type=float,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--append", action="store_true", help="Append only when the CSV header matches")
    return parser.parse_args(argv)


def run_udp_recorder(arguments):
    if not 1 <= arguments.port <= 65535:
        raise ValueError("port must be between 1 and 65535")

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    udp_socket.bind((arguments.bind_ip, arguments.port))
    udp_socket.settimeout(1.0)

    recorder = None
    try:
        if arguments.sample_distance is not None:
            print(
                "Warning: --sample-distance is deprecated and ignored; "
                "using --sample-period {:.3f} s".format(arguments.sample_period),
                file=sys.stderr,
            )
        recorder = MoraiGlobalCsvRecorder(
            arguments.output,
            sample_period=arguments.sample_period,
            append=arguments.append,
        )
        invalid_packets = 0
        zero_position_packets = 0
        print("MORAI Competition Status path recorder started")
        print("  listen: {}:{}".format(arguments.bind_ip, arguments.port))
        print("  sample period: {:.3f} s".format(arguments.sample_period))
        print("  output: {}".format(recorder.output_file))

        while True:
            try:
                packet, _sender = udp_socket.recvfrom(65535)
            except socket.timeout:
                continue

            receive_time_sec = time.time()
            sample_clock_sec = time.monotonic()
            try:
                status = parse_competition_vehicle_status(packet)
            except CompetitionStatusPacketError as error:
                invalid_packets += 1
                if invalid_packets <= 3 or invalid_packets % 100 == 0:
                    print("Ignored incompatible UDP packet: {}".format(error), file=sys.stderr)
                continue

            if all(abs(value) <= 1e-6 for value in status.position_m):
                zero_position_packets += 1
                if zero_position_packets == 3:
                    print(
                        "Competition Vehicle Status is reporting position (0, 0, 0); "
                        "the competition interface appears to mask position. No zero "
                        "position will be saved. Use morai_gps_csv_recorder.py with "
                        "the GPS sensor destination port instead.",
                        file=sys.stderr,
                    )
                continue
            zero_position_packets = 0

            if recorder.add_sample(
                status_to_sample(status, receive_time_sec), sample_clock_sec
            ):
                print(
                    "  saved #{:d}: ({:.3f}, {:.3f}, {:.3f})".format(
                        recorder.written_count, *status.position_m
                    )
                )
    except KeyboardInterrupt:
        print("\nStopping recorder...")
    finally:
        if recorder is not None:
            recorder.close()
        udp_socket.close()
        if recorder is not None:
            print("CSV saved: {}".format(recorder.output_file))


def main(argv=None):
    run_udp_recorder(parse_arguments(argv))


if __name__ == "__main__":
    main()
