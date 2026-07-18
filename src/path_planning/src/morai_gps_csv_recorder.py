#!/usr/bin/env python3
"""Record MORAI GPS NMEA positions as K-City map-local ENU CSV points."""

import argparse
import datetime
import math
import os
import socket
import sys
import time

from path_planning.coordinates import GpsToMapEnu, MapProjection
from path_planning.csv_path_writer import CsvPathWriter
from path_planning.morai_udp_gps import GpsPacketError, parse_nmea_datagram


PATH_PLANNING_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_OUTPUT_FILE = os.path.join(PATH_PLANNING_DIR, "data", "morai_global_path.csv")
DEFAULT_GLOBAL_INFO = os.path.join(
    PATH_PLANNING_DIR,
    "mgeo",
    "R_KR_PR_K-city_2025",
    "global_info.json",
)


class GpsCsvRecorder:
    """Write the latest valid GPS fix at a fixed wall-clock interval."""

    def __init__(self, output_file, sample_period=1.0, append=False):
        if not math.isfinite(sample_period) or sample_period <= 0.0:
            raise ValueError("sample_period must be a finite value greater than zero")
        self.output_file = os.path.abspath(os.path.expanduser(output_file))
        self.sample_period = float(sample_period)
        self._last_sample_clock = None
        self._sequence = 0
        self._writer = CsvPathWriter(self.output_file, append=append)

    @property
    def written_count(self):
        return self._sequence

    def add_fix(self, measurement, enu, receive_time_sec, sample_clock_sec):
        elapsed = (
            None
            if self._last_sample_clock is None
            else sample_clock_sec - self._last_sample_clock
        )
        if elapsed is not None and 0.0 <= elapsed < self.sample_period:
            return False

        speed = measurement.speed_mps
        course = measurement.course_deg
        velocity_east = ""
        velocity_north = ""
        if speed is not None and course is not None:
            course_rad = math.radians(course)
            velocity_east = "{:.6f}".format(speed * math.sin(course_rad))
            velocity_north = "{:.6f}".format(speed * math.cos(course_rad))

        self._sequence += 1
        self._writer.write(
            {
                "sequence": self._sequence,
                "recorded_at_utc": datetime.datetime.now(
                    datetime.timezone.utc
                ).isoformat(),
                "receive_time_sec": "{:.9f}".format(receive_time_sec),
                "global_enu_x_m": "{:.9f}".format(enu[0]),
                "global_enu_y_m": "{:.9f}".format(enu[1]),
                "global_enu_z_m": "{:.9f}".format(enu[2]),
                "velocity_x_mps": velocity_east,
                "velocity_y_mps": velocity_north,
                "velocity_z_mps": "0.000000" if speed is not None else "",
                "speed_mps": "" if speed is None else "{:.6f}".format(speed),
                "signed_speed_mps": "" if speed is None else "{:.6f}".format(speed),
            }
        )
        self._last_sample_clock = sample_clock_sec
        return True

    def close(self):
        self._writer.close()


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Record MORAI GPS NMEA fixes as K-City map-local ENU CSV."
    )
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=3001, help="GPS destination port")
    parser.add_argument("--output", default=DEFAULT_OUTPUT_FILE)
    parser.add_argument("--sample-period", type=float, default=1.0)
    parser.add_argument("--global-info", default=DEFAULT_GLOBAL_INFO)
    parser.add_argument("--utm-crs", default=None)
    parser.add_argument("--utm-origin-x", type=float, default=None)
    parser.add_argument("--utm-origin-y", type=float, default=None)
    parser.add_argument("--utm-origin-z", type=float, default=None)
    parser.add_argument("--append", action="store_true")
    return parser.parse_args(argv)


def configured_projection(arguments):
    configured = MapProjection.from_mgeo_global_info(arguments.global_info)
    return MapProjection(
        arguments.utm_crs or configured.crs,
        configured.origin_x_m
        if arguments.utm_origin_x is None
        else arguments.utm_origin_x,
        configured.origin_y_m
        if arguments.utm_origin_y is None
        else arguments.utm_origin_y,
        configured.origin_z_m
        if arguments.utm_origin_z is None
        else arguments.utm_origin_z,
    )


def run_udp_recorder(arguments):
    if not 1 <= arguments.port <= 65535:
        raise ValueError("port must be between 1 and 65535")

    projection = configured_projection(arguments)
    converter = GpsToMapEnu(projection)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    udp_socket.bind((arguments.bind_ip, arguments.port))
    udp_socket.settimeout(1.0)

    recorder = None
    latest_altitude_m = None
    invalid_packets = 0
    missing_altitude_packets = 0
    try:
        recorder = GpsCsvRecorder(
            arguments.output,
            sample_period=arguments.sample_period,
            append=arguments.append,
        )
        print("MORAI GPS path recorder started")
        print("  listen: {}:{}".format(arguments.bind_ip, arguments.port))
        print("  sample period: {:.3f} s".format(arguments.sample_period))
        print("  map CRS: {}".format(projection.crs))
        print(
            "  map origin: ({:.3f}, {:.3f}, {:.3f})".format(
                projection.origin_x_m,
                projection.origin_y_m,
                projection.origin_z_m,
            )
        )
        print("  output: {}".format(recorder.output_file))

        while True:
            try:
                packet, _sender = udp_socket.recvfrom(65535)
            except socket.timeout:
                continue

            receive_time_sec = time.time()
            sample_clock_sec = time.monotonic()
            try:
                measurement = parse_nmea_datagram(packet)
            except GpsPacketError as error:
                invalid_packets += 1
                if invalid_packets <= 3 or invalid_packets % 100 == 0:
                    print("Ignored incompatible GPS packet: {}".format(error), file=sys.stderr)
                continue

            if not measurement.fix_valid:
                continue
            if measurement.altitude_m is not None:
                latest_altitude_m = measurement.altitude_m
            if latest_altitude_m is None:
                missing_altitude_packets += 1
                if missing_altitude_packets == 1:
                    print(
                        "Waiting for a valid GGA altitude before saving 3-D path points...",
                        file=sys.stderr,
                    )
                continue

            enu = converter.convert(
                measurement.latitude_deg,
                measurement.longitude_deg,
                latest_altitude_m,
            )
            if recorder.add_fix(
                measurement,
                enu,
                receive_time_sec,
                sample_clock_sec,
            ):
                print(
                    "  saved #{:d}: ENU ({:.3f}, {:.3f}, {:.3f}) GPS ({:.8f}, {:.8f})".format(
                        recorder.written_count,
                        *enu,
                        measurement.latitude_deg,
                        measurement.longitude_deg,
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
