#!/usr/bin/env python3
"""Record allowed MORAI GPS UDP data as reproducible map-local ENU CSV."""

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
    """Write valid GPS fixes using spatial and optional time sampling."""

    def __init__(
        self,
        output_file,
        sample_period=0.0,
        sample_distance=0.5,
        projection=None,
        append=False,
    ):
        if not math.isfinite(sample_period) or sample_period < 0.0:
            raise ValueError("sample_period must be finite and non-negative")
        if not math.isfinite(sample_distance) or sample_distance < 0.0:
            raise ValueError("sample_distance must be finite and non-negative")
        if sample_period == 0.0 and sample_distance == 0.0:
            raise ValueError("sample_period and sample_distance cannot both be zero")
        self.output_file = os.path.abspath(os.path.expanduser(output_file))
        self.sample_period = float(sample_period)
        self.sample_distance = float(sample_distance)
        self.projection = projection
        self._last_sample_clock = None
        self._last_enu = None
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
        if (
            elapsed is not None
            and self.sample_period > 0.0
            and 0.0 <= elapsed < self.sample_period
        ):
            return False
        if (
            self._last_enu is not None
            and self.sample_distance > 0.0
            and math.hypot(
                float(enu[0]) - self._last_enu[0],
                float(enu[1]) - self._last_enu[1],
            )
            < self.sample_distance
        ):
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
                "latitude_deg": "{:.10f}".format(measurement.latitude_deg),
                "longitude_deg": "{:.10f}".format(measurement.longitude_deg),
                "altitude_m": (
                    ""
                    if measurement.altitude_m is None
                    else "{:.6f}".format(measurement.altitude_m)
                ),
                "projection_crs": "" if self.projection is None else self.projection.crs,
                "east_offset_m": (
                    ""
                    if self.projection is None
                    else "{:.6f}".format(self.projection.origin_x_m)
                ),
                "north_offset_m": (
                    ""
                    if self.projection is None
                    else "{:.6f}".format(self.projection.origin_y_m)
                ),
                "up_offset_m": (
                    ""
                    if self.projection is None
                    else "{:.6f}".format(self.projection.origin_z_m)
                ),
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
        self._last_enu = tuple(float(value) for value in enu)
        return True

    def close(self):
        self._writer.close()


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Record MORAI GPS UDP fixes as reproducible map-local ENU CSV."
    )
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=3001, help="GPS destination port")
    parser.add_argument("--output", default=DEFAULT_OUTPUT_FILE)
    parser.add_argument(
        "--sample-distance",
        type=float,
        default=0.5,
        help="minimum horizontal distance between saved reference points in metres",
    )
    parser.add_argument(
        "--sample-period",
        type=float,
        default=0.0,
        help="optional minimum time between saved points; zero disables the time gate",
    )
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
            sample_distance=arguments.sample_distance,
            projection=projection,
            append=arguments.append,
        )
        print("MORAI GPS path recorder started")
        print("  listen: {}:{}".format(arguments.bind_ip, arguments.port))
        print("  sample distance: {:.3f} m".format(arguments.sample_distance))
        if arguments.sample_period > 0.0:
            print("  minimum sample period: {:.3f} s".format(arguments.sample_period))
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
            if measurement.altitude_m is None:
                measurement = type(measurement)(
                    measurement.latitude_deg,
                    measurement.longitude_deg,
                    altitude_m=latest_altitude_m,
                    speed_mps=measurement.speed_mps,
                    course_deg=measurement.course_deg,
                    fix_valid=measurement.fix_valid,
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
