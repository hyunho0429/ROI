#!/usr/bin/env python3
"""Run Stanley path tracking against MORAI UDP without ROS."""

import argparse
import math
import os
import selectors
import socket
import sys
import time

from path_planning.coordinates import GpsToMapEnu, MapProjection
from path_planning.localization import LocalizationState, PlanarGpsImuEkf
from path_planning.morai_udp_ctrl_cmd import (
    brake_command,
    encode_ego_ctrl_cmd_24r1,
    velocity_command,
)
from path_planning.morai_udp_ego_status import UdpPacketError, parse_ego_vehicle_status_24r1
from path_planning.morai_udp_gps import GpsPacketError, parse_nmea_datagram
from path_planning.morai_udp_imu import ImuPacketError, parse_imu_packet, quaternion_to_yaw
from path_planning.stanley_controller import StanleyController, load_path_csv


PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_PATH = os.path.join(PACKAGE_DIR, "data", "morai_global_path.csv")
DEFAULT_GLOBAL_INFO = os.path.join(
    PACKAGE_DIR, "mgeo", "R_KR_PR_K-city_2025", "global_info.json"
)


def _receiver(bind_ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    sock.bind((bind_ip, port))
    sock.setblocking(False)
    return sock


def _projection_from_arguments(arguments):
    projection = MapProjection.from_mgeo_global_info(arguments.global_info)
    return MapProjection(
        arguments.utm_crs or projection.crs,
        projection.origin_x_m if arguments.utm_origin_x is None else arguments.utm_origin_x,
        projection.origin_y_m if arguments.utm_origin_y is None else arguments.utm_origin_y,
        projection.origin_z_m if arguments.utm_origin_z is None else arguments.utm_origin_z,
    )


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="MORAI UDP Stanley controller with GPS/IMU sensor fusion."
    )
    parser.add_argument("--path", default=DEFAULT_PATH, help="ENU waypoint CSV")
    parser.add_argument("--localization", choices=("gps-imu", "ego"), default="gps-imu")
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--gps-port", type=int, default=9100)
    parser.add_argument("--imu-port", type=int, default=9101)
    parser.add_argument("--ego-status-port", type=int, default=909)
    parser.add_argument("--control-ip", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=9090)
    parser.add_argument("--control-rate-hz", type=float, default=20.0)
    parser.add_argument("--target-speed-kmh", type=float, default=20.0)
    parser.add_argument("--stanley-gain", type=float, default=1.2)
    parser.add_argument("--softening-speed", type=float, default=1.0)
    parser.add_argument("--max-steering-deg", type=float, default=36.25)
    parser.add_argument(
        "--control-point-offset",
        type=float,
        default=0.0,
        help="Metres forward from the reported/estimated position; keep 0 unless its reference is verified",
    )
    parser.add_argument(
        "--morai-steer-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=-1.0,
        help="Convert Stanley left-positive wheel angle to MORAI normalized steering",
    )
    parser.add_argument("--imu-yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--imu-yaw-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--gps-timeout", type=float, default=1.0)
    parser.add_argument("--imu-timeout", type=float, default=0.5)
    parser.add_argument("--ego-timeout", type=float, default=0.5)
    parser.add_argument("--gps-position-sigma", type=float, default=1.5)
    parser.add_argument("--gps-speed-sigma", type=float, default=0.8)
    parser.add_argument("--imu-yaw-sigma-deg", type=float, default=3.0)
    parser.add_argument("--global-info", default=DEFAULT_GLOBAL_INFO)
    parser.add_argument("--utm-crs", default=None)
    parser.add_argument("--utm-origin-x", type=float, default=None)
    parser.add_argument("--utm-origin-y", type=float, default=None)
    parser.add_argument("--utm-origin-z", type=float, default=None)
    return parser.parse_args(argv)


def _validate(arguments):
    for name in ("gps_port", "imu_port", "ego_status_port", "control_port"):
        value = getattr(arguments, name)
        if not 1 <= value <= 65535:
            raise ValueError("{} must be between 1 and 65535".format(name))
    if arguments.control_rate_hz <= 0.0:
        raise ValueError("control-rate-hz must be positive")
    if arguments.target_speed_kmh < 0.0:
        raise ValueError("target-speed-kmh cannot be negative")


def run(arguments):
    _validate(arguments)
    points = load_path_csv(arguments.path)
    controller = StanleyController(
        points,
        gain=arguments.stanley_gain,
        softening_speed_mps=arguments.softening_speed,
        max_steering_deg=arguments.max_steering_deg,
        control_point_offset_m=arguments.control_point_offset,
    )
    selector = selectors.DefaultSelector()
    receive_sockets = []
    latest_gps_time = latest_imu_time = latest_ego_time = None
    ekf = converter = None
    ego_state = None

    if arguments.localization == "gps-imu":
        converter = GpsToMapEnu(_projection_from_arguments(arguments))
        ekf = PlanarGpsImuEkf(
            gps_position_sigma_m=arguments.gps_position_sigma,
            gps_speed_sigma_mps=arguments.gps_speed_sigma,
            imu_yaw_sigma_deg=arguments.imu_yaw_sigma_deg,
        )
        for kind, port in (("gps", arguments.gps_port), ("imu", arguments.imu_port)):
            sock = _receiver(arguments.bind_ip, port)
            selector.register(sock, selectors.EVENT_READ, kind)
            receive_sockets.append(sock)
    else:
        print("WARNING: --localization ego uses simulator Ground Truth and is for debugging only.")
        sock = _receiver(arguments.bind_ip, arguments.ego_status_port)
        selector.register(sock, selectors.EVENT_READ, "ego")
        receive_sockets.append(sock)

    control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (arguments.control_ip, arguments.control_port)
    period = 1.0 / arguments.control_rate_hz
    next_control = time.monotonic()
    last_log = 0.0
    invalid_counts = {"gps": 0, "imu": 0, "ego": 0}

    print("MORAI UDP Stanley controller started")
    print("  localization: {}".format(arguments.localization))
    print("  path: {} ({} points)".format(os.path.abspath(arguments.path), len(points)))
    if arguments.localization == "gps-imu":
        print("  GPS: {}:{} / IMU: {}:{}".format(
            arguments.bind_ip, arguments.gps_port, arguments.bind_ip, arguments.imu_port
        ))
    else:
        print("  Ego status: {}:{}".format(arguments.bind_ip, arguments.ego_status_port))
    print("  control destination: {}:{}".format(*destination))

    try:
        while True:
            now = time.monotonic()
            timeout = max(0.0, min(period, next_control - now))
            for key, _mask in selector.select(timeout):
                packet, _sender = key.fileobj.recvfrom(65535)
                received = time.monotonic()
                try:
                    if key.data == "gps":
                        measurement = parse_nmea_datagram(packet)
                        if measurement.fix_valid:
                            x_m, y_m, z_m = converter.convert(
                                measurement.latitude_deg,
                                measurement.longitude_deg,
                                measurement.altitude_m,
                            )
                            accepted = ekf.add_gps(
                                received,
                                x_m,
                                y_m,
                                z_m if measurement.altitude_m is not None else None,
                                measurement.speed_mps,
                                measurement.course_deg,
                            )
                            if accepted:
                                latest_gps_time = received
                    elif key.data == "imu":
                        measurement = parse_imu_packet(packet)
                        yaw = arguments.imu_yaw_sign * quaternion_to_yaw(
                            measurement.orientation_xyzw
                        ) + math.radians(arguments.imu_yaw_offset_deg)
                        gyro_z = arguments.imu_yaw_sign * measurement.angular_velocity_radps[2]
                        ekf.add_imu(received, yaw, gyro_z)
                        latest_imu_time = received
                    else:
                        status = parse_ego_vehicle_status_24r1(packet)
                        ego_state = LocalizationState(
                            status.position_m[0],
                            status.position_m[1],
                            status.position_m[2],
                            math.radians(status.rotation_deg[2]),
                            abs(status.signed_velocity_kmh) / 3.6,
                            received,
                        )
                        latest_ego_time = received
                except (GpsPacketError, ImuPacketError, UdpPacketError) as error:
                    invalid_counts[key.data] += 1
                    count = invalid_counts[key.data]
                    if count <= 3 or count % 100 == 0:
                        print("Ignored invalid {} packet: {}".format(key.data, error), file=sys.stderr)

            now = time.monotonic()
            if now < next_control:
                continue
            next_control = now + period
            if arguments.localization == "gps-imu":
                fresh = (
                    latest_gps_time is not None
                    and latest_imu_time is not None
                    and now - latest_gps_time <= arguments.gps_timeout
                    and now - latest_imu_time <= arguments.imu_timeout
                )
                state = ekf.state_at(now) if fresh else None
            else:
                fresh = latest_ego_time is not None and now - latest_ego_time <= arguments.ego_timeout
                state = ego_state if fresh else None

            if state is None:
                command = brake_command()
                result = None
            else:
                result = controller.compute(
                    state.x_m, state.y_m, state.z_m, state.yaw_rad, state.speed_mps
                )
                if result.goal_reached:
                    command = brake_command()
                else:
                    normalized = arguments.morai_steer_sign * (
                        result.steering_rad / controller.max_steering_rad
                    )
                    curve_scale = max(0.35, 1.0 - 0.55 * abs(normalized))
                    end_scale = max(0.25, min(1.0, result.remaining_distance_m / 15.0))
                    command = velocity_command(
                        arguments.target_speed_kmh * min(curve_scale, end_scale),
                        normalized,
                    )
            control_socket.sendto(encode_ego_ctrl_cmd_24r1(command), destination)

            if now - last_log >= 1.0:
                last_log = now
                if state is None:
                    print("Waiting for fresh localization; brake command active")
                else:
                    print(
                        "pos=({:.2f}, {:.2f}, {:.2f}) speed={:.2f}m/s "
                        "cte={:+.2f}m steer={:+.2f}deg remain={:.1f}m{}".format(
                            state.x_m,
                            state.y_m,
                            state.z_m,
                            state.speed_mps,
                            result.cross_track_error_m,
                            math.degrees(result.steering_rad),
                            result.remaining_distance_m,
                            " GOAL" if result.goal_reached else "",
                        )
                    )
    except KeyboardInterrupt:
        print("\nStopping controller and applying brake...")
    finally:
        stop_packet = encode_ego_ctrl_cmd_24r1(brake_command())
        for _ in range(5):
            control_socket.sendto(stop_packet, destination)
            time.sleep(0.02)
        selector.close()
        for sock in receive_sockets:
            sock.close()
        control_socket.close()


def main(argv=None):
    run(parse_arguments(argv))


if __name__ == "__main__":
    main()
