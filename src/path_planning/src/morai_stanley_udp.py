#!/usr/bin/env python3
"""Run Stanley tracking with MORAI competition UDP interfaces."""

import argparse
import math
import os
import selectors
import socket
import sys
import time

from path_planning.coordinates import GpsToMapEnu, MapProjection
from path_planning.localization import PlanarGpsImuEkf
from path_planning.longitudinal_controller import PedalSpeedController
from path_planning.morai_udp_collision_data import (
    CollisionPacketError,
    parse_collision_data_26r1,
)
from path_planning.morai_udp_competition_status import (
    CompetitionStatusPacketError,
    parse_competition_vehicle_status,
)
from path_planning.morai_udp_ctrl_cmd import (
    brake_command,
    encode_ego_ctrl_cmd_26r1,
    pedal_command,
)
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
        projection.origin_x_m
        if arguments.utm_origin_x is None
        else arguments.utm_origin_x,
        projection.origin_y_m
        if arguments.utm_origin_y is None
        else arguments.utm_origin_y,
        projection.origin_z_m
        if arguments.utm_origin_z is None
        else arguments.utm_origin_z,
    )


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="MORAI competition UDP Stanley controller with GPS/IMU fusion."
    )
    parser.add_argument("--path", default=DEFAULT_PATH, help="map-origin ENU waypoint CSV")
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--gps-port", type=int, default=9100)
    parser.add_argument("--imu-port", type=int, default=9101)
    parser.add_argument("--competition-status-port", type=int, default=3315)
    parser.add_argument("--collision-port", type=int, default=5678)
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
        help="metres forward from GPS position; keep 0 until sensor mounting is measured",
    )
    parser.add_argument(
        "--morai-steer-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=-1.0,
    )
    parser.add_argument("--imu-yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--imu-yaw-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--gps-timeout", type=float, default=1.0)
    parser.add_argument("--imu-timeout", type=float, default=0.5)
    parser.add_argument("--status-timeout", type=float, default=0.5)
    parser.add_argument("--collision-brake-seconds", type=float, default=3.0)
    parser.add_argument("--gps-position-sigma", type=float, default=1.5)
    parser.add_argument("--gps-speed-sigma", type=float, default=0.8)
    parser.add_argument("--imu-yaw-sigma-deg", type=float, default=3.0)
    parser.add_argument("--speed-kp", type=float, default=0.12)
    parser.add_argument("--speed-ki", type=float, default=0.04)
    parser.add_argument("--max-accel-pedal", type=float, default=0.65)
    parser.add_argument("--max-brake-pedal", type=float, default=0.8)
    parser.add_argument("--global-info", default=DEFAULT_GLOBAL_INFO)
    parser.add_argument("--utm-crs", default=None)
    parser.add_argument("--utm-origin-x", type=float, default=None)
    parser.add_argument("--utm-origin-y", type=float, default=None)
    parser.add_argument("--utm-origin-z", type=float, default=None)
    return parser.parse_args(argv)


def _validate(arguments):
    ports = (
        "gps_port",
        "imu_port",
        "competition_status_port",
        "collision_port",
        "control_port",
    )
    for name in ports:
        value = getattr(arguments, name)
        if not 1 <= value <= 65535:
            raise ValueError("{} must be between 1 and 65535".format(name))
    receive_ports = [getattr(arguments, name) for name in ports[:-1]]
    if len(receive_ports) != len(set(receive_ports)):
        raise ValueError("GPS, IMU, status, and collision receive ports must be distinct")
    if arguments.control_rate_hz <= 0.0:
        raise ValueError("control-rate-hz must be positive")
    if arguments.target_speed_kmh < 0.0:
        raise ValueError("target-speed-kmh cannot be negative")
    if arguments.collision_brake_seconds < 0.0:
        raise ValueError("collision-brake-seconds cannot be negative")
    for name in ("max_accel_pedal", "max_brake_pedal"):
        if not 0.0 <= getattr(arguments, name) <= 1.0:
            raise ValueError("{} must be between 0 and 1".format(name))


def run(arguments):
    _validate(arguments)
    projection = _projection_from_arguments(arguments)
    points = load_path_csv(arguments.path, gps_projection=projection)
    controller = StanleyController(
        points,
        gain=arguments.stanley_gain,
        softening_speed_mps=arguments.softening_speed,
        max_steering_deg=arguments.max_steering_deg,
        control_point_offset_m=arguments.control_point_offset,
    )
    speed_controller = PedalSpeedController(
        kp=arguments.speed_kp,
        ki=arguments.speed_ki,
        max_accel=arguments.max_accel_pedal,
        max_brake=arguments.max_brake_pedal,
    )
    converter = GpsToMapEnu(projection)
    localizer = PlanarGpsImuEkf(
        gps_position_sigma_m=arguments.gps_position_sigma,
        gps_speed_sigma_mps=arguments.gps_speed_sigma,
        imu_yaw_sigma_deg=arguments.imu_yaw_sigma_deg,
    )

    selector = selectors.DefaultSelector()
    receive_sockets = []
    receive_channels = (
        ("gps", arguments.gps_port),
        ("imu", arguments.imu_port),
        ("status", arguments.competition_status_port),
        ("collision", arguments.collision_port),
    )
    for kind, port in receive_channels:
        sock = _receiver(arguments.bind_ip, port)
        selector.register(sock, selectors.EVENT_READ, kind)
        receive_sockets.append(sock)

    control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (arguments.control_ip, arguments.control_port)
    period = 1.0 / arguments.control_rate_hz
    next_control = time.monotonic()
    last_log = 0.0
    latest_gps_time = latest_imu_time = latest_status_time = None
    status_speed_mps = None
    collision_brake_until = 0.0
    invalid_counts = {kind: 0 for kind, _port in receive_channels}
    packet_errors = (
        GpsPacketError,
        ImuPacketError,
        CompetitionStatusPacketError,
        CollisionPacketError,
    )

    print("MORAI competition Stanley controller started (26.R1 public protocols)")
    print("  path: {} ({} points)".format(os.path.abspath(arguments.path), len(points)))
    for kind, port in receive_channels:
        print("  {} receive: {}:{}".format(kind, arguments.bind_ip, port))
    print("  control destination: {}:{} (longCmdType 1 only)".format(*destination))

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
                            if localizer.add_gps(
                                received,
                                x_m,
                                y_m,
                                z_m if measurement.altitude_m is not None else None,
                                measurement.speed_mps,
                                measurement.course_deg,
                            ):
                                latest_gps_time = received
                    elif key.data == "imu":
                        measurement = parse_imu_packet(packet)
                        yaw = arguments.imu_yaw_sign * quaternion_to_yaw(
                            measurement.orientation_xyzw
                        ) + math.radians(arguments.imu_yaw_offset_deg)
                        gyro_z = (
                            arguments.imu_yaw_sign
                            * measurement.angular_velocity_radps[2]
                        )
                        localizer.add_imu(received, yaw, gyro_z)
                        latest_imu_time = received
                    elif key.data == "status":
                        status = parse_competition_vehicle_status(packet)
                        # Competition Status position/yaw are intentionally not
                        # used: localization must still reflect GPS/IMU noise.
                        status_speed_mps = abs(status.signed_velocity_kmh) / 3.6
                        latest_status_time = received
                    else:
                        collision = parse_collision_data_26r1(packet)
                        if collision.collision_detected:
                            collision_brake_until = max(
                                collision_brake_until,
                                received + arguments.collision_brake_seconds,
                            )
                            collided_ids = [
                                item.object_id for item in collision.collided_objects
                            ]
                            print(
                                "Collision detected (object ids {}); braking".format(
                                    collided_ids
                                ),
                                file=sys.stderr,
                            )
                except packet_errors as error:
                    invalid_counts[key.data] += 1
                    count = invalid_counts[key.data]
                    if count <= 3 or count % 100 == 0:
                        print(
                            "Ignored invalid {} packet: {}".format(key.data, error),
                            file=sys.stderr,
                        )

            now = time.monotonic()
            if now < next_control:
                continue
            next_control = now + period
            localization_fresh = (
                latest_gps_time is not None
                and latest_imu_time is not None
                and now - latest_gps_time <= arguments.gps_timeout
                and now - latest_imu_time <= arguments.imu_timeout
            )
            state = localizer.state_at(now) if localization_fresh else None
            collision_active = now < collision_brake_until

            if state is None or collision_active:
                speed_controller.reset()
                command = brake_command()
                result = None
                target_speed_mps = 0.0
                measured_speed_mps = 0.0 if state is None else state.speed_mps
            else:
                result = controller.compute(
                    state.x_m, state.y_m, state.z_m, state.yaw_rad, state.speed_mps
                )
                status_fresh = (
                    latest_status_time is not None
                    and now - latest_status_time <= arguments.status_timeout
                )
                measured_speed_mps = (
                    status_speed_mps if status_fresh else state.speed_mps
                )
                if result.goal_reached:
                    speed_controller.reset()
                    target_speed_mps = 0.0
                    command = brake_command()
                else:
                    normalized = arguments.morai_steer_sign * (
                        result.steering_rad / controller.max_steering_rad
                    )
                    curve_scale = max(0.35, 1.0 - 0.55 * abs(normalized))
                    end_scale = max(
                        0.25, min(1.0, result.remaining_distance_m / 15.0)
                    )
                    path_speed_mps = (
                        arguments.target_speed_kmh / 3.6
                        if result.target_speed_mps is None
                        else result.target_speed_mps
                    )
                    target_speed_mps = path_speed_mps * min(
                        curve_scale, end_scale
                    )
                    accel, brake = speed_controller.compute(
                        target_speed_mps, measured_speed_mps, now
                    )
                    command = pedal_command(accel, brake, normalized)
            control_socket.sendto(encode_ego_ctrl_cmd_26r1(command), destination)

            if now - last_log >= 1.0:
                last_log = now
                if collision_active:
                    print("Collision brake active")
                elif state is None:
                    print("Waiting for fresh GPS/IMU; brake command active")
                else:
                    print(
                        "pos=({:.2f}, {:.2f}, {:.2f}) speed={:.2f}/{:.2f}m/s "
                        "cte={:+.2f}m steer={:+.2f}deg pedal=({:.2f},{:.2f}) "
                        "remain={:.1f}m{}".format(
                            state.x_m,
                            state.y_m,
                            state.z_m,
                            measured_speed_mps,
                            target_speed_mps,
                            result.cross_track_error_m,
                            math.degrees(result.steering_rad),
                            command.accel,
                            command.brake,
                            result.remaining_distance_m,
                            " GOAL" if result.goal_reached else "",
                        )
                    )
    except KeyboardInterrupt:
        print("\nStopping controller and applying brake...")
    finally:
        stop_packet = encode_ego_ctrl_cmd_26r1(brake_command())
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
