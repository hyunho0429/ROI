"""Shared standalone UDP runtime for MORAI Stanley localization variants."""

import argparse
import math
import os
import selectors
import socket
import sys
import time

from path_planning.coordinates import GpsToMapEnu, MapProjection
from path_planning.localization_dead_reckoning import SpeedAidedDeadReckoning
from path_planning.localization_ins import InsErrorStateEkf
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
from path_planning.morai_udp_imu import ImuPacketError, parse_imu_packet
from path_planning.stanley_controller import StanleyController, load_path_csv


PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_PATH = os.path.join(PACKAGE_DIR, "data", "morai_global_path.csv")
DEFAULT_GLOBAL_INFO = os.path.join(
    PACKAGE_DIR, "mgeo", "R_KR_PR_K-city_2025", "global_info.json"
)


def _receiver(bind_ip, port):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    udp_socket.bind((bind_ip, port))
    udp_socket.setblocking(False)
    return udp_socket


def _projection(arguments):
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


def argument_parser(localization_mode):
    parser = argparse.ArgumentParser(
        description=(
            "MORAI UDP Stanley controller using {} localization".format(
                "15-state INS ESKF"
                if localization_mode == "ins"
                else "vehicle-speed dead reckoning"
            )
        )
    )
    parser.add_argument("--path", default=DEFAULT_PATH, help="map-origin ENU path CSV")
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--gps-port", type=int, default=9100)
    parser.add_argument("--imu-port", type=int, default=9101)
    parser.add_argument("--competition-status-port", type=int, default=3315)
    parser.add_argument("--collision-port", type=int, default=5678)
    parser.add_argument("--control-ip", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=9090)
    parser.add_argument("--control-rate-hz", type=float, default=20.0)
    parser.add_argument("--target-speed-kmh", type=float, default=15.0)
    parser.add_argument("--stanley-gain", type=float, default=1.2)
    parser.add_argument("--softening-speed", type=float, default=1.0)
    parser.add_argument("--max-steering-deg", type=float, default=36.25)
    parser.add_argument("--control-point-offset", type=float, default=0.0)
    parser.add_argument(
        "--morai-steer-sign", type=float, choices=(-1.0, 1.0), default=-1.0
    )
    parser.add_argument("--imu-timeout", type=float, default=0.5)
    parser.add_argument("--status-timeout", type=float, default=0.5)
    parser.add_argument(
        "--max-gps-outage",
        type=float,
        default=120.0,
        help="maximum seconds to continue aided navigation without a valid GPS fix",
    )
    parser.add_argument("--collision-brake-seconds", type=float, default=3.0)
    parser.add_argument("--gps-position-sigma", type=float, default=1.5)
    parser.add_argument("--gps-altitude-sigma", type=float, default=3.0)
    parser.add_argument("--gps-speed-sigma", type=float, default=0.8)
    parser.add_argument("--imu-orientation-sigma-deg", type=float, default=4.0)
    parser.add_argument("--vehicle-speed-sigma", type=float, default=0.25)
    parser.add_argument("--speed-kp", type=float, default=0.12)
    parser.add_argument("--speed-ki", type=float, default=0.04)
    parser.add_argument("--max-accel-pedal", type=float, default=0.65)
    parser.add_argument("--max-brake-pedal", type=float, default=0.8)
    parser.add_argument("--global-info", default=DEFAULT_GLOBAL_INFO)
    parser.add_argument("--utm-crs", default=None)
    parser.add_argument("--utm-origin-x", type=float, default=None)
    parser.add_argument("--utm-origin-y", type=float, default=None)
    parser.add_argument("--utm-origin-z", type=float, default=None)
    if localization_mode == "ins":
        parser.add_argument("--accel-noise-sigma", type=float, default=0.25)
        parser.add_argument("--gyro-noise-sigma-degps", type=float, default=0.8)
        parser.add_argument("--accel-bias-walk-sigma", type=float, default=0.02)
        parser.add_argument("--gyro-bias-walk-sigma-degps", type=float, default=0.03)
        parser.add_argument("--nhc-lateral-sigma", type=float, default=0.35)
        parser.add_argument("--nhc-vertical-sigma", type=float, default=0.25)
    else:
        parser.add_argument("--dr-position-drift-sigma", type=float, default=0.25)
        parser.add_argument("--orientation-correction-gain", type=float, default=0.12)
        parser.add_argument("--gyro-bias-gain", type=float, default=0.002)
    return parser


def _validate(arguments):
    receive_ports = (
        arguments.gps_port,
        arguments.imu_port,
        arguments.competition_status_port,
        arguments.collision_port,
    )
    for value in receive_ports + (arguments.control_port,):
        if not 1 <= value <= 65535:
            raise ValueError("UDP ports must be between 1 and 65535")
    if len(receive_ports) != len(set(receive_ports)):
        raise ValueError("GPS, IMU, status and collision ports must be distinct")
    positive_names = (
        "control_rate_hz",
        "imu_timeout",
        "status_timeout",
        "max_gps_outage",
        "gps_position_sigma",
        "gps_altitude_sigma",
        "vehicle_speed_sigma",
    )
    for name in positive_names:
        if getattr(arguments, name) <= 0.0:
            raise ValueError("{} must be positive".format(name))
    if arguments.target_speed_kmh < 0.0:
        raise ValueError("target-speed-kmh cannot be negative")
    for name in ("max_accel_pedal", "max_brake_pedal"):
        if not 0.0 <= getattr(arguments, name) <= 1.0:
            raise ValueError("{} must be between 0 and 1".format(name))


def _localizer(localization_mode, arguments):
    if localization_mode == "ins":
        return InsErrorStateEkf(
            gps_position_sigma_m=arguments.gps_position_sigma,
            gps_altitude_sigma_m=arguments.gps_altitude_sigma,
            gps_speed_sigma_mps=arguments.gps_speed_sigma,
            imu_orientation_sigma_deg=arguments.imu_orientation_sigma_deg,
            gyro_noise_sigma_degps=arguments.gyro_noise_sigma_degps,
            accel_noise_sigma_mps2=arguments.accel_noise_sigma,
            gyro_bias_walk_sigma_degps=arguments.gyro_bias_walk_sigma_degps,
            accel_bias_walk_sigma_mps2=arguments.accel_bias_walk_sigma,
            vehicle_speed_sigma_mps=arguments.vehicle_speed_sigma,
            nhc_lateral_sigma_mps=arguments.nhc_lateral_sigma,
            nhc_vertical_sigma_mps=arguments.nhc_vertical_sigma,
        )
    return SpeedAidedDeadReckoning(
        gps_position_sigma_m=arguments.gps_position_sigma,
        gps_altitude_sigma_m=arguments.gps_altitude_sigma,
        position_drift_sigma_mps=arguments.dr_position_drift_sigma,
        orientation_correction_gain=arguments.orientation_correction_gain,
        gyro_bias_gain=arguments.gyro_bias_gain,
    )


def run(localization_mode, arguments):
    _validate(arguments)
    projection = _projection(arguments)
    points = load_path_csv(arguments.path, gps_projection=projection)
    stanley = StanleyController(
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
    localizer = _localizer(localization_mode, arguments)
    converter = GpsToMapEnu(projection)

    selector = selectors.DefaultSelector()
    receive_sockets = []
    channels = (
        ("gps", arguments.gps_port),
        ("imu", arguments.imu_port),
        ("status", arguments.competition_status_port),
        ("collision", arguments.collision_port),
    )
    for name, port in channels:
        udp_socket = _receiver(arguments.bind_ip, port)
        selector.register(udp_socket, selectors.EVENT_READ, name)
        receive_sockets.append(udp_socket)
    control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    control_destination = (arguments.control_ip, arguments.control_port)

    latest_gps_time = latest_imu_time = latest_status_time = None
    status_speed_mps = 0.0
    collision_brake_until = 0.0
    invalid_counts = {name: 0 for name, _port in channels}
    packet_errors = (
        GpsPacketError,
        ImuPacketError,
        CompetitionStatusPacketError,
        CollisionPacketError,
    )
    period = 1.0 / arguments.control_rate_hz
    next_control = time.monotonic()
    last_log = 0.0

    print("MORAI Stanley {} controller started".format(localization_mode.upper()))
    print("  path: {} ({} points)".format(os.path.abspath(arguments.path), len(points)))
    for name, port in channels:
        print("  {} receive: {}:{}".format(name, arguments.bind_ip, port))
    print("  control: {}:{} (longCmdType 1)".format(*control_destination))
    print("  maximum GPS outage: {:.1f} s".format(arguments.max_gps_outage))

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
                        localizer.add_imu(
                            received,
                            measurement.orientation_xyzw,
                            measurement.angular_velocity_radps,
                            measurement.linear_acceleration_mps2,
                        )
                        latest_imu_time = received
                    elif key.data == "status":
                        status = parse_competition_vehicle_status(packet)
                        signed_speed_mps = status.signed_velocity_kmh / 3.6
                        localizer.add_vehicle_speed(received, signed_speed_mps)
                        status_speed_mps = abs(signed_speed_mps)
                        latest_status_time = received
                    else:
                        collision = parse_collision_data_26r1(packet)
                        if collision.collision_detected:
                            collision_brake_until = max(
                                collision_brake_until,
                                received + arguments.collision_brake_seconds,
                            )
                            print("Collision detected; braking", file=sys.stderr)
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
            gps_outage = (
                math.inf if latest_gps_time is None else now - latest_gps_time
            )
            sensor_fresh = (
                latest_imu_time is not None
                and now - latest_imu_time <= arguments.imu_timeout
                and latest_status_time is not None
                and now - latest_status_time <= arguments.status_timeout
                and gps_outage <= arguments.max_gps_outage
            )
            state = localizer.state_at(now) if sensor_fresh else None
            collision_active = now < collision_brake_until

            if state is None or collision_active:
                speed_controller.reset()
                command = brake_command()
                result = None
                target_speed_mps = 0.0
            else:
                result = stanley.compute(
                    state.x_m, state.y_m, state.z_m, state.yaw_rad, state.speed_mps
                )
                if result.goal_reached:
                    speed_controller.reset()
                    command = brake_command()
                    target_speed_mps = 0.0
                else:
                    normalized = arguments.morai_steer_sign * (
                        result.steering_rad / stanley.max_steering_rad
                    )
                    curve_scale = max(0.35, 1.0 - 0.55 * abs(normalized))
                    end_scale = max(
                        0.25, min(1.0, result.remaining_distance_m / 15.0)
                    )
                    target_speed_mps = (
                        arguments.target_speed_kmh
                        * min(curve_scale, end_scale)
                        / 3.6
                    )
                    accel, brake = speed_controller.compute(
                        target_speed_mps, status_speed_mps, now
                    )
                    command = pedal_command(accel, brake, normalized)
            control_socket.sendto(
                encode_ego_ctrl_cmd_26r1(command), control_destination
            )

            if now - last_log >= 1.0:
                last_log = now
                if collision_active:
                    print("Collision brake active")
                elif state is None:
                    print(
                        "Waiting/stale sensors: gps_outage={:.1f}s; brake active".format(
                            gps_outage
                        )
                    )
                else:
                    gps_label = (
                        "GPS" if gps_outage <= 1.0 else "GPS-OUT {:.1f}s".format(gps_outage)
                    )
                    print(
                        "{} pos=({:.2f},{:.2f},{:.2f}) speed={:.2f}/{:.2f} "
                        "cte={:+.2f} steer={:+.2f}deg remain={:.1f}m".format(
                            gps_label,
                            state.x_m,
                            state.y_m,
                            state.z_m,
                            status_speed_mps,
                            target_speed_mps,
                            result.cross_track_error_m,
                            math.degrees(result.steering_rad),
                            result.remaining_distance_m,
                        )
                    )
    except KeyboardInterrupt:
        print("\nStopping controller and applying brake...")
    finally:
        stop_packet = encode_ego_ctrl_cmd_26r1(brake_command())
        for _ in range(5):
            control_socket.sendto(stop_packet, control_destination)
            time.sleep(0.02)
        selector.close()
        for udp_socket in receive_sockets:
            udp_socket.close()
        control_socket.close()


def main(localization_mode, argv=None):
    parser = argument_parser(localization_mode)
    run(localization_mode, parser.parse_args(argv))
