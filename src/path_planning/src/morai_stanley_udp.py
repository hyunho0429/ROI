#!/usr/bin/env python3
"""Run Stanley tracking with MORAI competition UDP interfaces."""

import argparse
import math
import os
import selectors
import socket
import sys
import time

from path_planning.coordinates import GpsToMapEnu, GpsToRecordedLocalEnu, MapProjection
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
    external_control_ready,
    pedal_command,
)
from path_planning.morai_udp_gps import GpsPacketError, parse_nmea_datagram
from path_planning.morai_udp_imu import ImuPacketError, parse_imu_packet, quaternion_to_yaw
from path_planning.stanley_controller import (
    StanleyController,
    SteeringCommandFilter,
    load_path_csv,
    load_recorded_path_origin,
)


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
    parser.add_argument(
        "--path", default=DEFAULT_PATH, help="ENU CSV or MORAI GPS sensor path file"
    )
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--gps-port", type=int, default=3001)
    parser.add_argument("--imu-port", type=int, default=4001)
    parser.add_argument("--competition-status-port", type=int, default=909)
    parser.add_argument("--collision-port", type=int, default=5678)
    parser.add_argument("--control-ip", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=9090)
    parser.add_argument("--control-rate-hz", type=float, default=20.0)
    parser.add_argument("--target-speed-kmh", type=float, default=10.0)
    parser.add_argument("--stanley-gain", type=float, default=0.22)
    parser.add_argument("--softening-speed", type=float, default=3.0)
    parser.add_argument("--max-steering-deg", type=float, default=21.77)
    parser.add_argument("--vehicle-max-steering-deg", type=float, default=36.25)
    parser.add_argument(
        "--control-point-offset",
        type=float,
        default=3.0,
        help="front-axle control point offset from the localized vehicle position",
    )
    parser.add_argument("--heading-error-gain", type=float, default=1.0)
    parser.add_argument("--cross-track-error-gain", type=float, default=0.55)
    parser.add_argument("--cross-track-deadband", type=float, default=0.05)
    parser.add_argument("--minimum-waypoint-spacing", type=float, default=0.5)
    parser.add_argument("--waypoint-smoothing-window", type=int, default=9)
    parser.add_argument("--target-search-window", type=int, default=50)
    parser.add_argument("--allow-target-backtrack", action="store_true")
    parser.add_argument("--steering-filter-alpha", type=float, default=0.25)
    parser.add_argument("--max-steering-rate-radps", type=float, default=0.4)
    parser.add_argument(
        "--morai-steer-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=1.0,
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
    parser.add_argument("--speed-kp", type=float, default=0.35)
    parser.add_argument("--speed-ki", type=float, default=0.04)
    parser.add_argument("--max-accel-pedal", type=float, default=1.0)
    parser.add_argument("--max-brake-pedal", type=float, default=1.0)
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
    for name in ("max_steering_deg", "vehicle_max_steering_deg", "softening_speed"):
        if getattr(arguments, name) <= 0.0:
            raise ValueError("{} must be positive".format(name))
    for name in (
        "control_point_offset",
        "cross_track_deadband",
        "minimum_waypoint_spacing",
        "max_steering_rate_radps",
    ):
        if getattr(arguments, name) < 0.0:
            raise ValueError("{} cannot be negative".format(name))
    if not 0.0 <= arguments.steering_filter_alpha <= 1.0:
        raise ValueError("steering-filter-alpha must be between 0 and 1")
    if arguments.waypoint_smoothing_window < 1:
        raise ValueError("waypoint-smoothing-window must be at least 1")
    if arguments.target_search_window < 1:
        raise ValueError("target-search-window must be at least 1")
    if arguments.collision_brake_seconds < 0.0:
        raise ValueError("collision-brake-seconds cannot be negative")
    for name in ("max_accel_pedal", "max_brake_pedal"):
        if not 0.0 <= getattr(arguments, name) <= 1.0:
            raise ValueError("{} must be between 0 and 1".format(name))


def run(arguments):
    _validate(arguments)
    projection = _projection_from_arguments(arguments)
    recorded_origin = load_recorded_path_origin(arguments.path)
    points = load_path_csv(arguments.path, gps_projection=projection)
    controller = StanleyController(
        points,
        gain=arguments.stanley_gain,
        softening_speed_mps=arguments.softening_speed,
        max_steering_deg=arguments.max_steering_deg,
        control_point_offset_m=arguments.control_point_offset,
        heading_error_gain=arguments.heading_error_gain,
        cross_track_error_gain=arguments.cross_track_error_gain,
        cross_track_deadband_m=arguments.cross_track_deadband,
        minimum_waypoint_spacing_m=arguments.minimum_waypoint_spacing,
        waypoint_smoothing_window=arguments.waypoint_smoothing_window,
        search_back_segments=5 if arguments.allow_target_backtrack else 0,
        search_forward_segments=arguments.target_search_window,
    )
    steering_filter = SteeringCommandFilter(
        alpha=arguments.steering_filter_alpha,
        max_rate_radps=arguments.max_steering_rate_radps,
        max_abs_rad=controller.max_steering_rad,
    )
    speed_controller = PedalSpeedController(
        kp=arguments.speed_kp,
        ki=arguments.speed_ki,
        max_accel=arguments.max_accel_pedal,
        max_brake=arguments.max_brake_pedal,
    )
    converter = (
        GpsToMapEnu(projection)
        if recorded_origin is None
        else GpsToRecordedLocalEnu(recorded_origin)
    )
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
    status_ctrl_mode = status_gear = None
    last_drive_state = None
    collision_brake_until = 0.0
    invalid_counts = {kind: 0 for kind, _port in receive_channels}
    packet_errors = (
        GpsPacketError,
        ImuPacketError,
        CompetitionStatusPacketError,
        CollisionPacketError,
    )

    print("MORAI competition Stanley controller started (26.R1 public protocols)")
    print(
        "  path: {} ({} -> {} points after spacing/smoothing)".format(
            os.path.abspath(arguments.path),
            controller.original_point_count,
            len(controller.points),
        )
    )
    if recorded_origin is not None:
        print(
            "  coordinate frame: recorded GPS origin "
            "lat={:.8f}, lon={:.8f}, alt={:.3f}".format(
                recorded_origin.latitude_deg,
                recorded_origin.longitude_deg,
                recorded_origin.altitude_m,
            )
        )
    else:
        print("  coordinate frame: MGeo map-origin ENU")
    for kind, port in receive_channels:
        print("  {} receive: {}:{}".format(kind, arguments.bind_ip, port))
    print("  control destination: {}:{} (longCmdType 1 only)".format(*destination))
    print(
        "  Stanley: front axle {:.2f} m, no look-ahead target, "
        "fixed speed {:.1f} km/h".format(
            arguments.control_point_offset, arguments.target_speed_kmh
        )
    )
    print("  requesting AV-ExternalCtrl (ctrl_mode=2) and Drive (gear=4)")
    takeover_packet = encode_ego_ctrl_cmd_26r1(brake_command())
    for _ in range(3):
        control_socket.sendto(takeover_packet, destination)
        time.sleep(0.02)

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
                        status_ctrl_mode = status.ctrl_mode
                        status_gear = status.gear
                        latest_status_time = received
                        drive_state = (status_ctrl_mode, status_gear)
                        if drive_state != last_drive_state:
                            print(
                                "Competition control state: ctrl_mode={} {}, "
                                "gear={} {}".format(
                                    status_ctrl_mode,
                                    "(AV-ExternalCtrl)"
                                    if status_ctrl_mode == 2
                                    else "(not external)",
                                    status_gear,
                                    "(D)" if status_gear == 4 else "(not D)",
                                )
                            )
                            last_drive_state = drive_state
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
            gps_age = math.inf if latest_gps_time is None else now - latest_gps_time
            imu_age = math.inf if latest_imu_time is None else now - latest_imu_time
            localization_fresh = (
                gps_age <= arguments.gps_timeout
                and imu_age <= arguments.imu_timeout
            )
            state = localizer.state_at(now) if localization_fresh else None
            collision_active = now < collision_brake_until
            drive_control_ready = external_control_ready(
                status_ctrl_mode, status_gear
            )

            if state is None or collision_active or not drive_control_ready:
                speed_controller.reset()
                steering_filter.reset()
                command = brake_command()
                result = None
                target_speed_mps = 0.0
                measured_speed_mps = 0.0 if state is None else state.speed_mps
                raw_steering_rad = filtered_steering_rad = 0.0
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
                    steering_filter.reset()
                    target_speed_mps = 0.0
                    command = brake_command()
                    raw_steering_rad = filtered_steering_rad = 0.0
                else:
                    raw_steering_rad = result.steering_rad
                    filtered_steering_rad = steering_filter.update(
                        raw_steering_rad, now
                    )
                    normalized = arguments.morai_steer_sign * (
                        filtered_steering_rad
                        / math.radians(arguments.vehicle_max_steering_deg)
                    )
                    normalized = max(-1.0, min(1.0, normalized))
                    target_speed_mps = arguments.target_speed_kmh / 3.6
                    accel, brake = speed_controller.compute(
                        target_speed_mps, measured_speed_mps, now
                    )
                    command = pedal_command(accel, brake, normalized)
            control_socket.sendto(encode_ego_ctrl_cmd_26r1(command), destination)

            if now - last_log >= 1.0:
                last_log = now
                if collision_active:
                    print("Collision brake active")
                elif state is not None and not drive_control_ready:
                    print(
                        "Requesting AV-ExternalCtrl/D: current ctrl_mode={}, "
                        "gear={}; takeover brake command is being sent".format(
                            "never" if status_ctrl_mode is None else status_ctrl_mode,
                            "never" if status_gear is None else status_gear,
                        )
                    )
                elif state is None:
                    print(
                        "Waiting/stale sensors: GPS={}, IMU={} "
                        "(limits: GPS {:.1f}s, IMU {:.1f}s); brake active".format(
                            "never" if math.isinf(gps_age) else "{:.2f}s".format(gps_age),
                            "never" if math.isinf(imu_age) else "{:.2f}s".format(imu_age),
                            arguments.gps_timeout,
                            arguments.imu_timeout,
                        )
                    )
                else:
                    print(
                        "pos=({:.2f}, {:.2f}, {:.2f}) speed={:.2f}/{:.2f}m/s "
                        "yaw/path={:+.1f}/{:+.1f}deg herr={:+.1f}deg "
                        "cte={:+.2f}m steer(raw/filt)={:+.2f}/{:+.2f}deg "
                        "cmd=({:.2f},{:+.2f},{:.2f}) remain={:.1f}m{}".format(
                            state.x_m,
                            state.y_m,
                            state.z_m,
                            measured_speed_mps,
                            target_speed_mps,
                            math.degrees(state.yaw_rad),
                            math.degrees(result.path_yaw_rad),
                            math.degrees(result.heading_error_rad),
                            result.cross_track_error_m,
                            math.degrees(raw_steering_rad),
                            math.degrees(filtered_steering_rad),
                            command.accel,
                            command.steering_normalized,
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
