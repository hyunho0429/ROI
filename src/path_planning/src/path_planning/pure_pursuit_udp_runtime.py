"""Shared standalone UDP runtime for MORAI Stanley localization variants."""

import argparse
import math
import os
import selectors
import socket
import sys
import time

from path_planning.coordinates import GpsToMapEnu, GpsToRecordedLocalEnu, MapProjection
from path_planning.localization_dead_reckoning import SpeedAidedDeadReckoning
from path_planning.localization_ins import InsErrorStateEkf
from path_planning.longitudinal_controller import PedalSpeedController
from path_planning.morai_competition_config import (
    BIND_IP,
    COLLISION_HOST_PORT,
    COLLISION_PORT,
    COMPETITION_STATUS_HOST_PORT,
    COMPETITION_STATUS_PORT,
    CONTROL_DESTINATION_PORT,
    CONTROL_IP,
    CONTROL_PORT,
    GPS_PORT,
    IMU_PORT,
    TARGET_SPEED_KMH,
    VEHICLE_WHEELBASE_M,
)
from path_planning.morai_udp_collision_data import (
    CollisionPacketError,
    parse_collision_data,
)
from path_planning.morai_udp_competition_status import (
    CompetitionStatusPacketError,
    parse_competition_vehicle_status,
)
from path_planning.morai_udp_ctrl_cmd import (
    CONTROL_PROTOCOLS,
    brake_command,
    encode_ego_ctrl_cmd,
    external_control_ready,
    pedal_command,
)
from path_planning.morai_udp_gps import GpsPacketError, parse_nmea_datagram
from path_planning.morai_udp_imu import ImuPacketError, parse_imu_packet
from path_planning.stanley_controller import (
    PathPoint,
    StanleyController,
    SteeringCommandFilter,
    load_gps_path_projection,
    load_path_csv,
    load_recorded_path_origin,
)


PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_PATH = os.path.join(
    PACKAGE_DIR, "data", "2026_molit_comp_global_path.txt"
)
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


def _curve_limited_target_speed_mps(arguments, base_target_speed_mps, steering_rad):
    """Reduce only speed in curves, using Stanley steering demand as curvature cue."""
    steering_deg = abs(math.degrees(steering_rad))
    if steering_deg >= arguments.sharp_curve_steering_deg:
        return min(
            base_target_speed_mps, arguments.sharp_curve_speed_kmh / 3.6
        ), "sharp"
    if steering_deg >= arguments.medium_curve_steering_deg:
        return min(
            base_target_speed_mps, arguments.medium_curve_speed_kmh / 3.6
        ), "medium"
    return base_target_speed_mps, "straight"


def _offset_path_points_laterally(points, offset_m):
    """Shift path points by a signed left-normal offset in the path frame."""
    offset = float(offset_m)
    if abs(offset) < 1e-6:
        return points
    shifted = []
    last_index = len(points) - 1
    for index, point in enumerate(points):
        if index == 0:
            before, after = points[index], points[index + 1]
        elif index == last_index:
            before, after = points[index - 1], points[index]
        else:
            before, after = points[index - 1], points[index + 1]
        dx = after.x_m - before.x_m
        dy = after.y_m - before.y_m
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            shifted.append(point)
            continue
        left_x = -dy / norm
        left_y = dx / norm
        shifted.append(
            PathPoint(
                point.x_m + offset * left_x,
                point.y_m + offset * left_y,
                point.z_m,
                point.target_speed_mps,
            )
        )
    return shifted


def _startup_lane_safety_bias_rad(arguments, route_progress_m, steering_rad):
    """Nudge left only in the initial near-straight segment."""
    if route_progress_m is None:
        return 0.0
    if route_progress_m >= arguments.startup_lane_bias_distance_m:
        return 0.0
    if abs(math.degrees(steering_rad)) > arguments.startup_lane_bias_max_steering_deg:
        return 0.0
    fade = 1.0 - route_progress_m / arguments.startup_lane_bias_distance_m
    return math.radians(arguments.startup_lane_bias_deg) * max(0.0, min(1.0, fade))


def _startup_straight_steering_guard_rad(arguments, route_progress_m, steering_rad):
    """Suppress only small initial right steering drift before the first real turn."""
    if route_progress_m is None:
        return steering_rad
    if route_progress_m >= arguments.startup_steering_guard_distance_m:
        return steering_rad
    if abs(math.degrees(steering_rad)) > arguments.startup_steering_guard_deg:
        return steering_rad
    return max(0.0, steering_rad)


def _turn_steering_scale(arguments, steering_rad):
    """Soften large steering commands equally for left and right turns."""
    if abs(math.degrees(steering_rad)) < arguments.turn_steering_scale_min_deg:
        return 1.0
    return arguments.turn_steering_scale


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
    parser.add_argument(
        "--path",
        default=DEFAULT_PATH,
        help="ENU CSV/TXT or MORAI GPS sensor path file",
    )
    parser.add_argument("--bind-ip", default=BIND_IP)
    parser.add_argument("--gps-port", type=int, default=GPS_PORT)
    parser.add_argument("--imu-port", type=int, default=IMU_PORT)
    parser.add_argument(
        "--competition-status-host-port",
        type=int,
        default=COMPETITION_STATUS_HOST_PORT,
        help="MORAI source/Host Port for Competition Vehicle Status",
    )
    parser.add_argument(
        "--competition-status-port", type=int, default=COMPETITION_STATUS_PORT
    )
    parser.add_argument(
        "--collision-host-port",
        type=int,
        default=COLLISION_HOST_PORT,
        help="MORAI source/Host Port for CollisionData",
    )
    parser.add_argument(
        "--collision-port",
        type=int,
        default=COLLISION_PORT,
        help="algorithm Destination Port for CollisionData",
    )
    parser.add_argument("--control-ip", default=CONTROL_IP)
    parser.add_argument(
        "--control-port",
        type=int,
        default=CONTROL_PORT,
        help="MORAI Host Port for Ego Ctrl Cmd",
    )
    parser.add_argument(
        "--control-source-port",
        type=int,
        default=CONTROL_DESTINATION_PORT,
        help="algorithm source/Destination Port for Ego Ctrl Cmd",
    )
    parser.add_argument(
        "--control-protocol",
        choices=CONTROL_PROTOCOLS,
        default="25s4",
        help="Ego Ctrl Cmd wire layout (25s4: 55 bytes, 26r1: 59 bytes)",
    )
    parser.add_argument(
        "--control-rate-hz",
        type=float,
        default=30.0,
        help="control loop rate; main branch PID was configured for 30 Hz",
    )
    parser.add_argument("--target-speed-kmh", type=float, default=TARGET_SPEED_KMH)
    parser.add_argument("--medium-curve-speed-kmh", type=float, default=25.0)
    parser.add_argument("--sharp-curve-speed-kmh", type=float, default=18.0)
    parser.add_argument("--medium-curve-steering-deg", type=float, default=5.0)
    parser.add_argument("--sharp-curve-steering-deg", type=float, default=9.0)
    parser.add_argument(
        "--wheelbase",
        type=float,
        default=VEHICLE_WHEELBASE_M,
        help="deprecated compatibility option; Stanley uses control-point-offset",
    )
    parser.add_argument("--lookahead-distance", type=float, default=2.0)
    parser.add_argument("--lookahead-speed-gain", type=float, default=0.15)
    parser.add_argument("--minimum-lookahead", type=float, default=2.0)
    parser.add_argument("--maximum-lookahead", type=float, default=4.0)
    parser.add_argument("--stanley-gain", type=float, default=0.35)
    parser.add_argument("--softening-speed", type=float, default=2.2)
    parser.add_argument("--stanley-control-speed-floor-kmh", type=float, default=35.0)
    parser.add_argument("--path-lateral-offset-m", type=float, default=0.60)
    parser.add_argument("--heading-error-gain", type=float, default=0.74)
    parser.add_argument("--cross-track-error-gain", type=float, default=0.58)
    parser.add_argument("--cross-track-deadband", type=float, default=0.02)
    parser.add_argument("--goal-tolerance", type=float, default=2.0)
    parser.add_argument(
        "--max-steering-deg",
        type=float,
        default=16.0,
        help="Stanley controller steering limit in degrees",
    )
    parser.add_argument(
        "--vehicle-max-steering-deg",
        type=float,
        default=36.25,
        help="physical steering angle represented by MORAI command +/-1",
    )
    parser.add_argument(
        "--control-point-offset",
        type=float,
        default=1.5,
        help="front axle/control point offset from localization point",
    )
    parser.add_argument(
        "--heading-preview-distance",
        type=float,
        default=4.0,
        help="distance ahead of the nearest segment used for Stanley heading error",
    )
    parser.add_argument(
        "--heading-preview-start-distance",
        type=float,
        default=20.0,
        help="path progress before heading preview is allowed",
    )
    parser.add_argument(
        "--heading-preview-deadband-deg",
        type=float,
        default=2.0,
        help="disable heading preview for smaller preview-vs-nearest heading deltas",
    )
    parser.add_argument("--minimum-waypoint-spacing", type=float, default=0.5)
    parser.add_argument("--waypoint-smoothing-window", type=int, default=9)
    parser.add_argument("--target-search-window", type=int, default=50)
    parser.add_argument(
        "--allow-target-backtrack",
        action="store_true",
        help="allow the nearest segment search to move up to five segments backward",
    )
    parser.add_argument("--steering-filter-alpha", type=float, default=0.35)
    parser.add_argument("--max-steering-rate-radps", type=float, default=0.75)
    parser.add_argument("--startup-lane-bias-deg", type=float, default=0.0)
    parser.add_argument("--startup-lane-bias-distance-m", type=float, default=25.0)
    parser.add_argument("--startup-lane-bias-max-steering-deg", type=float, default=3.0)
    parser.add_argument("--startup-steering-guard-distance-m", type=float, default=15.0)
    parser.add_argument("--startup-steering-guard-deg", type=float, default=3.0)
    parser.add_argument("--turn-steering-scale", type=float, default=1.0)
    parser.add_argument("--turn-steering-scale-min-deg", type=float, default=3.0)
    parser.add_argument(
        "--morai-steer-sign", type=float, choices=(-1.0, 1.0), default=1.0
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
    parser.add_argument("--speed-kp", type=float, default=0.075)
    parser.add_argument("--speed-ki", type=float, default=0.0001)
    parser.add_argument("--speed-kd", type=float, default=0.025)
    parser.add_argument("--max-accel-pedal", type=float, default=1.0)
    parser.add_argument("--max-brake-pedal", type=float, default=1.0)
    parser.add_argument("--global-info", default=DEFAULT_GLOBAL_INFO)
    parser.add_argument("--utm-crs", default=None)
    parser.add_argument("--utm-origin-x", type=float, default=None)
    parser.add_argument("--utm-origin-y", type=float, default=None)
    parser.add_argument("--utm-origin-z", type=float, default=None)
    if localization_mode == "ins":
        parser.add_argument("--alignment-seconds", type=float, default=2.0)
        parser.add_argument("--alignment-min-samples", type=int, default=20)
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
    network_ports = receive_ports + (
        arguments.competition_status_host_port,
        arguments.collision_host_port,
        arguments.control_port,
        arguments.control_source_port,
    )
    for value in network_ports:
        if not 1 <= value <= 65535:
            raise ValueError("UDP ports must be between 1 and 65535")
    local_bind_ports = receive_ports + (arguments.control_source_port,)
    if len(local_bind_ports) != len(set(local_bind_ports)):
        raise ValueError(
            "GPS, IMU, status, collision and control source ports must be distinct"
        )
    positive_names = (
        "control_rate_hz",
        "imu_timeout",
        "status_timeout",
        "max_gps_outage",
        "gps_position_sigma",
        "gps_altitude_sigma",
        "vehicle_speed_sigma",
        "max_steering_deg",
        "vehicle_max_steering_deg",
        "wheelbase",
        "lookahead_distance",
        "minimum_lookahead",
        "maximum_lookahead",
        "softening_speed",
        "stanley_control_speed_floor_kmh",
        "heading_preview_distance",
        "heading_preview_start_distance",
        "medium_curve_speed_kmh",
        "sharp_curve_speed_kmh",
        "medium_curve_steering_deg",
        "sharp_curve_steering_deg",
        "startup_lane_bias_distance_m",
        "startup_lane_bias_max_steering_deg",
        "startup_steering_guard_distance_m",
        "startup_steering_guard_deg",
        "turn_steering_scale_min_deg",
        "goal_tolerance",
    )
    for name in positive_names:
        if getattr(arguments, name) <= 0.0:
            raise ValueError("{} must be positive".format(name))
    if arguments.target_speed_kmh < 0.0:
        raise ValueError("target-speed-kmh cannot be negative")
    if arguments.medium_curve_speed_kmh > arguments.target_speed_kmh:
        raise ValueError("medium-curve-speed-kmh must be <= target-speed-kmh")
    if arguments.sharp_curve_speed_kmh > arguments.medium_curve_speed_kmh:
        raise ValueError("sharp-curve-speed-kmh must be <= medium-curve-speed-kmh")
    if arguments.sharp_curve_steering_deg <= arguments.medium_curve_steering_deg:
        raise ValueError(
            "sharp-curve-steering-deg must be greater than medium-curve-steering-deg"
        )
    if not 0.0 < arguments.turn_steering_scale <= 1.0:
        raise ValueError("turn-steering-scale must be in (0, 1]")
    for name in (
        "lookahead_speed_gain",
        "stanley_gain",
        "heading_error_gain",
        "cross_track_error_gain",
        "cross_track_deadband",
        "heading_preview_deadband_deg",
        "startup_lane_bias_deg",
        "minimum_waypoint_spacing",
        "max_steering_rate_radps",
        "speed_kp",
        "speed_ki",
        "speed_kd",
    ):
        if getattr(arguments, name) < 0.0:
            raise ValueError("{} cannot be negative".format(name))
    for name in ("max_accel_pedal", "max_brake_pedal"):
        if getattr(arguments, name) <= 0.0:
            raise ValueError("{} must be positive".format(name))
    if not 0.0 <= arguments.steering_filter_alpha <= 1.0:
        raise ValueError("steering-filter-alpha must be between 0 and 1")
    if arguments.waypoint_smoothing_window < 1:
        raise ValueError("waypoint-smoothing-window must be at least 1")
    if arguments.target_search_window < 1:
        raise ValueError("target-search-window must be at least 1")
    if arguments.maximum_lookahead < arguments.minimum_lookahead:
        raise ValueError("maximum-lookahead must be >= minimum-lookahead")
    if not math.isfinite(arguments.control_point_offset):
        raise ValueError("control-point-offset must be finite")
    if not math.isfinite(arguments.path_lateral_offset_m):
        raise ValueError("path-lateral-offset-m must be finite")
    if hasattr(arguments, "alignment_seconds"):
        if arguments.alignment_seconds < 0.0:
            raise ValueError("alignment-seconds cannot be negative")
        if arguments.alignment_min_samples < 1:
            raise ValueError("alignment-min-samples must be at least 1")
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
            alignment_duration_s=arguments.alignment_seconds,
            alignment_min_samples=arguments.alignment_min_samples,
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
    csv_projection = load_gps_path_projection(arguments.path, projection)
    active_projection = csv_projection or projection
    recorded_origin = load_recorded_path_origin(arguments.path)
    points = load_path_csv(arguments.path, gps_projection=active_projection)
    points = _offset_path_points_laterally(points, arguments.path_lateral_offset_m)
    stanley = StanleyController(
        points,
        gain=arguments.stanley_gain,
        softening_speed_mps=arguments.softening_speed,
        max_steering_deg=arguments.max_steering_deg,
        control_point_offset_m=arguments.control_point_offset,
        heading_preview_distance_m=arguments.heading_preview_distance,
        heading_preview_start_distance_m=arguments.heading_preview_start_distance,
        heading_preview_deadband_rad=math.radians(arguments.heading_preview_deadband_deg),
        heading_error_gain=arguments.heading_error_gain,
        cross_track_error_gain=arguments.cross_track_error_gain,
        cross_track_deadband_m=arguments.cross_track_deadband,
        minimum_waypoint_spacing_m=arguments.minimum_waypoint_spacing,
        waypoint_smoothing_window=arguments.waypoint_smoothing_window,
        search_back_segments=5 if arguments.allow_target_backtrack else 0,
        search_forward_segments=arguments.target_search_window,
        goal_tolerance_m=arguments.goal_tolerance,
    )
    steering_filter = SteeringCommandFilter(
        alpha=arguments.steering_filter_alpha,
        max_rate_radps=arguments.max_steering_rate_radps,
        max_abs_rad=stanley.max_steering_rad,
    )
    speed_controller = PedalSpeedController(
        kp=arguments.speed_kp,
        ki=arguments.speed_ki,
        kd=arguments.speed_kd,
        nominal_dt=1.0 / arguments.control_rate_hz,
        max_accel=arguments.max_accel_pedal,
        max_brake=arguments.max_brake_pedal,
    )
    localizer = _localizer(localization_mode, arguments)
    converter = (
        GpsToMapEnu(active_projection)
        if recorded_origin is None
        else GpsToRecordedLocalEnu(recorded_origin)
    )

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
    try:
        control_socket.bind((arguments.bind_ip, arguments.control_source_port))
    except OSError as error:
        control_socket.close()
        selector.close()
        for udp_socket in receive_sockets:
            udp_socket.close()
        raise OSError(
            "cannot bind Ego Ctrl Cmd Destination/source {}:{} ({})".format(
                arguments.bind_ip, arguments.control_source_port, error
            )
        ) from error
    control_destination = (arguments.control_ip, arguments.control_port)
    encode_control = lambda command: encode_ego_ctrl_cmd(
        command, arguments.control_protocol
    )

    latest_gps_time = latest_imu_time = latest_status_time = None
    status_speed_mps = 0.0
    status_vel_x_kmh = 0.0
    status_wheelbase_m = arguments.wheelbase
    use_status_wheelbase_for_control_point = abs(
        arguments.control_point_offset - VEHICLE_WHEELBASE_M
    ) < 1e-6
    status_ctrl_mode = status_gear = None
    status_accel_pedal = status_brake_pedal = status_front_steer_deg = 0.0
    last_drive_state = None
    route_initial_remaining_m = None
    collision_brake_until = 0.0
    invalid_counts = {name: 0 for name, _port in channels}
    unexpected_source_counts = {"status": 0, "collision": 0}
    expected_source_ports = {
        "status": arguments.competition_status_host_port,
        "collision": arguments.collision_host_port,
    }
    packet_errors = (
        GpsPacketError,
        ImuPacketError,
        CompetitionStatusPacketError,
        CollisionPacketError,
    )
    period = 1.0 / arguments.control_rate_hz
    next_control = time.monotonic()
    last_log = 0.0

    print(
        "MORAI Stanley {} controller started".format(
            localization_mode.upper()
        )
    )
    print(
        "  path: {} ({} -> {} points after spacing/smoothing, lateral offset {:+.2f} m)".format(
            os.path.abspath(arguments.path),
            stanley.original_point_count,
            len(stanley.points),
            arguments.path_lateral_offset_m,
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
    elif csv_projection is not None:
        print(
            "  coordinate frame: GPS CSV UTM origin "
            "EastOffset={:.3f}, NorthOffset={:.3f}".format(
                csv_projection.origin_x_m, csv_projection.origin_y_m
            )
        )
    else:
        print("  coordinate frame: MGeo map-origin ENU")
    for name, port in channels:
        expected_source = expected_source_ports.get(name)
        if expected_source is None:
            print("  {} receive: destination {}:{}".format(name, arguments.bind_ip, port))
        else:
            print(
                "  {} receive: MORAI host/source *:{} -> destination {}:{}".format(
                    name, expected_source, arguments.bind_ip, port
                )
            )
    command_packet_size = len(encode_control(brake_command()))
    print(
        "  control: source {}:{} -> MORAI host {}:{} "
        "(protocol {}, {} bytes, longCmdType 1)".format(
            arguments.bind_ip,
            arguments.control_source_port,
            control_destination[0],
            control_destination[1],
            arguments.control_protocol,
            command_packet_size,
        )
    )
    if localization_mode == "ins":
        print("  localization: GPS/IMU/status-aided 15-state error-state EKF INS")
        print(
            "  alignment: hold brake for {:.1f}s (at least {} IMU samples)".format(
                arguments.alignment_seconds, arguments.alignment_min_samples
            )
        )
    else:
        print("  localization: GPS/IMU/status-aided dead reckoning")
    print(
        "  Stanley: front {:.2f} m, heading_preview={:.2f} m "
        "(start {:.1f} m, deadband {:.1f} deg), "
        "gain={:.3f}, softening={:.2f} m/s, "
        "control_speed_floor={:.1f} km/h, "
        "heading_gain={:.2f}, cte_gain={:.2f}, deadband={:.2f} m, "
        "fixed speed {:.1f} km/h".format(
            arguments.control_point_offset,
            arguments.heading_preview_distance,
            arguments.heading_preview_start_distance,
            arguments.heading_preview_deadband_deg,
            arguments.stanley_gain,
            arguments.softening_speed,
            arguments.stanley_control_speed_floor_kmh,
            arguments.heading_error_gain,
            arguments.cross_track_error_gain,
            arguments.cross_track_deadband,
            arguments.target_speed_kmh,
        )
    )
    print(
        "  curve speed planner: medium {:.1f} km/h at {:.1f} deg, "
        "sharp {:.1f} km/h at {:.1f} deg".format(
            arguments.medium_curve_speed_kmh,
            arguments.medium_curve_steering_deg,
            arguments.sharp_curve_speed_kmh,
            arguments.sharp_curve_steering_deg,
        )
    )
    print(
        "  steering smoothing: alpha={:.2f}, max_rate={:.2f} rad/s".format(
            arguments.steering_filter_alpha,
            arguments.max_steering_rate_radps,
        )
    )
    print(
        "  startup lane safety: left bias {:.2f} deg over {:.1f} m "
        "when |raw steer| <= {:.1f} deg".format(
            arguments.startup_lane_bias_deg,
            arguments.startup_lane_bias_distance_m,
            arguments.startup_lane_bias_max_steering_deg,
        )
    )
    print(
        "  startup steering guard: zero small right raw steer up to {:.1f} deg over {:.1f} m".format(
            arguments.startup_steering_guard_deg,
            arguments.startup_steering_guard_distance_m,
        )
    )
    print(
        "  turn steering limit: scale {:.2f} "
        "when |raw steer| >= {:.1f} deg".format(
            arguments.turn_steering_scale,
            arguments.turn_steering_scale_min_deg,
        )
    )
    print(
        "  longitudinal PID: Kp={:.6f}, Ki={:.6f}, Kd={:.6f} at {:.1f} Hz".format(
            arguments.speed_kp,
            arguments.speed_ki,
            arguments.speed_kd,
            arguments.control_rate_hz,
        )
    )
    print("  maximum GPS outage: {:.1f} s".format(arguments.max_gps_outage))
    print("  requesting AV-ExternalCtrl (ctrl_mode=2) and Drive (gear=4)")
    takeover_packet = encode_control(brake_command())
    for _ in range(3):
        control_socket.sendto(takeover_packet, control_destination)
        time.sleep(0.02)

    try:
        while True:
            now = time.monotonic()
            timeout = max(0.0, min(period, next_control - now))
            for key, _mask in selector.select(timeout):
                packet, sender = key.fileobj.recvfrom(65535)
                received = time.monotonic()
                expected_source = expected_source_ports.get(key.data)
                if expected_source is not None and sender[1] != expected_source:
                    unexpected_source_counts[key.data] += 1
                    count = unexpected_source_counts[key.data]
                    if count <= 3 or count % 100 == 0:
                        print(
                            "Warning: {} packet source port is {}, expected MORAI "
                            "Host Port {} (packet will still be parsed)".format(
                                key.data, sender[1], expected_source
                            ),
                            file=sys.stderr,
                        )
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
                        status_vel_x_kmh = status.velocity_kmh[0]
                        status_ctrl_mode = status.ctrl_mode
                        status_gear = status.gear
                        status_accel_pedal = status.accel_pedal
                        status_brake_pedal = status.brake_pedal
                        status_front_steer_deg = status.front_steer_deg
                        if status.wheelbase_m > 0.5:
                            status_wheelbase_m = status.wheelbase_m
                            if use_status_wheelbase_for_control_point:
                                # Stanley should use the front axle as the
                                # control point.  When the user keeps the
                                # wheelbase default, follow the live status
                                # wheelbase.  If an explicit offset is supplied
                                # for earlier turn-in, preserve that tuning.
                                stanley.control_point_offset_m = status_wheelbase_m
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
                        collision = parse_collision_data(packet)
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
            imu_age = math.inf if latest_imu_time is None else now - latest_imu_time
            status_age = (
                math.inf if latest_status_time is None else now - latest_status_time
            )
            sensor_fresh = (
                imu_age <= arguments.imu_timeout
                and status_age <= arguments.status_timeout
                and gps_outage <= arguments.max_gps_outage
            )
            state = localizer.state_at(now) if sensor_fresh else None
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
                raw_steering_rad = guarded_steering_rad = filtered_steering_rad = 0.0
                startup_bias_rad = 0.0
                steering_scale = 1.0
                normalized_steering = 0.0
                curve_speed_mode = "stop"
            else:
                stanley_control_speed_mps = max(
                    state.speed_mps,
                    arguments.stanley_control_speed_floor_kmh / 3.6,
                )
                result = stanley.compute(
                    state.x_m,
                    state.y_m,
                    state.z_m,
                    state.yaw_rad,
                    stanley_control_speed_mps,
                )
                if result.goal_reached:
                    speed_controller.reset()
                    steering_filter.reset()
                    command = brake_command()
                    target_speed_mps = 0.0
                    raw_steering_rad = guarded_steering_rad = filtered_steering_rad = 0.0
                    startup_bias_rad = 0.0
                    steering_scale = 1.0
                    normalized_steering = 0.0
                    curve_speed_mode = "goal"
                else:
                    raw_steering_rad = result.steering_rad
                    if route_initial_remaining_m is None:
                        route_initial_remaining_m = result.remaining_distance_m
                    route_progress_m = max(
                        0.0,
                        route_initial_remaining_m - result.remaining_distance_m,
                    )
                    guarded_steering_rad = _startup_straight_steering_guard_rad(
                        arguments, route_progress_m, raw_steering_rad
                    )
                    startup_bias_rad = _startup_lane_safety_bias_rad(
                        arguments, route_progress_m, guarded_steering_rad
                    )
                    steering_scale = _turn_steering_scale(arguments, guarded_steering_rad)
                    filtered_steering_rad = steering_filter.update(
                        guarded_steering_rad * steering_scale + startup_bias_rad, now
                    )
                    normalized_steering = arguments.morai_steer_sign * (
                        filtered_steering_rad
                        / math.radians(arguments.vehicle_max_steering_deg)
                    )
                    normalized_steering = max(
                        -1.0, min(1.0, normalized_steering)
                    )
                    base_target_speed_mps = (
                        arguments.target_speed_kmh / 3.6
                        if result.target_speed_mps is None
                        else result.target_speed_mps
                    )
                    target_speed_mps, curve_speed_mode = (
                        _curve_limited_target_speed_mps(
                            arguments, base_target_speed_mps, raw_steering_rad
                        )
                    )
                    accel, brake = speed_controller.compute(
                        target_speed_mps, state.speed_mps, now
                    )
                    command = pedal_command(
                        accel, brake, normalized_steering
                    )
            control_socket.sendto(
                encode_control(command), control_destination
            )

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
                        "Waiting/stale sensors: GPS={}, IMU={}, Competition={} "
                        "(limits: GPS {:.1f}s, IMU {:.1f}s, Competition {:.1f}s); "
                        "brake active".format(
                            "never" if math.isinf(gps_outage) else "{:.2f}s".format(gps_outage),
                            "never" if math.isinf(imu_age) else "{:.2f}s".format(imu_age),
                            "never" if math.isinf(status_age) else "{:.2f}s".format(status_age),
                            arguments.max_gps_outage,
                            arguments.imu_timeout,
                            arguments.status_timeout,
                        )
                    )
                else:
                    gps_label = (
                        "GPS" if gps_outage <= 1.0 else "GPS-OUT {:.1f}s".format(gps_outage)
                    )
                    print(
                        "{} pos=({:.2f},{:.2f},{:.2f}) speed={:.2f}/{:.2f} "
                        "curve={} "
                        "vel_x={:+.2f}km/h "
                        "front={:.2f}m preview={:.1f}m "
                        "yaw/path={:+.1f}/{:+.1f}deg herr={:+.1f}deg "
                        "cte={:+.2f}m steer(raw/guard/scale/bias/filt)={:+.2f}/{:+.2f}/{:.2f}/{:+.2f}/{:+.2f}deg "
                        "cmd=({:.2f},{:+.2f},{:.2f}) "
                        "feedback=({:.2f},{:+.2f}deg,{:.2f}) "
                        "remain={:.1f}m{}".format(
                            gps_label,
                            state.x_m,
                            state.y_m,
                            state.z_m,
                            state.speed_mps,
                            target_speed_mps,
                            curve_speed_mode,
                            status_vel_x_kmh,
                            stanley.control_point_offset_m,
                            stanley.heading_preview_distance_m,
                            math.degrees(state.yaw_rad),
                            math.degrees(result.path_yaw_rad),
                            math.degrees(result.heading_error_rad),
                            result.cross_track_error_m,
                            math.degrees(raw_steering_rad),
                            math.degrees(guarded_steering_rad),
                            steering_scale,
                            math.degrees(startup_bias_rad),
                            math.degrees(filtered_steering_rad),
                            command.accel,
                            command.steering_normalized,
                            command.brake,
                            status_accel_pedal,
                            status_front_steer_deg,
                            status_brake_pedal,
                            result.remaining_distance_m,
                            " GOAL" if result.goal_reached else "",
                        )
                    )
    except KeyboardInterrupt:
        print("\nStopping controller and applying brake...")
    finally:
        stop_packet = encode_control(brake_command())
        for _ in range(5):
            control_socket.sendto(stop_packet, control_destination)
            time.sleep(0.02)
        selector.close()
        for udp_socket in receive_sockets:
            udp_socket.close()
        control_socket.close()


def main(localization_mode, argv=None):
    parser = argument_parser(localization_mode)
    if argv is None:
        argv = sys.argv[1:]
    # roslaunch appends ROS remapping arguments (for example __name:=...).
    # The controller deliberately remains UDP-only, so discard only those
    # process-management arguments before handing options to argparse.
    argv = [value for value in argv if ":=" not in value]
    run(localization_mode, parser.parse_args(argv))
