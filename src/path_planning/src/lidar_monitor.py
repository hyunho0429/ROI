#!/usr/bin/env python3
"""Real-time MORAI VLP-16 LiDAR obstacle monitor over UDP.

This monitor is intentionally independent from ROS topics.  It receives MORAI
3D LiDAR Intensity UDP packets, decodes the Velodyne/VLP-16 raw payload, and
accumulates enough packets to cover a meaningful azimuth sweep before deciding
whether there are obstacle-like returns in the ego-vehicle forward corridor.

Coordinate conventions used here:

* MORAI/Velodyne raw frame: +x right, +y forward, +z up.
* Monitor/ego-local frame: +x forward, +y left, +z up.

The ego vehicle's simulator position (x, y, z) is map-global ENU.  That is not
needed for this local front-obstacle monitor; it becomes relevant only when
transforming LiDAR-local points into the global map.
"""

import argparse
import math
import socket
import time

try:
    from path_planning.morai_competition_config import BIND_IP
except ImportError:
    BIND_IP = "0.0.0.0"

try:
    from path_planning.morai_competition_config import LIDAR_HOST_PORT, LIDAR_PORT
except ImportError:
    LIDAR_HOST_PORT = 2000
    LIDAR_PORT = 2001

from path_planning.morai_udp_lidar import (
    LidarPacketError,
    parse_lidar_intensity_packet,
)


def argument_parser():
    parser = argparse.ArgumentParser(
        description="Monitor MORAI VLP-16 UDP LiDAR and print front-obstacle candidates."
    )
    parser.add_argument("--bind-ip", default=BIND_IP)
    parser.add_argument("--host-port", type=int, default=LIDAR_HOST_PORT)
    parser.add_argument("--destination-port", type=int, default=LIDAR_PORT)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--packets-per-window",
        type=int,
        default=80,
        help="number of UDP packets accumulated for each detection window",
    )
    parser.add_argument(
        "--windows",
        type=int,
        default=0,
        help="number of windows to print; 0 means run until Ctrl+C",
    )
    parser.add_argument(
        "--lidar-yaw-offset-deg",
        type=float,
        default=0.0,
        help=(
            "rotation from parsed LiDAR-local frame to ego frame. If front objects "
            "appear on the side, run with --yaw-scan and set this value accordingly"
        ),
    )
    parser.add_argument("--front-x-min", type=float, default=-40.0)
    parser.add_argument("--front-x-max", type=float, default=35.0)
    parser.add_argument("--front-y-abs", type=float, default=40.0)
    parser.add_argument("--front-z-min", type=float, default=-2.2)
    parser.add_argument("--front-z-max", type=float, default=2.0)
    parser.add_argument(
        "--fov-left-deg",
        type=float,
        default=180.0,
        help=(
            "left side angular limit of the sampled LiDAR area in ego-local "
            "coordinates; 0 deg is straight ahead"
        ),
    )
    parser.add_argument(
        "--fov-right-deg",
        type=float,
        default=180.0,
        help=(
            "right side angular limit of the sampled LiDAR area in ego-local "
            "coordinates; 0 deg is straight ahead"
        ),
    )
    parser.add_argument(
        "--rear-blind-deg",
        type=float,
        default=60.0,
        help=(
            "angular sector centered at 180/-180 deg to ignore around the ego rear; "
            "use 0 for full 360 deg"
        ),
    )
    parser.add_argument(
        "--min-distance",
        type=float,
        default=0.2,
        help="ignore zero/no-return and unrealistically near measurements",
    )
    parser.add_argument(
        "--height-above-ground",
        type=float,
        default=0.18,
        help=(
            "object-like point threshold above estimated road ground in the front "
            "corridor. Lower this for very low boxes"
        ),
    )
    parser.add_argument(
        "--min-object-points",
        type=int,
        default=3,
        help="minimum object-like points required to report DETECTED",
    )
    parser.add_argument(
        "--closest-points",
        type=int,
        default=8,
        help="number of nearest front/object points to print",
    )
    parser.add_argument(
        "--yaw-scan",
        action="store_true",
        help="print candidate counts for common LiDAR mounting yaw offsets",
    )
    parser.add_argument(
        "--quiet-raw",
        action="store_true",
        help="hide per-window raw count details",
    )
    return parser


def _rotate_xy(x_forward, y_left, yaw_offset_deg):
    yaw_rad = math.radians(yaw_offset_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        cos_yaw * x_forward - sin_yaw * y_left,
        sin_yaw * x_forward + cos_yaw * y_left,
    )


def _point_as_ego(point, yaw_offset_deg):
    x_forward, y_left = _rotate_xy(point.x_m, point.y_m, yaw_offset_deg)
    bearing_deg = math.degrees(math.atan2(y_left, x_forward))
    return {
        "x": x_forward,
        "y": y_left,
        "z": point.z_m,
        "bearing": bearing_deg,
        "distance": point.distance_m,
        "intensity": point.intensity,
        "azimuth": point.azimuth_deg,
        "laser": point.laser_id,
        "raw_x_right": point.raw_x_right_m,
        "raw_y_forward": point.raw_y_forward_m,
    }


def _valid_points(points, arguments, yaw_offset_deg=None):
    if yaw_offset_deg is None:
        yaw_offset_deg = arguments.lidar_yaw_offset_deg
    result = []
    for point in points:
        if point.distance_m < arguments.min_distance:
            continue
        ego = _point_as_ego(point, yaw_offset_deg)
        if arguments.rear_blind_deg > 0.0:
            rear_delta = abs(abs(ego["bearing"]) - 180.0)
            if rear_delta <= 0.5 * arguments.rear_blind_deg:
                continue
        if not (-arguments.fov_right_deg <= ego["bearing"] <= arguments.fov_left_deg):
            continue
        if (
            arguments.front_x_min <= ego["x"] <= arguments.front_x_max
            and abs(ego["y"]) <= arguments.front_y_abs
            and arguments.front_z_min <= ego["z"] <= arguments.front_z_max
        ):
            result.append(ego)
    return result


def _percentile(sorted_values, ratio):
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    index = int(round((len(sorted_values) - 1) * ratio))
    index = max(0, min(index, len(sorted_values) - 1))
    return sorted_values[index]


def _detect_obstacle(points, arguments, yaw_offset_deg=None):
    front_points = _valid_points(points, arguments, yaw_offset_deg)
    if not front_points:
        return {
            "detected": False,
            "front_points": [],
            "object_points": [],
            "ground_z": None,
        }

    z_values = sorted(point["z"] for point in front_points)
    ground_z = _percentile(z_values, 0.10)
    object_z_min = ground_z + arguments.height_above_ground
    object_points = [
        point
        for point in front_points
        if point["z"] >= object_z_min
    ]
    return {
        "detected": len(object_points) >= arguments.min_object_points,
        "front_points": front_points,
        "object_points": object_points,
        "ground_z": ground_z,
        "object_z_min": object_z_min,
    }


def _nearest(points, limit):
    return sorted(points, key=lambda point: (point["x"], abs(point["y"]), point["distance"]))[
        : max(0, limit)
    ]


def _print_points(label, points, limit):
    visible = _nearest(points, limit)
    if not visible:
        print("  {}: none".format(label))
        return
    print("  {}: showing {} nearest".format(label, len(visible)))
    for index, point in enumerate(visible):
        print(
            "    [{}] x_forward={:+.2f}m, y_left={:+.2f}m, z_up={:+.2f}m, "
            "bearing={:+.1f}deg, dist={:.2f}m, intensity={}, azimuth={:.1f}deg, laser={}".format(
                index,
                point["x"],
                point["y"],
                point["z"],
                point["bearing"],
                point["distance"],
                point["intensity"],
                point["azimuth"],
                point["laser"],
            )
        )


def _yaw_scan(points, arguments):
    if not arguments.yaw_scan:
        return
    print("  yaw scan:")
    for yaw in (-180, -135, -90, -45, 0, 45, 90, 135, 180):
        result = _detect_obstacle(points, arguments, yaw)
        front_count = len(result["front_points"])
        object_count = len(result["object_points"])
        nearest = _nearest(result["front_points"], 1)
        if nearest:
            nearest_text = "nearest x={:+.1f} y={:+.1f} z={:+.1f} dist={:.1f}".format(
                nearest[0]["x"],
                nearest[0]["y"],
                nearest[0]["z"],
                nearest[0]["distance"],
            )
        else:
            nearest_text = "nearest n/a"
        print(
            "    yaw {:+4.0f}deg: front={:<4d} object_like={:<4d} {}".format(
                yaw,
                front_count,
                object_count,
                nearest_text,
            )
        )


def _validate(arguments):
    for port in (arguments.host_port, arguments.destination_port):
        if not 1 <= port <= 65535:
            raise ValueError("UDP ports must be between 1 and 65535")
    if arguments.timeout <= 0.0:
        raise ValueError("timeout must be positive")
    if arguments.packets_per_window < 1:
        raise ValueError("packets-per-window must be at least 1")
    if arguments.windows < 0:
        raise ValueError("windows cannot be negative")
    if arguments.front_x_min < 0.0 or arguments.front_x_max <= arguments.front_x_min:
        raise ValueError("front x range must satisfy 0 <= min < max")
    if arguments.front_y_abs < 0.0:
        raise ValueError("front-y-abs cannot be negative")
    if arguments.front_z_max <= arguments.front_z_min:
        raise ValueError("front z range must satisfy min < max")
    if arguments.fov_left_deg < 0.0 or arguments.fov_right_deg < 0.0:
        raise ValueError("FOV limits cannot be negative")
    if arguments.fov_left_deg > 180.0 or arguments.fov_right_deg > 180.0:
        raise ValueError("FOV limits cannot exceed 180 degrees")
    if arguments.rear_blind_deg < 0.0 or arguments.rear_blind_deg > 180.0:
        raise ValueError("rear-blind-deg must be between 0 and 180")
    if arguments.min_distance < 0.0:
        raise ValueError("min-distance cannot be negative")
    if arguments.height_above_ground < 0.0:
        raise ValueError("height-above-ground cannot be negative")
    if arguments.min_object_points < 1:
        raise ValueError("min-object-points must be at least 1")
    if arguments.closest_points < 0:
        raise ValueError("closest-points cannot be negative")


def run(arguments):
    _validate(arguments)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    udp_socket.bind((arguments.bind_ip, arguments.destination_port))
    udp_socket.settimeout(arguments.timeout)

    print(
        "MORAI VLP-16 LiDAR monitor: source *:{} -> destination {}:{}; "
        "window={} packets; yaw_offset={:+.1f}deg".format(
            arguments.host_port,
            arguments.bind_ip,
            arguments.destination_port,
            arguments.packets_per_window,
            arguments.lidar_yaw_offset_deg,
        )
    )
    print(
        "front corridor: x={:.1f}..{:.1f}m, |y|<={:.1f}m, z={:.1f}..{:.1f}m; "
        "FOV right=-{:.1f}deg left=+{:.1f}deg, rear_blind={:.1f}deg; "
        "object if >= {} points above ground+{:.2f}m".format(
            arguments.front_x_min,
            arguments.front_x_max,
            arguments.front_y_abs,
            arguments.front_z_min,
            arguments.front_z_max,
            arguments.fov_right_deg,
            arguments.fov_left_deg,
            arguments.rear_blind_deg,
            arguments.min_object_points,
            arguments.height_above_ground,
        )
    )

    window_index = 0
    try:
        while arguments.windows == 0 or window_index < arguments.windows:
            window_index += 1
            points = []
            packets = 0
            bad_packets = 0
            started_at = time.time()
            while packets < arguments.packets_per_window:
                try:
                    packet, sender = udp_socket.recvfrom(65535)
                except socket.timeout:
                    print("TIMEOUT: no packet received within {:.1f}s".format(arguments.timeout))
                    return 2
                if sender[1] != arguments.host_port and not arguments.quiet_raw:
                    print(
                        "WARNING: sender source port {}, expected {}".format(
                            sender[1],
                            arguments.host_port,
                        )
                    )
                packets += 1
                try:
                    lidar_packet = parse_lidar_intensity_packet(packet)
                except (LidarPacketError, ValueError):
                    bad_packets += 1
                    continue
                points.extend(lidar_packet.points)

            elapsed = max(1e-6, time.time() - started_at)
            result = _detect_obstacle(points, arguments)
            status = "DETECTED" if result["detected"] else "none"
            print(
                "\nwindow {}: obstacle={} packets={} bad={} points={} rate={:.1f} pkt/s".format(
                    window_index,
                    status,
                    packets,
                    bad_packets,
                    len(points),
                    packets / elapsed,
                )
            )
            if not arguments.quiet_raw:
                print(
                    "  front_points={} object_like={} ground_z={} object_z_min={}".format(
                        len(result["front_points"]),
                        len(result["object_points"]),
                        "n/a"
                        if result["ground_z"] is None
                        else "{:+.2f}m".format(result["ground_z"]),
                        "n/a"
                        if result.get("object_z_min") is None
                        else "{:+.2f}m".format(result["object_z_min"]),
                    )
                )
            _print_points("front nearest", result["front_points"], arguments.closest_points)
            _print_points("object-like nearest", result["object_points"], arguments.closest_points)
            _yaw_scan(points, arguments)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted")
        return 130
    finally:
        udp_socket.close()


def main(argv=None):
    return run(argument_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
