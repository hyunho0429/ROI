#!/usr/bin/env python3
"""Competition Pure Pursuit runner using UDP GPS/IMU/Competition Status.

This file keeps the path preprocessing and interpolated Pure Pursuit idea from
the original ``pure_pursuit_interpolated_stable.py`` while replacing the public
EgoVehicleStatus dependency with the competition-only UDP interfaces:

GPS + IMU -> 15-state INS error-state EKF -> Pure Pursuit
Competition Vehicle Status -> vehicle speed, wheelbase and control-state guard
Ego Ctrl Cmd UDP -> longCmdType=1 accel/brake/steering command
"""

import math
import selectors
import socket
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
REPO_ROOT = ROOT.parent
PACKAGE_SRC = REPO_ROOT / "src" / "path_planning" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from path_planning.coordinates import GpsToMapEnu, MapProjection
from path_planning.localization_ins import InsErrorStateEkf
from path_planning.longitudinal_controller import PedalSpeedController
from path_planning.morai_competition_config import (
    BIND_IP,
    COLLISION_PORT,
    COMPETITION_STATUS_PORT,
    CONTROL_DESTINATION_PORT,
    CONTROL_IP,
    CONTROL_PORT,
    GPS_PORT,
    IMU_PORT,
    VEHICLE_LENGTH_M,
    VEHICLE_WIDTH_M,
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
    CONTROL_PROTOCOL_25S4,
    brake_command,
    encode_ego_ctrl_cmd,
    external_control_ready,
    pedal_command,
)
from path_planning.morai_udp_gps import GpsPacketError, parse_nmea_datagram
from path_planning.morai_udp_imu import ImuPacketError, parse_imu_packet


# ---------------------------------------------------------------------------
# Network settings for the current competition network.
# MORAI publisher Host Port -> algorithm Destination Port:
#   GPS 3001, IMU 4001, Competition Status 9080 -> 9081, Collision 9091 -> 9092
# Ego Ctrl Cmd:
#   algorithm source/destination port 9094 -> MORAI host 192.168.56.1:9093
# ---------------------------------------------------------------------------

MORAI_HOST_IP = CONTROL_IP  # 192.168.56.1
ALGORITHM_IP = "192.168.56.101"
COMMAND_PROTOCOL = CONTROL_PROTOCOL_25S4


# ---------------------------------------------------------------------------
# Path, vehicle and controller settings.
# ---------------------------------------------------------------------------

PATH_FILE = ROOT / "2026_molit_comp_global_path.txt"
PROCESSED_PATH_FILE = ROOT / "processed_global_path.txt"
LOG_FILE = ROOT / "driving_log.csv"
GLOBAL_INFO_FILE = (
    REPO_ROOT
    / "src"
    / "path_planning"
    / "mgeo"
    / "R_KR_PR_K-city_2025"
    / "global_info.json"
)

WHEELBASE = VEHICLE_WHEELBASE_M  # 2023 Hyundai IONIQ 5: 3.0 m
VEHICLE_MAX_STEER_RAD = math.radians(36.25)
CONTROLLER_MAX_STEER_RAD = math.radians(21.77)
STEER_SIGN = 1.0

MAX_PATH_DISTANCE = 15.0
GOAL_DISTANCE = 3.0

# Original stable branch speed policy, now driven by a PID pedal controller.
STRAIGHT_SPEED_KMH = 10.0
MEDIUM_CURVE_SPEED_KMH = 7.0
SHARP_CURVE_SPEED_KMH = 4.5
RECOVERY_SPEED_KMH = 3.0

# Shorter lookahead prevents early turn-in on the competition route.
MIN_LOOKAHEAD = 2.0
MAX_LOOKAHEAD = 4.0
LOOKAHEAD_SPEED_GAIN = 0.15

SEARCH_BACKWARD = 8
SEARCH_FORWARD = 80
MAX_INDEX_ADVANCE_PER_CYCLE = 15

MAX_STEER_RATE_RADPS = 0.35
STEER_LOW_PASS_ALPHA = 0.15

INTERPOLATION_SPACING = 0.20
DUPLICATE_MIN_DISTANCE = 0.05
SMOOTHING_WINDOW = 5
SMOOTHING_PASSES = 1

CONTROL_RATE_HZ = 30.0
CONTROL_PERIOD = 1.0 / CONTROL_RATE_HZ
PRINT_PERIOD = 0.2
SENSOR_TIMEOUT = 0.8
COLLISION_BRAKE_SECONDS = 3.0


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle_rad(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def remove_duplicate_points(path, min_distance):
    if not path:
        return []
    cleaned = [path[0]]
    for point in path[1:]:
        previous = cleaned[-1]
        if distance(previous[0], previous[1], point[0], point[1]) >= min_distance:
            cleaned.append(point)
    return cleaned


def interpolate_path_by_distance(path, spacing):
    if len(path) < 2:
        return path[:]

    cumulative = [0.0]
    for index in range(1, len(path)):
        x0, y0, _ = path[index - 1]
        x1, y1, _ = path[index]
        cumulative.append(cumulative[-1] + distance(x0, y0, x1, y1))

    total_length = cumulative[-1]
    if total_length <= spacing:
        return path[:]

    interpolated = []
    target_distance = 0.0
    segment_index = 0
    while target_distance < total_length:
        while (
            segment_index < len(path) - 2
            and cumulative[segment_index + 1] < target_distance
        ):
            segment_index += 1

        d0 = cumulative[segment_index]
        d1 = cumulative[segment_index + 1]
        x0, y0, z0 = path[segment_index]
        x1, y1, z1 = path[segment_index + 1]
        ratio = 0.0 if d1 <= d0 else (target_distance - d0) / (d1 - d0)
        ratio = clamp(ratio, 0.0, 1.0)
        interpolated.append(
            (
                x0 + ratio * (x1 - x0),
                y0 + ratio * (y1 - y0),
                z0 + ratio * (z1 - z0),
            )
        )
        target_distance += spacing

    if interpolated[-1] != path[-1]:
        interpolated.append(path[-1])
    return interpolated


def smooth_path_moving_average(path, window_size, passes):
    if len(path) < 3 or window_size < 3 or passes <= 0:
        return path[:]
    if window_size % 2 == 0:
        window_size += 1

    half_window = window_size // 2
    smoothed = path[:]
    for _ in range(passes):
        next_path = smoothed[:]
        for index in range(1, len(smoothed) - 1):
            start = max(0, index - half_window)
            end = min(len(smoothed), index + half_window + 1)
            points = smoothed[start:end]
            next_path[index] = (
                sum(point[0] for point in points) / len(points),
                sum(point[1] for point in points) / len(points),
                sum(point[2] for point in points) / len(points),
            )
        next_path[0] = path[0]
        next_path[-1] = path[-1]
        smoothed = next_path
    return smoothed


def preprocess_path(raw_path):
    cleaned = remove_duplicate_points(raw_path, DUPLICATE_MIN_DISTANCE)
    interpolated = interpolate_path_by_distance(cleaned, INTERPOLATION_SPACING)
    smoothed = smooth_path_moving_average(
        interpolated, SMOOTHING_WINDOW, SMOOTHING_PASSES
    )
    return cleaned, interpolated, smoothed


def save_path(path, output_file):
    with open(output_file, "w", encoding="utf-8") as file:
        for x, y, z in path:
            file.write(f"{x:.9f} {y:.9f} {z:.9f}\n")


def load_path(path_file):
    path = []
    with open(path_file, "r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            values = line.replace(",", " ").split()
            if len(values) < 2:
                print(f"[path warning] skip line {line_number}: {line}")
                continue
            try:
                x = float(values[0])
                y = float(values[1])
                z = float(values[2]) if len(values) >= 3 else 0.0
                path.append((x, y, z))
            except ValueError:
                print(f"[path warning] invalid number at line {line_number}: {line}")
    if len(path) < 2:
        raise RuntimeError("path file needs at least two points")
    return path


def distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def path_heading(path, index, step=3):
    i0 = int(clamp(index, 0, len(path) - 1))
    i1 = int(clamp(index + step, 0, len(path) - 1))
    if i0 == i1:
        i0 = max(0, i0 - 1)
    x0, y0, _ = path[i0]
    x1, y1, _ = path[i1]
    return math.atan2(y1 - y0, x1 - x0)


def find_nearest_index(path, ego_x, ego_y, ego_yaw_rad, previous_index):
    search_start = max(0, previous_index - SEARCH_BACKWARD)
    search_end = min(len(path), previous_index + SEARCH_FORWARD + 1)

    best_index = previous_index
    best_score = float("inf")
    for index in range(search_start, search_end):
        px, py, _ = path[index]
        dist = distance(ego_x, ego_y, px, py)
        heading = path_heading(path, index)
        heading_error = abs(normalize_angle_rad(heading - ego_yaw_rad))
        heading_penalty = 8.0 * heading_error
        backward_penalty = 0.15 * max(0, previous_index - index)
        score = dist + heading_penalty + backward_penalty
        if score < best_score:
            best_score = score
            best_index = index

    best_index = min(best_index, previous_index + MAX_INDEX_ADVANCE_PER_CYCLE)
    px, py, _ = path[best_index]
    return best_index, distance(ego_x, ego_y, px, py)


def calculate_curve_angle(path, nearest_index):
    last = len(path) - 1
    i0 = min(nearest_index, last)
    i1 = min(nearest_index + 10, last)
    i2 = min(nearest_index + 30, last)
    if i0 == i1 or i1 == i2:
        return 0.0
    x0, y0, _ = path[i0]
    x1, y1, _ = path[i1]
    x2, y2, _ = path[i2]
    h1 = math.atan2(y1 - y0, x1 - x0)
    h2 = math.atan2(y2 - y1, x2 - x1)
    return normalize_angle_rad(h2 - h1)


def select_target_speed(curve_angle, path_distance):
    curve_deg = abs(math.degrees(curve_angle))
    if path_distance > 4.0:
        return RECOVERY_SPEED_KMH
    if curve_deg >= 14.0:
        return SHARP_CURVE_SPEED_KMH
    if curve_deg >= 6.0:
        return MEDIUM_CURVE_SPEED_KMH
    return STRAIGHT_SPEED_KMH


def select_lookahead(speed_kmh, curve_angle):
    curve_deg = abs(math.degrees(curve_angle))
    lookahead = MIN_LOOKAHEAD + LOOKAHEAD_SPEED_GAIN * max(0.0, speed_kmh / 3.6)
    if curve_deg >= 14.0:
        lookahead *= 0.65
    elif curve_deg >= 6.0:
        lookahead *= 0.82
    return clamp(lookahead, MIN_LOOKAHEAD, MAX_LOOKAHEAD)


def find_lookahead_index(path, nearest_index, lookahead_distance):
    accumulated = 0.0
    for index in range(nearest_index, len(path) - 1):
        x1, y1, _ = path[index]
        x2, y2, _ = path[index + 1]
        accumulated += distance(x1, y1, x2, y2)
        if accumulated >= lookahead_distance:
            return index + 1
    return len(path) - 1


def calculate_pure_pursuit_steer_rad(ego_x, ego_y, ego_yaw_rad, target_x, target_y):
    dx = target_x - ego_x
    dy = target_y - ego_y
    local_x = math.cos(ego_yaw_rad) * dx + math.sin(ego_yaw_rad) * dy
    local_y = -math.sin(ego_yaw_rad) * dx + math.cos(ego_yaw_rad) * dy
    target_distance = max(math.hypot(local_x, local_y), 0.1)
    target_valid = local_x > -0.5
    if not target_valid:
        return 0.0, local_x, local_y, False

    steering_rad = math.atan2(2.0 * WHEELBASE * local_y, target_distance ** 2)
    steering_rad = clamp(
        steering_rad, -CONTROLLER_MAX_STEER_RAD, CONTROLLER_MAX_STEER_RAD
    )
    return steering_rad, local_x, local_y, True


class SteeringFilter:
    def __init__(self, alpha, max_rate_radps, max_abs_rad):
        self.alpha = float(alpha)
        self.max_rate_radps = float(max_rate_radps)
        self.max_abs_rad = float(max_abs_rad)
        self._last_value = 0.0
        self._last_time = None

    def reset(self):
        self._last_value = 0.0
        self._last_time = None

    def update(self, target_rad, timestamp):
        target = clamp(float(target_rad), -self.max_abs_rad, self.max_abs_rad)
        if self._last_time is None:
            self._last_time = float(timestamp)
            self._last_value = target
            return self._last_value

        dt = max(1e-3, min(0.2, float(timestamp) - self._last_time))
        self._last_time = float(timestamp)
        blended = self._last_value + self.alpha * (target - self._last_value)
        max_delta = self.max_rate_radps * dt
        self._last_value = clamp(
            blended, self._last_value - max_delta, self._last_value + max_delta
        )
        self._last_value = clamp(self._last_value, -self.max_abs_rad, self.max_abs_rad)
        return self._last_value


def receiver(bind_ip, port):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    udp_socket.bind((bind_ip, port))
    udp_socket.setblocking(False)
    return udp_socket


def command_socket():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((BIND_IP, CONTROL_DESTINATION_PORT))
    return udp_socket


def send_command(udp_socket, command):
    packet = encode_ego_ctrl_cmd(command, COMMAND_PROTOCOL)
    udp_socket.sendto(packet, (MORAI_HOST_IP, CONTROL_PORT))


def send_stop(udp_socket, duration=1.0):
    command = brake_command(1.0)
    end_time = time.monotonic() + duration
    while time.monotonic() < end_time:
        send_command(udp_socket, command)
        time.sleep(0.05)


def initialize_log():
    with open(LOG_FILE, "w", encoding="utf-8") as file:
        file.write(
            "time,x,y,z,yaw_deg,speed_kmh,target_speed_kmh,"
            "nearest,target,path_distance,curve_deg,lookahead,"
            "steer_rad,steer_normalized,accel,brake,ctrl_mode,gear,link_id\n"
        )


def append_log(
    now,
    state,
    status,
    target_speed_kmh,
    nearest_index,
    lookahead_index,
    nearest_distance,
    curve_angle,
    lookahead,
    steer_rad,
    steer_normalized,
    accel,
    brake,
):
    link_id = "" if status is None else status.link_id
    ctrl_mode = "" if status is None else status.ctrl_mode
    gear = "" if status is None else status.gear
    with open(LOG_FILE, "a", encoding="utf-8") as file:
        file.write(
            f"{now:.3f},{state.x_m:.6f},{state.y_m:.6f},{state.z_m:.3f},"
            f"{math.degrees(state.yaw_rad):.3f},{state.speed_mps * 3.6:.3f},"
            f"{target_speed_kmh:.3f},{nearest_index},{lookahead_index},"
            f"{nearest_distance:.3f},{math.degrees(curve_angle):.3f},"
            f"{lookahead:.3f},{steer_rad:.6f},{steer_normalized:.5f},"
            f"{accel:.5f},{brake:.5f},{ctrl_mode},{gear},{link_id}\n"
        )


def build_localizer():
    return InsErrorStateEkf(
        gps_position_sigma_m=1.5,
        gps_altitude_sigma_m=3.0,
        gps_speed_sigma_mps=0.8,
        imu_orientation_sigma_deg=4.0,
        gyro_noise_sigma_degps=0.8,
        accel_noise_sigma_mps2=0.25,
        gyro_bias_walk_sigma_degps=0.03,
        accel_bias_walk_sigma_mps2=0.02,
        vehicle_speed_sigma_mps=0.25,
        nhc_lateral_sigma_mps=0.35,
        nhc_vertical_sigma_mps=0.25,
        alignment_duration_s=2.0,
        alignment_min_samples=20,
    )


def main():
    raw_path = load_path(PATH_FILE)
    cleaned_path, interpolated_path, path = preprocess_path(raw_path)
    save_path(path, PROCESSED_PATH_FILE)
    initialize_log()

    projection = MapProjection.from_mgeo_global_info(GLOBAL_INFO_FILE)
    gps_converter = GpsToMapEnu(projection)
    localizer = build_localizer()
    speed_controller = PedalSpeedController(
        kp=0.075,
        ki=0.0001,
        kd=0.025,
        nominal_dt=CONTROL_PERIOD,
        max_accel=1.0,
        max_brake=1.0,
    )
    steering_filter = SteeringFilter(
        alpha=STEER_LOW_PASS_ALPHA,
        max_rate_radps=MAX_STEER_RATE_RADPS,
        max_abs_rad=CONTROLLER_MAX_STEER_RAD,
    )

    selector = selectors.DefaultSelector()
    sockets = {
        "gps": receiver(BIND_IP, GPS_PORT),
        "imu": receiver(BIND_IP, IMU_PORT),
        "status": receiver(BIND_IP, COMPETITION_STATUS_PORT),
        "collision": receiver(BIND_IP, COLLISION_PORT),
    }
    for name, udp_socket in sockets.items():
        selector.register(udp_socket, selectors.EVENT_READ, name)
    ctrl_socket = command_socket()

    last_status = None
    last_gps_time = None
    last_imu_time = None
    last_status_time = None
    collision_until = 0.0
    previous_nearest_index = 0
    previous_print_time = 0.0

    print("=" * 78)
    print("MORAI Competition Pure Pursuit - INS/EKF UDP")
    print(f"path                 : {PATH_FILE}")
    print(f"processed path       : {PROCESSED_PATH_FILE}")
    print(f"log                  : {LOG_FILE}")
    print(f"path points          : raw={len(raw_path)}, final={len(path)}")
    print(f"vehicle              : 2023 Hyundai IONIQ 5")
    print(f"length/width/wheelbase: {VEHICLE_LENGTH_M:.3f} / {VEHICLE_WIDTH_M:.3f} / {WHEELBASE:.3f} m")
    print(f"GPS/IMU/status ports : {GPS_PORT}, {IMU_PORT}, {COMPETITION_STATUS_PORT}")
    print(f"Collision port       : {COLLISION_PORT}")
    print(f"Ego Ctrl Cmd         : source {CONTROL_DESTINATION_PORT} -> {MORAI_HOST_IP}:{CONTROL_PORT}")
    print(f"Algorithm PC IP      : {ALGORITHM_IP} (set this as MORAI destination IP)")
    print("localization         : GPS + IMU + vehicle-speed aided 15-state INS EKF")
    print("control              : longCmdType=1 accel/brake/steering UDP")
    print("=" * 78)

    try:
        while True:
            loop_start = time.monotonic()
            for key, _mask in selector.select(timeout=0.0):
                name = key.data
                try:
                    packet, _addr = key.fileobj.recvfrom(4096)
                except BlockingIOError:
                    continue
                now = time.monotonic()

                if name == "gps":
                    try:
                        gps = parse_nmea_datagram(packet)
                        if gps.fix_valid:
                            x_m, y_m, z_m = gps_converter.convert(
                                gps.latitude_deg,
                                gps.longitude_deg,
                                gps.altitude_m,
                            )
                            localizer.add_gps(
                                now,
                                x_m,
                                y_m,
                                z_m,
                                speed_mps=gps.speed_mps,
                                course_deg=gps.course_deg,
                            )
                            last_gps_time = now
                    except (GpsPacketError, RuntimeError) as error:
                        print(f"[GPS] {error}")

                elif name == "imu":
                    try:
                        imu = parse_imu_packet(packet)
                        localizer.add_imu(
                            now,
                            imu.orientation_xyzw,
                            imu.angular_velocity_radps,
                            imu.linear_acceleration_mps2,
                        )
                        last_imu_time = now
                    except ImuPacketError as error:
                        print(f"[IMU] {error}")

                elif name == "status":
                    try:
                        last_status = parse_competition_vehicle_status(packet)
                        localizer.add_vehicle_speed(
                            now, last_status.signed_velocity_kmh / 3.6
                        )
                        last_status_time = now
                    except CompetitionStatusPacketError as error:
                        print(f"[Competition Status] {error}")

                elif name == "collision":
                    try:
                        collision = parse_collision_data(packet)
                        if collision.collision_detected:
                            collision_until = now + COLLISION_BRAKE_SECONDS
                    except CollisionPacketError as error:
                        print(f"[Collision] {error}")

            now = time.monotonic()
            state = localizer.state_at(now)

            stale_gps = last_gps_time is None or now - last_gps_time > SENSOR_TIMEOUT
            stale_imu = last_imu_time is None or now - last_imu_time > SENSOR_TIMEOUT
            stale_status = (
                last_status_time is None or now - last_status_time > SENSOR_TIMEOUT
            )
            collision_active = now < collision_until
            control_ready = (
                last_status is not None
                and external_control_ready(last_status.ctrl_mode, last_status.gear)
            )

            if (
                state is None
                or stale_gps
                or stale_imu
                or stale_status
                or collision_active
                or not control_ready
            ):
                speed_controller.reset()
                steering_filter.reset()
                command = brake_command(1.0 if collision_active else 0.35)
                send_command(ctrl_socket, command)
                if time.time() - previous_print_time >= 0.5:
                    reasons = []
                    if state is None:
                        reasons.append("INS not ready/alignment")
                    if stale_gps:
                        reasons.append("GPS stale")
                    if stale_imu:
                        reasons.append("IMU stale")
                    if stale_status:
                        reasons.append("Competition stale")
                    if collision_active:
                        reasons.append("collision")
                    if not control_ready:
                        mode = None if last_status is None else last_status.ctrl_mode
                        gear = None if last_status is None else last_status.gear
                        reasons.append(f"ctrl_mode/gear not ready ({mode}/{gear})")
                    print("BRAKE active:", ", ".join(reasons))
                    previous_print_time = time.time()
                time.sleep(CONTROL_PERIOD)
                continue

            status_speed_kmh = abs(last_status.signed_velocity_kmh)
            measured_speed_mps = status_speed_kmh / 3.6
            wheelbase = (
                last_status.wheelbase_m
                if last_status.wheelbase_m > 0.5
                else WHEELBASE
            )

            nearest_index, nearest_distance = find_nearest_index(
                path,
                state.x_m,
                state.y_m,
                state.yaw_rad,
                previous_nearest_index,
            )
            previous_nearest_index = max(previous_nearest_index, nearest_index)

            if nearest_distance > MAX_PATH_DISTANCE:
                steering_filter.reset()
                speed_controller.reset()
                send_command(ctrl_socket, brake_command(0.35))
                print(f"BRAKE active: path distance too large ({nearest_distance:.2f} m)")
                time.sleep(CONTROL_PERIOD)
                continue

            curve_angle = calculate_curve_angle(path, nearest_index)
            target_speed_kmh = select_target_speed(curve_angle, nearest_distance)
            lookahead = select_lookahead(status_speed_kmh, curve_angle)
            lookahead_index = find_lookahead_index(path, nearest_index, lookahead)
            target_x, target_y, _ = path[lookahead_index]
            raw_steer_rad, local_x, local_y, target_valid = calculate_pure_pursuit_steer_rad(
                state.x_m,
                state.y_m,
                state.yaw_rad,
                target_x,
                target_y,
            )

            if not target_valid:
                target_speed_kmh = min(target_speed_kmh, RECOVERY_SPEED_KMH)
                raw_steer_rad = 0.0

            # If the simulator reports a different wheelbase, preserve the
            # original Pure Pursuit curvature while updating the bicycle model.
            if abs(wheelbase - WHEELBASE) > 1e-3 and abs(raw_steer_rad) > 1e-9:
                curvature = math.tan(raw_steer_rad) / WHEELBASE
                raw_steer_rad = math.atan(wheelbase * curvature)
                raw_steer_rad = clamp(
                    raw_steer_rad,
                    -CONTROLLER_MAX_STEER_RAD,
                    CONTROLLER_MAX_STEER_RAD,
                )

            filtered_steer_rad = steering_filter.update(raw_steer_rad, now)
            steering_normalized = STEER_SIGN * clamp(
                filtered_steer_rad / VEHICLE_MAX_STEER_RAD,
                -1.0,
                1.0,
            )

            target_speed_mps = target_speed_kmh / 3.6
            accel, brake = speed_controller.compute(
                target_speed_mps, measured_speed_mps, now
            )

            if nearest_distance > 2.0:
                accel = min(accel, 0.08)
            if nearest_distance > 4.0:
                accel = 0.0
                brake = max(brake, 0.08)
            if abs(steering_normalized) > 0.65:
                accel = min(accel, 0.06)
            if brake > 0.01:
                accel = 0.0
            elif accel > 0.01:
                brake = 0.0

            goal_x, goal_y, _ = path[-1]
            goal_distance = distance(state.x_m, state.y_m, goal_x, goal_y)
            if lookahead_index >= len(path) - 2 and goal_distance <= GOAL_DISTANCE:
                print("Reached final waypoint. Sending stop command.")
                send_stop(ctrl_socket)
                break

            send_command(
                ctrl_socket,
                pedal_command(accel, brake, steering_normalized),
            )
            append_log(
                time.time(),
                state,
                last_status,
                target_speed_kmh,
                nearest_index,
                lookahead_index,
                nearest_distance,
                curve_angle,
                lookahead,
                filtered_steer_rad,
                steering_normalized,
                accel,
                brake,
            )

            if time.time() - previous_print_time >= PRINT_PERIOD:
                print(
                    f"ego=({state.x_m:8.2f},{state.y_m:8.2f}) "
                    f"yaw={math.degrees(state.yaw_rad):7.2f} "
                    f"speed={status_speed_kmh:5.2f}/{target_speed_kmh:4.1f}km/h "
                    f"idx={nearest_index:5d}->{lookahead_index:5d} "
                    f"dist={nearest_distance:5.2f} "
                    f"curve={math.degrees(curve_angle):6.1f} "
                    f"Ld={lookahead:4.2f} "
                    f"local=({local_x:5.2f},{local_y:5.2f}) "
                    f"steer={math.degrees(filtered_steer_rad):6.2f}deg/"
                    f"{steering_normalized:+.3f} "
                    f"cmd=({accel:.2f},{brake:.2f}) "
                    f"link={last_status.link_id}"
                )
                previous_print_time = time.time()

            sleep_time = CONTROL_PERIOD - (time.monotonic() - loop_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nUser interrupted control loop.")
    finally:
        print("Sending stop command...")
        try:
            send_stop(ctrl_socket)
        finally:
            for udp_socket in sockets.values():
                udp_socket.close()
            ctrl_socket.close()
        print("Done.")


if __name__ == "__main__":
    main()
