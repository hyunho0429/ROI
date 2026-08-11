#!/usr/bin/env python3

import math
import sys
import time
import socket
import ctypes
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.append(str(ROOT))

from lib.network.UDP import Sender
from lib.define.EgoVehicleStatus import EgoVehicleStatus
from lib.define.EgoCtrlCmd import EgoCtrlCmd


# ============================================================
# 네트워크 설정
# ============================================================

UBUNTU_IP = "192.168.0.200"
EGO_STATUS_PORT = 1911

MORAI_IP = "192.168.0.151"
CMD_CONTROL_PORT = 9093


# ============================================================
# 경로 및 차량 설정
# ============================================================

PATH_FILE = ROOT / "2026_molit_comp_global_path.txt"
PROCESSED_PATH_FILE = ROOT / "processed_global_path.txt"
LOG_FILE = ROOT / "driving_log.csv"

WHEELBASE = 2.7
MAX_STEER_RAD = math.radians(30.0)
STEER_SIGN = 1.0

MAX_PATH_DISTANCE = 15.0
GOAL_DISTANCE = 3.0

# 속도 설정
STRAIGHT_SPEED_KMH = 40
MEDIUM_CURVE_SPEED_KMH = 30
SHARP_CURVE_SPEED_KMH = 20
RECOVERY_SPEED_KMH = 3.0

# Look-ahead 설정
MIN_LOOKAHEAD = 3.0
MAX_LOOKAHEAD = 9.0
LOOKAHEAD_SPEED_GAIN = 0.15
# LOOKAHEAD_SPEED_GAIN = 0.35

# 경로 검색 범위
SEARCH_BACKWARD = 8
SEARCH_FORWARD = 80
MAX_INDEX_ADVANCE_PER_CYCLE = 15

# 명령 변화율 제한
MAX_STEER_CHANGE_PER_SEC = 1.8
MAX_ACCEL_CHANGE_PER_SEC = 0.60
MAX_BRAKE_CHANGE_PER_SEC = 0.80

# 조향 비대칭 보정
#LEFT_STEER_GAIN = 1.0
#RIGHT_STEER_GAIN = 1.35

# 경로 전처리 설정
INTERPOLATION_SPACING = 0.20
DUPLICATE_MIN_DISTANCE = 0.05
SMOOTHING_WINDOW = 5
SMOOTHING_PASSES = 1

CONTROL_PERIOD = 0.02
CONTROL_PERIOD = 0.02
LOG_PERIOD = 0.10

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle_rad(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def rate_limit(target, previous, max_rate_per_sec, dt):
    max_change = max_rate_per_sec * max(dt, 1e-3)
    return clamp(
        target,
        previous - max_change,
        previous + max_change
    )


def remove_duplicate_points(path, min_distance):
    """
    연속된 중복점 또는 지나치게 가까운 점을 제거한다.
    """
    if not path:
        return []

    cleaned = [path[0]]

    for point in path[1:]:
        previous = cleaned[-1]

        if distance(
            previous[0],
            previous[1],
            point[0],
            point[1]
        ) >= min_distance:
            cleaned.append(point)

    return cleaned


def interpolate_path_by_distance(path, spacing):
    """
    경로 누적거리를 기준으로 일정 간격의 웨이포인트를 생성한다.
    선형 보간을 사용하되, 이후 스무딩 단계에서 급격한 꺾임을 완화한다.
    """
    if len(path) < 2:
        return path[:]

    cumulative = [0.0]

    for index in range(1, len(path)):
        x0, y0, _ = path[index - 1]
        x1, y1, _ = path[index]

        segment = distance(x0, y0, x1, y1)
        cumulative.append(cumulative[-1] + segment)

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

        if d1 <= d0:
            ratio = 0.0
        else:
            ratio = (target_distance - d0) / (d1 - d0)

        ratio = clamp(ratio, 0.0, 1.0)

        x = x0 + ratio * (x1 - x0)
        y = y0 + ratio * (y1 - y0)
        z = z0 + ratio * (z1 - z0)

        interpolated.append((x, y, z))
        target_distance += spacing

    # 마지막 점은 원본 경로의 마지막 점으로 정확히 유지
    if interpolated[-1] != path[-1]:
        interpolated.append(path[-1])

    return interpolated


def smooth_path_moving_average(path, window_size, passes):
    """
    이동평균으로 경로의 급격한 방향 변화를 완화한다.

    시작점과 끝점은 원본 좌표를 유지한다.
    """
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

            x = sum(point[0] for point in points) / len(points)
            y = sum(point[1] for point in points) / len(points)
            z = sum(point[2] for point in points) / len(points)

            next_path[index] = (x, y, z)

        next_path[0] = path[0]
        next_path[-1] = path[-1]
        smoothed = next_path

    return smoothed


def save_path(path, output_file):
    with open(output_file, "w", encoding="utf-8") as file:
        for x, y, z in path:
            file.write(f"{x:.9f} {y:.9f} {z:.9f}\n")


def preprocess_path(raw_path):
    """
    원본 경로에 다음 순서로 전처리를 적용한다.

    1. 중복점 제거
    2. 일정 간격 재보간
    3. 약한 이동평균 스무딩
    """
    cleaned = remove_duplicate_points(
        raw_path,
        DUPLICATE_MIN_DISTANCE
    )

    interpolated = interpolate_path_by_distance(
        cleaned,
        INTERPOLATION_SPACING
    )

    smoothed = smooth_path_moving_average(
        interpolated,
        SMOOTHING_WINDOW,
        SMOOTHING_PASSES
    )

    return cleaned, interpolated, smoothed


def load_path(path_file):
    path = []

    with open(path_file, "r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            values = line.replace(",", " ").split()

            if len(values) < 2:
                print(f"[경고] {line_number}번째 줄 건너뜀: {line}")
                continue

            try:
                x = float(values[0])
                y = float(values[1])
                z = float(values[2]) if len(values) >= 3 else 0.0
                path.append((x, y, z))
            except ValueError:
                print(f"[경고] 숫자 변환 실패: {line}")

    if len(path) < 2:
        raise RuntimeError("경로점이 2개 미만입니다.")

    return path


def distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def path_heading(path, index, step=3):
    i0 = clamp(index, 0, len(path) - 1)
    i1 = clamp(index + step, 0, len(path) - 1)

    i0 = int(i0)
    i1 = int(i1)

    if i0 == i1:
        i0 = max(0, i0 - 1)

    x0, y0, _ = path[i0]
    x1, y1, _ = path[i1]
    return math.atan2(y1 - y0, x1 - x0)


def find_nearest_index(
    path,
    ego_x,
    ego_y,
    ego_yaw_deg,
    previous_index
):
    """
    거리뿐 아니라 차량 진행 방향과 경로 방향도 함께 사용한다.
    가까이 겹치는 경로에서 다른 가지로 인덱스가 튀는 현상을 억제한다.
    """
    search_start = max(0, previous_index - SEARCH_BACKWARD)
    search_end = min(
        len(path),
        previous_index + SEARCH_FORWARD + 1
    )

    ego_yaw = math.radians(ego_yaw_deg)
    best_index = previous_index
    best_score = float("inf")
    best_distance = float("inf")

    for index in range(search_start, search_end):
        px, py, _ = path[index]
        dist = distance(ego_x, ego_y, px, py)

        heading = path_heading(path, index)
        heading_error = abs(normalize_angle_rad(heading - ego_yaw))

        # 방향이 100도 이상 반대인 경로점은 강하게 배제
        heading_penalty = 8.0 * heading_error

        # 이전 인덱스보다 뒤로 가는 후보에는 작은 페널티
        backward_penalty = 0.15 * max(0, previous_index - index)

        score = dist + heading_penalty + backward_penalty

        if score < best_score:
            best_score = score
            best_index = index
            best_distance = dist

    # 한 주기에 인덱스가 지나치게 멀리 점프하지 못하도록 제한
    best_index = min(
        best_index,
        previous_index + MAX_INDEX_ADVANCE_PER_CYCLE
    )

    px, py, _ = path[best_index]
    best_distance = distance(ego_x, ego_y, px, py)

    return best_index, best_distance


def calculate_curve_angle(path, nearest_index):
    last = len(path) - 1

    i0 = min(nearest_index, last)
    i1 = min(nearest_index + 20, last)
    i2 = min(nearest_index + 60, last)

    if i0 == i1 or i1 == i2:
        return 0.0

    x0, y0, _ = path[i0]
    x1, y1, _ = path[i1]
    x2, y2, _ = path[i2]

    h1 = math.atan2(y1 - y0, x1 - x0)
    h2 = math.atan2(y2 - y1, x2 - x1)

    return normalize_angle_rad(h2 - h1)
    
def calculate_preview_curve_angle(path, nearest_index):

    last = len(path) - 1

    preview_offsets = [0, 20, 40, 60]   # 0m, 4m, 8m, 12m 정도

    max_curve = 0.0

    for offset in preview_offsets:

        idx = min(nearest_index + offset, last)

        curve = abs(calculate_curve_angle(path, idx))

        if curve > max_curve:
            max_curve = curve

    return max_curve
        
def point_to_segment_signed_distance(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1

    seg_len2 = dx * dx + dy * dy
    if seg_len2 < 1e-9:
        return math.hypot(px - x1, py - y1)

    t = ((px - x1) * dx + (py - y1) * dy) / seg_len2
    t = clamp(t, 0.0, 1.0)

    proj_x = x1 + t * dx
    proj_y = y1 + t * dy

    dist = math.hypot(px - proj_x, py - proj_y)

    cross = dx * (py - proj_y) - dy * (px - proj_x)

    if cross >= 0:
        return dist
    else:
        return -dist


def calculate_signed_cte(path, nearest_index, ego_x, ego_y):

    candidates = []

    if nearest_index > 0:
        candidates.append((nearest_index - 1, nearest_index))

    if nearest_index < len(path) - 1:
        candidates.append((nearest_index, nearest_index + 1))

    best = None

    for i0, i1 in candidates:

        x1, y1, _ = path[i0]
        x2, y2, _ = path[i1]

        cte = point_to_segment_signed_distance(
            ego_x,
            ego_y,
            x1,
            y1,
            x2,
            y2,
        )

        if best is None or abs(cte) < abs(best):
            best = cte

    if best is None:
        return 0.0

    return best

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

    lookahead = MIN_LOOKAHEAD + LOOKAHEAD_SPEED_GAIN * speed_kmh

    if curve_deg >= 14.0:
        lookahead *= 0.65
    elif curve_deg >= 6.0:
        lookahead *= 0.82

    return clamp(lookahead, MIN_LOOKAHEAD, MAX_LOOKAHEAD)


def find_lookahead_index(path, nearest_index, lookahead_distance):
    """
    경로 누적거리 기준으로 목표점을 선택한다.
    가까운 다른 경로 가지를 직선거리로 잘못 선택하는 것을 방지한다.
    """
    accumulated = 0.0

    for index in range(nearest_index, len(path) - 1):
        x1, y1, _ = path[index]
        x2, y2, _ = path[index + 1]

        accumulated += distance(x1, y1, x2, y2)

        if accumulated >= lookahead_distance:
            return index + 1

    return len(path) - 1


def calculate_pure_pursuit_steer(
    ego_x,
    ego_y,
    ego_yaw_deg,
    target_x,
    target_y
):
    yaw_rad = math.radians(ego_yaw_deg)

    dx = target_x - ego_x
    dy = target_y - ego_y

    local_x = (
        math.cos(yaw_rad) * dx
        + math.sin(yaw_rad) * dy
    )
    local_y = (
        -math.sin(yaw_rad) * dx
        + math.cos(yaw_rad) * dy
    )

    target_distance = max(math.hypot(local_x, local_y), 0.1)

    # 목표점이 약간 뒤에 있어도 즉시 조향 0으로 만들지 않는다.
    # 완전히 뒤쪽일 때만 안전하게 감속하도록 유효하지 않은 목표로 처리한다.
    target_valid = local_x > -0.5

    if not target_valid:
        return 0.0, local_x, local_y, False

    effective_local_x = max(local_x, 0.3)

    steering_rad = math.atan2(
        2.0 * WHEELBASE * local_y,
        target_distance ** 2
    )

    steering_rad *= STEER_SIGN

#    if steering_rad >= 0.0:
#        steering_rad *= LEFT_STEER_GAIN
#    else:
#        steering_rad *= RIGHT_STEER_GAIN

    steering = clamp(
        steering_rad / MAX_STEER_RAD,
        -1.0,
        1.0
    )

    return steering, effective_local_x, local_y, True


def calculate_longitudinal_control(speed_kmh, target_speed_kmh):
    error = target_speed_kmh - speed_kmh

    # 가속과 브레이크를 동시에 사용하지 않는다.
    if error > 0.4:
        accel = clamp(0.04 + 0.025 * error, 0.0, 0.28)
        brake = 0.0
    elif error < -0.8:
        accel = 0.0
        brake = clamp(0.02 * abs(error), 0.0, 0.2)
    else:
        # 데드밴드에서 급격한 가감속 반복을 막는다.
        accel = 0.015
        brake = 0.0

    return accel, brake


def make_command(accel, brake, steer):
    command = EgoCtrlCmd()

    command.ctrl_mode = 2
    command.gear = 4
    command.cmd_type = 1

    command.velocity = 0.0
    command.acceleration = 0.0
    command.accel = float(clamp(accel, 0.0, 1.0))
    command.brake = float(clamp(brake, 0.0, 1.0))
    command.steer = float(clamp(steer, -1.0, 1.0))

    return command


def send_stop(sender, duration=2.0):
    stop_command = make_command(
        accel=0.0,
        brake=1.0,
        steer=0.0
    )

    end_time = time.time() + duration

    while time.time() < end_time:
        sender.send(stop_command)
        time.sleep(0.05)


def initialize_log():
    with open(LOG_FILE, "w", encoding="utf-8") as file:
        file.write(
            "time,x,y,yaw,speed,steer,signed_cte\n"
        )


def append_log(
    now,
    ego_x,
    ego_y,
    ego_yaw,
    speed_kmh,
    steer,
    signed_cte
):
    with open(LOG_FILE, "a", encoding="utf-8") as file:
        file.write(
            f"{now:.3f},"
            f"{ego_x:.6f},"
            f"{ego_y:.6f},"
            f"{ego_yaw:.3f},"
            f"{speed_kmh:.3f},"
            f"{steer:.5f},"
            f"{signed_cte:.5f}\n"
        )


def main():
    raw_path = load_path(PATH_FILE)

    cleaned_path, interpolated_path, path = preprocess_path(
        raw_path
    )

    save_path(path, PROCESSED_PATH_FILE)
    initialize_log()

    print("=" * 72)
    print("MORAI Stable Pure Pursuit UDP Controller")
    print(f"원본 경로 파일 : {PATH_FILE}")
    print(f"가공 경로 파일 : {PROCESSED_PATH_FILE}")
    print(f"원본 경로점    : {len(raw_path)}개")
    print(f"중복 제거 후   : {len(cleaned_path)}개")
    print(f"보간 후        : {len(interpolated_path)}개")
    print(f"최종 경로점    : {len(path)}개")
    print(
        f"보간 간격      : {INTERPOLATION_SPACING:.2f} m / "
        f"스무딩 창 {SMOOTHING_WINDOW} / "
        f"반복 {SMOOTHING_PASSES}"
    )
    print(f"주행 로그      : {LOG_FILE}")
    print(f"직선 목표속도  : {STRAIGHT_SPEED_KMH:.1f} km/h")
    print(f"급곡선 목표속도: {SHARP_CURVE_SPEED_KMH:.1f} km/h")
    print("=" * 72)
    print("MORAI에서 Q를 눌러 AV-ExternalCtrl로 설정하세요.")
    print("종료: Ctrl+C")
    print()

    status_socket = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )
    status_socket.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR,
        1
    )
    status_socket.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_RCVBUF,
        2 ** 20
    )
    status_socket.bind((UBUNTU_IP, EGO_STATUS_PORT))
    print("bind ok")
    status_socket.settimeout(1.0)

    status_size = ctypes.sizeof(EgoVehicleStatus)

    sender = Sender(MORAI_IP, CMD_CONTROL_PORT)

    previous_nearest_index = 0
    previous_print_time = 0.0
    previous_log_time = 0.0
    previous_time = time.monotonic()

    previous_steer = 0.0
    previous_accel = 0.0
    previous_brake = 0.0

    try:
        while True:
            try:
                raw_data, _ = status_socket.recvfrom(4096)
            except socket.timeout:
                print("[수신 대기] Ego Vehicle Status 패킷이 없습니다.")
                continue

            if len(raw_data) < status_size:
                print(
                    f"[패킷 오류] 수신={len(raw_data)}, "
                    f"필요={status_size}"
                )
                continue

            status = EgoVehicleStatus.from_buffer_copy(
                raw_data[:status_size]
            )
            

            now_monotonic = time.monotonic()
            dt = clamp(
                now_monotonic - previous_time,
                0.005,
                0.100
            )
            previous_time = now_monotonic

            ego_x = float(status.pos_x)
            ego_y = float(status.pos_y)
            ego_yaw = float(status.yaw)
            speed_kmh = abs(float(status.signed_vel))

            # 비정상 센서값은 해당 주기 명령에서 제외
            if not all(
                math.isfinite(value)
                for value in (ego_x, ego_y, ego_yaw, speed_kmh)
            ):
                print("[센서 오류] NaN 또는 Inf가 감지되었습니다.")
                continue

            nearest_index, nearest_distance = find_nearest_index(
                path,
                ego_x,
                ego_y,
                ego_yaw,
                previous_nearest_index
            )
            signed_cte = calculate_signed_cte(
                path,
                nearest_index,
                ego_x,
                ego_y
            )

            previous_nearest_index = max(
                previous_nearest_index,
                nearest_index
            )

            if nearest_distance > MAX_PATH_DISTANCE:
                target_steer = 0.0
                target_accel = 0.0
                target_brake = 0.35

                steer = rate_limit(
                    target_steer,
                    previous_steer,
                    MAX_STEER_CHANGE_PER_SEC,
                    dt
                )
                accel = rate_limit(
                    target_accel,
                    previous_accel,
                    MAX_ACCEL_CHANGE_PER_SEC,
                    dt
                )
                brake = rate_limit(
                    target_brake,
                    previous_brake,
                    MAX_BRAKE_CHANGE_PER_SEC,
                    dt
                )

                sender.send(make_command(accel, brake, steer))

                previous_steer = steer
                previous_accel = accel
                previous_brake = brake
                continue

            curve_angle = calculate_preview_curve_angle(
                path,
                nearest_index
            )

            target_speed = select_target_speed(
                curve_angle,
                nearest_distance
            )

            lookahead = select_lookahead(
                speed_kmh,
                curve_angle
            )

            lookahead_index = find_lookahead_index(
                path,
                nearest_index,
                lookahead
            )

            target_x, target_y, _ = path[lookahead_index]

            raw_steer, local_x, local_y, target_valid = (
                calculate_pure_pursuit_steer(
                    ego_x,
                    ego_y,
                    ego_yaw,
                    target_x,
                    target_y
                )
            )

            if not target_valid:
                target_speed = min(
                    target_speed,
                    RECOVERY_SPEED_KMH
                )
                raw_steer = previous_steer

            raw_accel, raw_brake = calculate_longitudinal_control(
                speed_kmh,
                target_speed
            )

            # 경로 오차가 커질수록 가속 제한
            if nearest_distance > 2.0:
                raw_accel = min(raw_accel, 0.08)

            if nearest_distance > 4.0:
                raw_accel = 0.0
                raw_brake = max(raw_brake, 0.08)

            # 조향이 큰 구간에서는 가속을 제한하되 갑작스러운 강제 브레이크는 하지 않는다.
            if abs(raw_steer) > 0.65:
                raw_accel = min(raw_accel, 0.06)

            steer = rate_limit(
                raw_steer,
                previous_steer,
                MAX_STEER_CHANGE_PER_SEC,
                dt
            )
            accel = rate_limit(
                raw_accel,
                previous_accel,
                MAX_ACCEL_CHANGE_PER_SEC,
                dt
            )
            brake = rate_limit(
                raw_brake,
                previous_brake,
                MAX_BRAKE_CHANGE_PER_SEC,
                dt
            )

            # 가속/브레이크 동시 명령 완전 차단
            if brake > 0.01:
                accel = 0.0
            elif accel > 0.01:
                brake = 0.0

            goal_x, goal_y, _ = path[-1]
            goal_distance = distance(
                ego_x,
                ego_y,
                goal_x,
                goal_y
            )

            if (
                lookahead_index >= len(path) - 2
                and goal_distance <= GOAL_DISTANCE
            ):
                print("최종 목적지에 도착했습니다.")
                send_stop(sender)
                break

            sender.send(
                make_command(
                    accel=accel,
                    brake=brake,
                    steer=steer
                )
            )

            previous_steer = steer
            previous_accel = accel
            previous_brake = brake

            if time.time() - previous_log_time >= LOG_PERIOD:

                append_log(
                    time.time(),
                    ego_x,
                    ego_y,
                    ego_yaw,
                    speed_kmh,
                    steer,
                    signed_cte
                )

                previous_log_time = time.time()
            

            if time.time() - previous_print_time >= 0.2:
                print(
                    f"ego=({ego_x:8.2f},{ego_y:8.2f}) | "
                    f"yaw={ego_yaw:7.2f} | "
                    f"speed={speed_kmh:5.2f}/{target_speed:4.1f} | "
                    f"idx={nearest_index:5d}->{lookahead_index:5d} | "
                    f"dist={nearest_distance:5.2f} | "
                    f"curve={math.degrees(curve_angle):6.1f} | "
                    f"Ld={lookahead:4.1f} | "
                    f"local=({local_x:5.2f},{local_y:5.2f}) | "
                    f"steer={steer:6.3f} | "
                    f"accel={accel:5.2f} | "
                    f"brake={brake:5.2f}"
                )
                previous_print_time = time.time()

            sleep_time = CONTROL_PERIOD - (
                time.monotonic() - now_monotonic
            )
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n사용자가 제어를 중단했습니다.")

    except Exception as error:
        print(f"\n오류 발생: {error}")

    finally:
        print("정지 명령 전송 중...")
        send_stop(sender)
        status_socket.close()
        print("프로그램 종료")


if __name__ == "__main__":
    main()
