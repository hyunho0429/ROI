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
STRAIGHT_SPEED_KMH = 10.0
MEDIUM_CURVE_SPEED_KMH = 7.0
SHARP_CURVE_SPEED_KMH = 4.5
RECOVERY_SPEED_KMH = 3.0

# Look-ahead 설정
MIN_LOOKAHEAD = 3.0
MAX_LOOKAHEAD = 9.0
LOOKAHEAD_SPEED_GAIN = 0.35

# 경로 검색 범위
SEARCH_BACKWARD = 8
SEARCH_FORWARD = 80
MAX_INDEX_ADVANCE_PER_CYCLE = 15

# 명령 변화율 제한
MAX_ACCEL_CHANGE_PER_SEC = 0.60
MAX_BRAKE_CHANGE_PER_SEC = 0.80

# 조향 비대칭 보정
LEFT_STEER_GAIN = 1.0
RIGHT_STEER_GAIN = 1.10

# 장커브 스티어 안정화
CURVE_HOLD_THRESHOLD_DEG = 5.0
CURVE_SIGN_HYSTERESIS = 0.08

# raw steer에 적용하는 1차 저역통과 필터 계수
# 작을수록 더 부드럽지만 반응이 느려진다.
STEER_LOW_PASS_ALPHA = 0.35

# 정상 추종 시 기본 비율
STEER_FEEDFORWARD_GAIN = 0.75
STEER_FEEDBACK_GAIN = 0.30

# 스티어 명령 변화율과 변화 가속도 제한
MAX_STEER_CHANGE_PER_SEC = 0.65
MAX_STEER_ACCEL_PER_SEC2 = 1.50

# 스티어 진동 감지 및 복구
STEER_SIGN_DEADBAND = 0.04
OSCILLATION_WINDOW_SEC = 1.5
OSCILLATION_COUNT_LIMIT = 3
OSCILLATION_RECOVERY_DURATION_SEC = 2.5
OSCILLATION_RECOVERY_SPEED_KMH = 2.5

# S자 곡률 부호 전환 안정화
CURVATURE_FILTER_ALPHA = 0.18
MAX_CURVATURE_CHANGE_PER_SEC = 0.035
CURVATURE_SIGN_THRESHOLD = 0.0025

S_CURVE_TRANSITION_DURATION_SEC = 1.20
S_CURVE_TRANSITION_SPEED_KMH = 3.0
S_CURVE_PP_STEER_LIMIT = 0.30
S_CURVE_FINAL_STEER_LIMIT = 0.45
S_CURVE_ZERO_CROSS_RATE = 0.45

# Pure Pursuit + Stanley 하이브리드 조향
PURE_PURSUIT_WEIGHT = 0.70
STANLEY_WEIGHT = 0.30

STANLEY_CROSS_TRACK_GAIN = 0.65
STANLEY_SOFTENING_SPEED_MPS = 1.5
STANLEY_HEADING_WEIGHT = 1.0
STANLEY_CROSSTRACK_WEIGHT = 1.0
STANLEY_MAX_STEER = 0.65

# 경로 오차가 커지면 Stanley 비중을 자동 증가
HYBRID_DISTANCE_LOW = 0.8
HYBRID_DISTANCE_HIGH = 2.0
HYBRID_STANLEY_WEIGHT_MAX = 0.55

# 경로 전처리 설정
INTERPOLATION_SPACING = 0.20
DUPLICATE_MIN_DISTANCE = 0.05
SMOOTHING_WINDOW = 5
SMOOTHING_PASSES = 1

CONTROL_PERIOD = 0.02


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


def calculate_signed_curvature(path, index, offset=8):
    """
    세 점을 이용해 경로의 부호 있는 곡률을 계산한다.
    양수는 좌회전, 음수는 우회전이다.
    """
    last = len(path) - 1

    i0 = max(0, index - offset)
    i1 = min(index, last)
    i2 = min(index + offset, last)

    if i0 == i1 or i1 == i2:
        return 0.0

    x1, y1, _ = path[i0]
    x2, y2, _ = path[i1]
    x3, y3, _ = path[i2]

    a = distance(x1, y1, x2, y2)
    b = distance(x2, y2, x3, y3)
    c = distance(x1, y1, x3, y3)

    denominator = a * b * c

    if denominator < 1e-6:
        return 0.0

    twice_area = (
        (x2 - x1) * (y3 - y1)
        - (y2 - y1) * (x3 - x1)
    )

    curvature = 2.0 * twice_area / denominator
    return curvature


def calculate_average_curvature(path, nearest_index):
    """
    앞쪽 여러 점의 곡률을 평균내 순간적인 웨이포인트 잡음 영향을 줄인다.
    """
    samples = []

    for offset in (0, 5, 10, 15, 20):
        index = min(nearest_index + offset, len(path) - 1)
        samples.append(
            calculate_signed_curvature(path, index)
        )

    return sum(samples) / len(samples)


def filter_curvature(
    raw_curvature,
    previous_curvature,
    dt
):
    """
    경로 곡률의 순간적인 부호/크기 변화를 완화한다.
    """
    low_passed = (
        CURVATURE_FILTER_ALPHA * raw_curvature
        + (1.0 - CURVATURE_FILTER_ALPHA) * previous_curvature
    )

    max_change = MAX_CURVATURE_CHANGE_PER_SEC * max(dt, 1e-3)

    return clamp(
        low_passed,
        previous_curvature - max_change,
        previous_curvature + max_change
    )


def curvature_sign(curvature):
    if curvature > CURVATURE_SIGN_THRESHOLD:
        return 1

    if curvature < -CURVATURE_SIGN_THRESHOLD:
        return -1

    return 0


def update_s_curve_transition(
    current_time,
    filtered_curvature,
    previous_curve_sign,
    transition_start_time,
    transition_from_steer,
    previous_steer
):
    """
    곡률 부호가 좌↔우로 바뀌는 순간 S자 전환 모드를 시작한다.
    """
    current_sign = curvature_sign(filtered_curvature)

    if (
        current_sign != 0
        and previous_curve_sign != 0
        and current_sign != previous_curve_sign
    ):
        transition_start_time = current_time
        transition_from_steer = previous_steer

    if current_sign != 0:
        previous_curve_sign = current_sign

    transition_elapsed = current_time - transition_start_time

    transition_active = (
        0.0 <= transition_elapsed
        < S_CURVE_TRANSITION_DURATION_SEC
    )

    if transition_active:
        transition_progress = clamp(
            transition_elapsed / S_CURVE_TRANSITION_DURATION_SEC,
            0.0,
            1.0
        )
    else:
        transition_progress = 1.0

    return (
        previous_curve_sign,
        transition_start_time,
        transition_from_steer,
        transition_active,
        transition_progress
    )


def calculate_s_curve_transition_steer(
    transition_from_steer,
    desired_new_steer,
    transition_progress
):
    """
    좌/우 조향 전환 시 반드시 0을 거쳐 반대 방향으로 이동한다.

    전반 50%: 기존 조향 → 0
    후반 50%: 0 → 새로운 조향
    """
    desired_new_steer = clamp(
        desired_new_steer,
        -S_CURVE_FINAL_STEER_LIMIT,
        S_CURVE_FINAL_STEER_LIMIT
    )

    if transition_progress < 0.5:
        first_progress = transition_progress / 0.5

        return transition_from_steer * (
            1.0 - first_progress
        )

    second_progress = (
        transition_progress - 0.5
    ) / 0.5

    return desired_new_steer * second_progress


def curvature_to_normalized_steer(curvature):
    """
    bicycle model의 delta = atan(L * kappa)를 이용해
    경로 곡률을 기본 조향 명령으로 변환한다.
    """
    steering_rad = math.atan(WHEELBASE * curvature)
    steering_rad *= STEER_SIGN

    if steering_rad >= 0.0:
        steering_rad *= LEFT_STEER_GAIN
    else:
        steering_rad *= RIGHT_STEER_GAIN

    return clamp(
        steering_rad / MAX_STEER_RAD,
        -1.0,
        1.0
    )


def apply_curve_sign_hysteresis(
    steer_target,
    curve_angle
):
    """
    장커브에서 커브 반대 방향으로 발생하는 작은 조향만 억제한다.

    이전 조향값을 그대로 반환하지 않는다.
    잘못된 조향을 고정하면 한 번 이탈한 뒤 진동이 지속될 수 있다.
    """
    curve_deg = math.degrees(curve_angle)

    if abs(curve_deg) < CURVE_HOLD_THRESHOLD_DEG:
        return steer_target

    expected_sign = 1.0 if curve_deg > 0.0 else -1.0

    if (
        steer_target * expected_sign < 0.0
        and abs(steer_target) < CURVE_SIGN_HYSTERESIS
    ):
        return 0.0

    return steer_target


def select_adaptive_steer_gains(path_distance):
    """
    정상 추종에서는 곡률 기반 feedforward를 우선하고,
    경로 오차가 커지면 Pure Pursuit feedback 비중을 높인다.
    """
    if path_distance < 0.8:
        return 0.80, 0.25

    if path_distance < 2.0:
        return 0.60, 0.45

    return 0.30, 0.70


def steer_sign(steer):
    if steer > STEER_SIGN_DEADBAND:
        return 1
    if steer < -STEER_SIGN_DEADBAND:
        return -1
    return 0


def update_oscillation_detector(
    raw_steer,
    current_time,
    previous_raw_sign,
    sign_change_times,
    recovery_until
):
    """
    짧은 시간 안에 스티어 부호가 반복 반전되면 진동으로 판단한다.
    """
    current_sign = steer_sign(raw_steer)

    if (
        current_sign != 0
        and previous_raw_sign != 0
        and current_sign != previous_raw_sign
    ):
        sign_change_times.append(current_time)

    cutoff = current_time - OSCILLATION_WINDOW_SEC
    sign_change_times = [
        change_time
        for change_time in sign_change_times
        if change_time >= cutoff
    ]

    if len(sign_change_times) >= OSCILLATION_COUNT_LIMIT:
        recovery_until = max(
            recovery_until,
            current_time + OSCILLATION_RECOVERY_DURATION_SEC
        )
        sign_change_times = []

    if current_sign != 0:
        previous_raw_sign = current_sign

    return (
        previous_raw_sign,
        sign_change_times,
        recovery_until
    )

def second_order_steer_limit(
    target_steer,
    previous_steer,
    previous_steer_rate,
    dt
):
    """
    조향각 변화율뿐 아니라 조향각 가속도까지 제한한다.
    """
    desired_rate = (
        target_steer - previous_steer
    ) / max(dt, 1e-3)

    desired_rate = clamp(
        desired_rate,
        -MAX_STEER_CHANGE_PER_SEC,
        MAX_STEER_CHANGE_PER_SEC
    )

    max_rate_change = MAX_STEER_ACCEL_PER_SEC2 * dt

    steer_rate = clamp(
        desired_rate,
        previous_steer_rate - max_rate_change,
        previous_steer_rate + max_rate_change
    )

    steer = previous_steer + steer_rate * dt
    steer = clamp(steer, -1.0, 1.0)

    return steer, steer_rate


def calculate_signed_cross_track_error(
    path,
    nearest_index,
    ego_x,
    ego_y
):
    """
    nearest path segment 기준 부호 있는 횡방향 오차를 계산한다.

    양수/음수 부호는 경로 진행 방향 기준 차량이 어느 쪽에 있는지를 뜻한다.
    """
    last_index = len(path) - 1

    i0 = min(nearest_index, last_index)
    i1 = min(nearest_index + 1, last_index)

    if i0 == i1:
        i0 = max(0, i0 - 1)

    x0, y0, _ = path[i0]
    x1, y1, _ = path[i1]

    segment_dx = x1 - x0
    segment_dy = y1 - y0
    segment_length = max(
        math.hypot(segment_dx, segment_dy),
        1e-6
    )

    cross_track_error = (
        segment_dx * (ego_y - y0)
        - segment_dy * (ego_x - x0)
    ) / segment_length

    return cross_track_error


def calculate_stanley_steer(
    path,
    nearest_index,
    ego_x,
    ego_y,
    ego_yaw_deg,
    speed_kmh
):
    """
    Stanley control:
        steer = heading_error + atan(k * cte / (v + softening))

    반환 steer는 MORAI 정규화 범위 [-1, 1]이다.
    """
    path_yaw = path_heading(
        path,
        nearest_index,
        step=5
    )

    ego_yaw = math.radians(ego_yaw_deg)

    heading_error = normalize_angle_rad(
        path_yaw - ego_yaw
    )

    cross_track_error = calculate_signed_cross_track_error(
        path,
        nearest_index,
        ego_x,
        ego_y
    )

    speed_mps = max(speed_kmh / 3.6, 0.0)

    cross_track_term = math.atan2(
        STANLEY_CROSS_TRACK_GAIN * cross_track_error,
        speed_mps + STANLEY_SOFTENING_SPEED_MPS
    )

    steering_rad = (
        STANLEY_HEADING_WEIGHT * heading_error
        + STANLEY_CROSSTRACK_WEIGHT * cross_track_term
    )

    steering_rad *= STEER_SIGN

    if steering_rad >= 0.0:
        steering_rad *= LEFT_STEER_GAIN
    else:
        steering_rad *= RIGHT_STEER_GAIN

    steering_normalized = clamp(
        steering_rad / MAX_STEER_RAD,
        -STANLEY_MAX_STEER,
        STANLEY_MAX_STEER
    )

    return (
        steering_normalized,
        heading_error,
        cross_track_error,
        cross_track_term
    )


def select_hybrid_weights(path_distance):
    """
    정상 추종에서는 Pure Pursuit 비중을 높이고,
    경로 오차가 커지면 Stanley 비중을 높인다.
    """
    if path_distance <= HYBRID_DISTANCE_LOW:
        stanley_weight = STANLEY_WEIGHT

    elif path_distance >= HYBRID_DISTANCE_HIGH:
        stanley_weight = HYBRID_STANLEY_WEIGHT_MAX

    else:
        ratio = (
            path_distance - HYBRID_DISTANCE_LOW
        ) / (
            HYBRID_DISTANCE_HIGH - HYBRID_DISTANCE_LOW
        )

        stanley_weight = (
            STANLEY_WEIGHT
            + ratio
            * (
                HYBRID_STANLEY_WEIGHT_MAX
                - STANLEY_WEIGHT
            )
        )

    pure_pursuit_weight = 1.0 - stanley_weight

    return pure_pursuit_weight, stanley_weight


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

    if steering_rad >= 0.0:
        steering_rad *= LEFT_STEER_GAIN
    else:
        steering_rad *= RIGHT_STEER_GAIN

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
        brake = clamp(0.02 * abs(error), 0.0, 0.20)
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
            "time,x,y,yaw,speed,target_speed,"
            "nearest,target,path_distance,curve_deg,"
            "lookahead,steer,accel,brake\n"
        )


def append_log(
    now,
    ego_x,
    ego_y,
    ego_yaw,
    speed_kmh,
    target_speed,
    nearest_index,
    lookahead_index,
    nearest_distance,
    curve_angle,
    lookahead,
    steer,
    accel,
    brake
):
    with open(LOG_FILE, "a", encoding="utf-8") as file:
        file.write(
            f"{now:.3f},{ego_x:.6f},{ego_y:.6f},"
            f"{ego_yaw:.3f},{speed_kmh:.3f},"
            f"{target_speed:.3f},{nearest_index},"
            f"{lookahead_index},{nearest_distance:.3f},"
            f"{math.degrees(curve_angle):.3f},"
            f"{lookahead:.3f},{steer:.5f},"
            f"{accel:.5f},{brake:.5f}\n"
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
    print(
        f"Hybrid steer   : PP {PURE_PURSUIT_WEIGHT:.2f} / "
        f"Stanley {STANLEY_WEIGHT:.2f}"
    )
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
    status_socket.settimeout(1.0)

    status_size = ctypes.sizeof(EgoVehicleStatus)

    sender = Sender(MORAI_IP, CMD_CONTROL_PORT)

    previous_nearest_index = 0
    previous_print_time = 0.0
    previous_time = time.monotonic()

    previous_steer = 0.0
    previous_steer_rate = 0.0
    previous_accel = 0.0
    previous_brake = 0.0

    previous_raw_steer_sign = 0
    steer_sign_change_times = []
    oscillation_recovery_until = 0.0

    previous_filtered_curvature = 0.0
    previous_curve_sign = 0
    s_curve_transition_start_time = -1e9
    s_curve_transition_from_steer = 0.0

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

            curve_angle = calculate_curve_angle(
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

            pp_steer, local_x, local_y, target_valid = (
                calculate_pure_pursuit_steer(
                    ego_x,
                    ego_y,
                    ego_yaw,
                    target_x,
                    target_y
                )
            )

            (
                stanley_steer,
                stanley_heading_error,
                stanley_cross_track_error,
                stanley_cross_track_term
            ) = calculate_stanley_steer(
                path,
                nearest_index,
                ego_x,
                ego_y,
                ego_yaw,
                speed_kmh
            )

            (
                hybrid_pp_weight,
                hybrid_stanley_weight
            ) = select_hybrid_weights(
                nearest_distance
            )

            hybrid_feedback_steer = (
                hybrid_pp_weight * pp_steer
                + hybrid_stanley_weight * stanley_steer
            )

            hybrid_feedback_steer = clamp(
                hybrid_feedback_steer,
                -1.0,
                1.0
            )

            raw_average_curvature = calculate_average_curvature(
                path,
                nearest_index
            )

            average_curvature = filter_curvature(
                raw_average_curvature,
                previous_filtered_curvature,
                dt
            )
            previous_filtered_curvature = average_curvature

            feedforward_steer = curvature_to_normalized_steer(
                average_curvature
            )

            feedforward_gain, feedback_gain = (
                select_adaptive_steer_gains(
                    nearest_distance
                )
            )

            detector_time = time.monotonic()

            (
                previous_curve_sign,
                s_curve_transition_start_time,
                s_curve_transition_from_steer,
                s_curve_transition_active,
                s_curve_transition_progress
            ) = update_s_curve_transition(
                detector_time,
                average_curvature,
                previous_curve_sign,
                s_curve_transition_start_time,
                s_curve_transition_from_steer,
                previous_steer
            )

            if s_curve_transition_active:
                # 전환 구간에서는 Pure Pursuit의 순간 반대 보정을 제한한다.
                limited_hybrid_steer = clamp(
                    hybrid_feedback_steer,
                    -S_CURVE_PP_STEER_LIMIT,
                    S_CURVE_PP_STEER_LIMIT
                )

                desired_new_steer = (
                    0.85 * feedforward_steer
                    + 0.15 * limited_hybrid_steer
                )

                raw_steer = calculate_s_curve_transition_steer(
                    s_curve_transition_from_steer,
                    desired_new_steer,
                    s_curve_transition_progress
                )

                target_speed = min(
                    target_speed,
                    S_CURVE_TRANSITION_SPEED_KMH
                )

            else:
                raw_steer = (
                    feedforward_gain * feedforward_steer
                    + feedback_gain * hybrid_feedback_steer
                )

            raw_steer = clamp(raw_steer, -1.0, 1.0)

            raw_steer = apply_curve_sign_hysteresis(
                raw_steer,
                curve_angle
            )

            (
                previous_raw_steer_sign,
                steer_sign_change_times,
                oscillation_recovery_until
            ) = update_oscillation_detector(
                raw_steer,
                detector_time,
                previous_raw_steer_sign,
                steer_sign_change_times,
                oscillation_recovery_until
            )

            oscillation_recovery_active = (
                detector_time < oscillation_recovery_until
            )

            if oscillation_recovery_active:
                target_speed = min(
                    target_speed,
                    OSCILLATION_RECOVERY_SPEED_KMH
                )

                # 진동 중에는 feedback 과보정을 줄이고
                # 곡률 방향 조향을 중심으로 천천히 복구한다.
                raw_steer = (
                    0.75 * feedforward_steer
                    + 0.20 * hybrid_feedback_steer
                )
                raw_steer = clamp(
                    raw_steer,
                    -0.65,
                    0.65
                )

            # 1차 저역통과 필터
            filtered_steer = (
                STEER_LOW_PASS_ALPHA * raw_steer
                + (1.0 - STEER_LOW_PASS_ALPHA) * previous_steer
            )

            if not target_valid:
                target_speed = min(
                    target_speed,
                    RECOVERY_SPEED_KMH
                )

                # 목표점이 순간적으로 뒤로 갔을 때 이전 조향을 고정하지 않고
                # 곡률 기반 조향으로 서서히 복구한다.
                filtered_steer = (
                    0.80 * previous_steer
                    + 0.20 * feedforward_steer
                )

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
            if abs(filtered_steer) > 0.65:
                raw_accel = min(raw_accel, 0.06)

            if s_curve_transition_active:
                # 전환 중에는 추가로 1차 변화량을 제한해 급격한 반전을 차단한다.
                transition_limited_steer = rate_limit(
                    filtered_steer,
                    previous_steer,
                    S_CURVE_ZERO_CROSS_RATE,
                    dt
                )
            else:
                transition_limited_steer = filtered_steer

            steer, previous_steer_rate = (
                second_order_steer_limit(
                    transition_limited_steer,
                    previous_steer,
                    previous_steer_rate,
                    dt
                )
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

            append_log(
                time.time(),
                ego_x,
                ego_y,
                ego_yaw,
                speed_kmh,
                target_speed,
                nearest_index,
                lookahead_index,
                nearest_distance,
                curve_angle,
                lookahead,
                steer,
                accel,
                brake
            )

            if time.time() - previous_print_time >= 0.2:
                print(
                    f"ego=({ego_x:8.2f},{ego_y:8.2f}) | "
                    f"yaw={ego_yaw:7.2f} | "
                    f"speed={speed_kmh:5.2f}/{target_speed:4.1f} | "
                    f"idx={nearest_index:5d}->{lookahead_index:5d} | "
                    f"dist={nearest_distance:5.2f} | "
                    f"curve={math.degrees(curve_angle):6.1f} | "
                    f"kappa={average_curvature:7.4f} | "
                    f"gain={feedforward_gain:.2f}/{feedback_gain:.2f} | "
                    f"H={hybrid_pp_weight:.2f}/{hybrid_stanley_weight:.2f} | "
                    f"cte={stanley_cross_track_error:5.2f} | "
                    f"he={math.degrees(stanley_heading_error):5.1f} | "
                    f"S={'ON' if s_curve_transition_active else 'OFF'} | "
                    f"osc={'ON' if oscillation_recovery_active else 'OFF'} | "
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
