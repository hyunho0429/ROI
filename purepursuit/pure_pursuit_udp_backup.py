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

WHEELBASE = 2.7             # 차량 축거 추정값 [m]
LOOKAHEAD_DISTANCE = 8.5    # 전방 주시거리 [m]
MAX_STEER_RAD = math.radians(30.0)

TARGET_SPEED_KMH = 10.0     # 첫 시험은 저속으로
MAX_PATH_DISTANCE = 15.0    # 경로에서 이 거리 이상 떨어지면 출발 금지
GOAL_DISTANCE = 3.0         # 마지막 점 도착 판정 거리 [m]

STEER_SIGN = 1.0
# 차가 목표 경로의 반대 방향으로 조향하면 -1.0으로 변경


def load_path(path_file):
    """공백으로 구분된 x y z 경로 파일을 읽는다."""
    path = []

    with open(path_file, "r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            values = line.replace(",", " ").split()

            if len(values) < 2:
                print(f"[경고] {line_number}번째 줄을 건너뜀: {line}")
                continue

            try:
                x = float(values[0])
                y = float(values[1])
                z = float(values[2]) if len(values) >= 3 else 0.0
                path.append((x, y, z))

            except ValueError:
                print(f"[경고] 숫자로 변환할 수 없는 줄: {line}")

    if len(path) < 2:
        raise RuntimeError("경로점이 2개 미만입니다.")

    return path


def distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def find_nearest_index(path, ego_x, ego_y, start_index=0):
    """
    현재 차량과 가장 가까운 경로점을 찾는다.
    이전 인덱스보다 뒤쪽을 우선 탐색해 경로 인덱스 역행을 방지한다.
    """
    search_start = max(0, start_index - 10)

    nearest_index = search_start
    nearest_distance = float("inf")

    for index in range(search_start, len(path)):
        px, py, _ = path[index]
        current_distance = distance(ego_x, ego_y, px, py)

        if current_distance < nearest_distance:
            nearest_distance = current_distance
            nearest_index = index

        # 가장 가까운 점을 지난 뒤 거리가 충분히 증가하면 탐색 종료
        elif index > nearest_index + 100:
            break

    return nearest_index, nearest_distance


def find_lookahead_index(path, nearest_index, ego_x, ego_y, lookahead):
    """현재 위치에서 lookahead 이상 떨어진 첫 번째 전방 경로점을 선택한다."""
    for index in range(nearest_index, len(path)):
        px, py, _ = path[index]

        if distance(ego_x, ego_y, px, py) >= lookahead:
            return index

    return len(path) - 1


def calculate_pure_pursuit_steer(
    ego_x,
    ego_y,
    ego_yaw_deg,
    target_x,
    target_y
):
    """
    Pure Pursuit 조향 계산.

    MORAI yaw:
      0도   = +x 방향
      90도  = +y 방향
    """
    yaw_rad = math.radians(ego_yaw_deg)

    dx = target_x - ego_x
    dy = target_y - ego_y

    # 월드 좌표 목표점을 차량 좌표로 변환
    local_x = math.cos(yaw_rad) * dx + math.sin(yaw_rad) * dy
    local_y = -math.sin(yaw_rad) * dx + math.cos(yaw_rad) * dy

    lookahead_actual = max(math.hypot(local_x, local_y), 0.1)

    # 목표점이 차량 뒤쪽에 있으면 조향하지 않음
    if local_x <= 0.0:
        return 0.0, local_x, local_y

    steering_rad = math.atan2(
        2.0 * WHEELBASE * local_y,
        lookahead_actual ** 2
    )

    steering_rad *= STEER_SIGN

    # MORAI steer 범위 -1 ~ 1로 정규화
    steering_normalized = steering_rad / MAX_STEER_RAD
    steering_normalized = max(-1.0, min(1.0, steering_normalized))

    return steering_normalized, local_x, local_y


def calculate_longitudinal_control(speed_kmh):
    """간단한 목표속도 기반 가속·브레이크 제어."""
    speed_error = TARGET_SPEED_KMH - speed_kmh

    if speed_error > 2.0:
        accel = min(0.35, 0.08 + 0.025 * speed_error)
        brake = 0.0

    elif speed_error > 0.0:
        accel = min(0.15, 0.02 + 0.02 * speed_error)
        brake = 0.0

    elif speed_error > -2.0:
        accel = 0.0
        brake = 0.0

    else:
        accel = 0.0
        brake = min(0.35, 0.04 * abs(speed_error))

    return accel, brake


def make_command(accel, brake, steer):
    command = EgoCtrlCmd()

    command.ctrl_mode = 2
    command.gear = 4
    command.cmd_type = 1

    command.velocity = 0.0
    command.acceleration = 0.0
    command.accel = float(accel)
    command.brake = float(brake)
    command.steer = float(steer)

    return command


def send_stop(sender, duration=2.0):
    """가속을 끄고 일정 시간 브레이크 명령을 보낸다."""
    stop_command = make_command(
        accel=0.0,
        brake=1.0,
        steer=0.0
    )

    end_time = time.time() + duration

    while time.time() < end_time:
        sender.send(stop_command)
        time.sleep(0.05)


def main():
    path = load_path(PATH_FILE)

    print("=" * 65)
    print("MORAI Pure Pursuit UDP Controller")
    print(f"경로 파일       : {PATH_FILE}")
    print(f"총 경로점       : {len(path)}개")
    print(f"첫 경로점       : x={path[0][0]:.3f}, y={path[0][1]:.3f}")
    print(f"마지막 경로점   : x={path[-1][0]:.3f}, y={path[-1][1]:.3f}")
    print(f"목표 속도       : {TARGET_SPEED_KMH:.1f} km/h")
    print(f"Look-ahead      : {LOOKAHEAD_DISTANCE:.1f} m")
    print("=" * 65)
    print("MORAI에서 Q를 눌러 AV-ExternalCtrl로 설정하세요.")
    print("종료: Ctrl+C")
    print()

    status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 ** 20)
    status_socket.bind((UBUNTU_IP, EGO_STATUS_PORT))
    status_socket.settimeout(1.0)

    status_size = ctypes.sizeof(EgoVehicleStatus)

    print(
        f"Ego Status 수신 대기: "
        f"{UBUNTU_IP}:{EGO_STATUS_PORT}, packet size={status_size}"
    )

    sender = Sender(
        MORAI_IP,
        CMD_CONTROL_PORT
    )

    previous_nearest_index = 0
    previous_print_time = 0.0

    try:
        while True:
            try:
                raw_data, sender_address = status_socket.recvfrom(4096)
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

            ego_x = float(status.pos_x)
            ego_y = float(status.pos_y)
            ego_yaw = float(status.yaw)
            speed_kmh = abs(float(status.signed_vel))

            nearest_index, nearest_distance = find_nearest_index(
                path,
                ego_x,
                ego_y,
                previous_nearest_index
            )

            previous_nearest_index = max(
                previous_nearest_index,
                nearest_index
            )

            # 차량이 경로에서 너무 멀면 출발하지 않음
            if nearest_distance > MAX_PATH_DISTANCE:
                command = make_command(
                    accel=0.0,
                    brake=1.0,
                    steer=0.0
                )
                sender.send(command)

                if time.time() - previous_print_time >= 0.5:
                    print(
                        f"[출발 금지] 차량이 경로에서 {nearest_distance:.2f} m "
                        f"떨어져 있습니다. 차량을 경로 시작점 근처로 이동하세요."
                    )
                    previous_print_time = time.time()

                continue

            lookahead_index = find_lookahead_index(
                path,
                nearest_index,
                ego_x,
                ego_y,
                LOOKAHEAD_DISTANCE
            )

            target_x, target_y, _ = path[lookahead_index]

            steer, local_x, local_y = calculate_pure_pursuit_steer(
                ego_x,
                ego_y,
                ego_yaw,
                target_x,
                target_y
            )

            goal_x, goal_y, _ = path[-1]
            goal_distance = distance(
                ego_x,
                ego_y,
                goal_x,
                goal_y
            )

            # 최종 목적지 도착
            if (
                lookahead_index >= len(path) - 2
                and goal_distance <= GOAL_DISTANCE
            ):
                print("최종 목적지에 도착했습니다.")
                send_stop(sender)
                break

            accel, brake = calculate_longitudinal_control(speed_kmh)

            # 조향이 클 때 속도를 낮추기 위해 가속 제한
            if abs(steer) > 0.6:
                accel = min(accel, 0.08)

                if speed_kmh > 8.0:
                    brake = max(brake, 0.08)

            command = make_command(
                accel=accel,
                brake=brake,
                steer=steer
            )

            sender.send(command)

            if time.time() - previous_print_time >= 0.2:
                print(
                    f"ego=({ego_x:8.2f}, {ego_y:8.2f}) | "
                    f"yaw={ego_yaw:7.2f} | "
                    f"speed={speed_kmh:5.2f} | "
                    f"nearest={nearest_index:5d} | "
                    f"target={lookahead_index:5d} | "
                    f"path_dist={nearest_distance:5.2f} | "
                    f"local=({local_x:5.2f}, {local_y:5.2f}) | "
                    f"steer={steer:6.3f} | "
                    f"accel={accel:5.2f} | "
                    f"brake={brake:5.2f}"
                )
                previous_print_time = time.time()

            time.sleep(0.02)

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
