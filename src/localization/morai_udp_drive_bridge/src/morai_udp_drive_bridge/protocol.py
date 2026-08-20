"""MORAI Competition Vehicle Status 수신과 Ego Ctrl Cmd 송신 프로토콜.

차량 상태는 EgoVehicleStatus와 동일한 ``#MoraiInfo$`` 패킷 레이아웃을 사용한다.
Competition 네트워크에서는 이 구조의 일부 정보만 유효하므로 ROS 호환 출력도
실제 제공 필드만 사용한다. 제어 송신은 공식 24.R2 계열 EgoCtrlCmd 55바이트
레이아웃을 유지한다.
"""

from __future__ import annotations

import struct

from path_planning.morai_udp_competition_status import (
    BASE_PACKET_SIZE as COMPETITION_STATUS_BASE_PACKET_SIZE,
    EXTENDED_PACKET_SIZE as COMPETITION_STATUS_EXTENDED_PACKET_SIZE,
    CompetitionStatusPacketError,
    parse_competition_vehicle_status as _parse_competition_vehicle_status,
)


COMPETITION_STATUS_HOST_PORT = 9080
COMPETITION_STATUS_PORT = 9081
COMPETITION_STATUS_PACKET_SIZES = (
    COMPETITION_STATUS_BASE_PACKET_SIZE,
    COMPETITION_STATUS_EXTENDED_PACKET_SIZE,
)
COMPETITION_STATUS_MAX_PACKET_SIZE = max(COMPETITION_STATUS_PACKET_SIZES)
EGO_CTRL_CMD_FORMAT = "<14s i 3i 3b 5f 2s"
EGO_CTRL_CMD_PACKET_SIZE = struct.calcsize(EGO_CTRL_CMD_FORMAT)


class ProtocolError(ValueError):
    """MORAI UDP 패킷이 선택한 대회 프로토콜과 맞지 않을 때 발생한다."""


def parse_competition_vehicle_status(packet: bytes):
    """대회 전용 181/229바이트 상태 패킷을 엄격하게 해석한다."""

    try:
        return _parse_competition_vehicle_status(packet)
    except CompetitionStatusPacketError as exc:
        raise ProtocolError(str(exc)) from exc


def build_ego_ctrl_cmd(
    *,
    cmd_type: int,
    velocity_kmh: float = 0.0,
    acceleration_mps2: float = 0.0,
    accel: float = 0.0,
    brake: float = 0.0,
    steer_normalized: float = 0.0,
    ctrl_mode: int = 2,
    gear: int = 4,
) -> bytes:
    """공식 EgoCtrlCmd 55바이트 패킷을 생성한다.

    ``steer_normalized``는 실제 앞바퀴각/최대앞바퀴각이며 -1~1이다.
    ``velocity_kmh``는 MORAI UDP 프로토콜 단위인 km/h이다.
    """

    values = (
        b"#MoraiCtrlCmd$",
        23,
        0,
        0,
        0,
        int(ctrl_mode),
        int(gear),
        int(cmd_type),
        float(velocity_kmh),
        float(acceleration_mps2),
        max(0.0, min(1.0, float(accel))),
        max(0.0, min(1.0, float(brake))),
        max(-1.0, min(1.0, float(steer_normalized))),
        b"\r\n",
    )
    packet = struct.pack(EGO_CTRL_CMD_FORMAT, *values)
    if len(packet) != EGO_CTRL_CMD_PACKET_SIZE:
        raise ProtocolError(f"EgoCtrlCmd 생성 길이 오류: {len(packet)}")
    return packet
