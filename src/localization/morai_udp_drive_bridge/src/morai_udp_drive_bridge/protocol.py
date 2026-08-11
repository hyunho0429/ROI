"""MORAI NetworkModule Ego UDP 프로토콜.

공식 24.R2 계열 구조체의 ``_pack_ = 1`` little-endian 레이아웃을
Python struct로 명시한다. 차량 상태 수신은 181바이트, 제어 송신은
55바이트이다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Tuple


EGO_STATUS_FORMAT = "<11s i 3i i i b b f i 24f 38s 2s"
EGO_STATUS_PACKET_SIZE = struct.calcsize(EGO_STATUS_FORMAT)
EGO_CTRL_CMD_FORMAT = "<14s i 3i 3b 5f 2s"
EGO_CTRL_CMD_PACKET_SIZE = struct.calcsize(EGO_CTRL_CMD_FORMAT)


class ProtocolError(ValueError):
    """MORAI Ego UDP 패킷이 프로토콜과 맞지 않을 때 발생한다."""


@dataclass(frozen=True)
class EgoVehicleStatusMeasurement:
    sec: int
    nsec: int
    ctrl_mode: int
    gear: int
    signed_velocity_kmh: float
    map_data_id: int
    accel_pedal: float
    brake_pedal: float
    size_x_m: float
    size_y_m: float
    size_z_m: float
    overhang_m: float
    wheelbase_m: float
    rear_overhang_m: float
    pos_x_m: float
    pos_y_m: float
    pos_z_m: float
    roll_deg: float
    pitch_deg: float
    heading_deg: float
    velocity_x_kmh: float
    velocity_y_kmh: float
    velocity_z_kmh: float
    angular_velocity_x_deg_s: float
    angular_velocity_y_deg_s: float
    angular_velocity_z_deg_s: float
    acceleration_x_mps2: float
    acceleration_y_mps2: float
    acceleration_z_mps2: float
    steer_deg: float
    link_id: str
    packet_length: int


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ProtocolError(f"EgoVehicleStatus {name} 값이 유한하지 않다: {value}")
    return value


def parse_ego_vehicle_status(packet: bytes) -> EgoVehicleStatusMeasurement:
    if len(packet) != EGO_STATUS_PACKET_SIZE:
        raise ProtocolError(
            f"EgoVehicleStatus 패킷 길이 오류: expected={EGO_STATUS_PACKET_SIZE}, actual={len(packet)}"
        )

    values = struct.unpack(EGO_STATUS_FORMAT, packet)
    sec = int(values[5])
    nsec = int(values[6])
    if sec < 0 or not 0 <= nsec < 1_000_000_000:
        raise ProtocolError(f"EgoVehicleStatus timestamp 오류: sec={sec}, nsec={nsec}")

    floats = [_finite(value, "float") for value in values[11:35]]
    link_id = values[35].split(b"\x00", 1)[0].decode("utf-8", errors="replace")
    return EgoVehicleStatusMeasurement(
        sec=sec,
        nsec=nsec,
        ctrl_mode=int(values[7]),
        gear=int(values[8]),
        signed_velocity_kmh=_finite(values[9], "signed_velocity_kmh"),
        map_data_id=int(values[10]),
        accel_pedal=floats[0],
        brake_pedal=floats[1],
        size_x_m=floats[2],
        size_y_m=floats[3],
        size_z_m=floats[4],
        overhang_m=floats[5],
        wheelbase_m=floats[6],
        rear_overhang_m=floats[7],
        pos_x_m=floats[8],
        pos_y_m=floats[9],
        pos_z_m=floats[10],
        roll_deg=floats[11],
        pitch_deg=floats[12],
        heading_deg=floats[13],
        velocity_x_kmh=floats[14],
        velocity_y_kmh=floats[15],
        velocity_z_kmh=floats[16],
        angular_velocity_x_deg_s=floats[17],
        angular_velocity_y_deg_s=floats[18],
        angular_velocity_z_deg_s=floats[19],
        acceleration_x_mps2=floats[20],
        acceleration_y_mps2=floats[21],
        acceleration_z_mps2=floats[22],
        steer_deg=floats[23],
        link_id=link_id,
        packet_length=len(packet),
    )


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
