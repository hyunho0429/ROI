"""MORAI SIM: Drive GPS·IMU UDP 프로토콜 파서.

MORAI 공식 NetworkModule의 ctypes 구조체를 Python 표준 라이브러리만으로
명시적으로 다시 표현한다. 공식 예제가 사용하는 Ubuntu x86 환경의 little-endian
규칙을 고정하여, 운영체제의 native alignment에 따라 값이 달라지지 않게 한다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Optional, Tuple


GPS_PACKET_SIZE = 1028  # header 6 bytes + data 1022 bytes
IMU_PACKET_SIZE = 115  # MORAI NetworkModule 24.R2/26.R1 ctypes structure
LEGACY_IMU_PACKET_SIZE = 107  # 문서에 남아 있는 구형 IMU 형식
IMU_DATA_SIZE = 80  # double 10개
# MORAI 버전에 따라 data_length 필드가 80 또는 88로 전송된다.
# 두 값 모두 뒤의 실제 double 데이터는 10개(80바이트)로 동일하다.
IMU_ACCEPTED_DATA_LENGTHS = (IMU_DATA_SIZE, IMU_DATA_SIZE + 8)

_GPS_HEADERS = (b"$GPRMC", b"$GPGGA")
_IMU_FORMAT = "<9s i 3i i i 10d 2s"
_LEGACY_IMU_FORMAT = "<9s 4i 10d 2s"


class ProtocolError(ValueError):
    """패킷 길이·헤더·필드가 MORAI 프로토콜과 맞지 않을 때 발생한다."""


@dataclass(frozen=True)
class GpsMeasurement:
    """한 개의 GPRMC 또는 GPGGA 패킷에서 추출한 GPS 측정값."""

    sentence_type: str
    utc: str
    latitude: float
    longitude: float
    altitude: Optional[float]
    status: int
    quality: Optional[int]
    raw_sentence: str
    packet_length: int


@dataclass(frozen=True)
class ImuMeasurement:
    """IMU UDP 패킷에서 추출한 값.

    quaternion은 MORAI 구조체 순서인 ``(w, x, y, z)``로 보관한다.
    ROS 메시지로 변환할 때만 ``(x, y, z, w)`` 필드에 넣는다.
    """

    layout: str
    header: bytes
    data_length: int
    auxiliary: Tuple[int, ...]
    sec: int
    nsec: int
    quaternion: Tuple[float, float, float, float]
    angular_velocity: Tuple[float, float, float]
    linear_acceleration: Tuple[float, float, float]
    tail: bytes
    packet_length: int


def _decode_ascii(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("ascii", errors="strict")


def _parse_float(raw: str, field_name: str) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"GPS {field_name} 값이 숫자가 아닙니다: {raw!r}") from exc
    if not math.isfinite(value):
        raise ProtocolError(f"GPS {field_name} 값이 유한하지 않습니다: {raw!r}")
    return value


def _parse_nmea_coordinate(raw: str, direction: str, field_name: str) -> float:
    value = _parse_float(raw, field_name)
    degrees = int(value // 100)
    minutes = value - degrees * 100
    if not 0.0 <= minutes < 60.0:
        raise ProtocolError(f"GPS {field_name} 분 값이 범위를 벗어났습니다: {minutes}")

    decimal = degrees + minutes / 60.0
    if direction in ("S", "W"):
        decimal = -decimal
    elif direction not in ("N", "E"):
        raise ProtocolError(f"GPS {field_name} 방향이 올바르지 않습니다: {direction!r}")
    return decimal


def _verify_nmea_checksum(sentence: str) -> None:
    """NMEA checksum이 있으면 검증한다.

    MORAI 예제 파서는 checksum을 사용하지 않으므로 checksum이 없는 패킷은
    허용하되, checksum이 포함된 패킷이 틀리면 폐기한다.
    """

    if not sentence.startswith("$"):
        raise ProtocolError(f"NMEA sentence가 $로 시작하지 않습니다: {sentence!r}")

    star = sentence.find("*")
    if star < 0:
        return

    supplied = sentence[star + 1 :].strip().split(",", 1)[0]
    if len(supplied) < 2:
        raise ProtocolError(f"NMEA checksum이 짧습니다: {sentence!r}")

    try:
        expected = int(supplied[:2], 16)
    except ValueError as exc:
        raise ProtocolError(f"NMEA checksum이 16진수가 아닙니다: {supplied!r}") from exc

    calculated = 0
    for character in sentence[1:star]:
        calculated ^= ord(character)
    if calculated != expected:
        raise ProtocolError(
            f"NMEA checksum 불일치: 계산={calculated:02X}, 수신={expected:02X}"
        )


def _strip_nmea_checksum(field: str) -> str:
    return field.split("*", 1)[0]


def parse_gps_packet(
    packet: bytes,
    *,
    expected_packet_size: int = GPS_PACKET_SIZE,
    validate_checksum: bool = True,
    allow_short_packet: bool = True,
) -> GpsMeasurement:
    """MORAI GPS UDP 패킷 하나를 파싱한다.

    패킷은 공식 구조체의 ``header[6] + data[1022]`` 형식이다. ``data``는
    NMEA sentence의 header 뒤쪽이며, 남는 바이트는 NUL padding으로 처리한다.
    """

    # MORAI 버전에 따라 1028바이트 NUL-padding 패킷 또는 실제 NMEA 길이의
    # 짧은 패킷(현재 확인값 77바이트)이 들어온다.
    short_packet_is_valid = 6 <= len(packet) < expected_packet_size
    if len(packet) != expected_packet_size and not (allow_short_packet and short_packet_is_valid):
        raise ProtocolError(
            "GPS 패킷 길이 오류: "
            f"expected={expected_packet_size} 또는 짧은 NMEA 패킷, actual={len(packet)}"
        )

    header = packet[:6]
    if header not in _GPS_HEADERS:
        raise ProtocolError(f"GPS header 오류: {header!r}")

    raw_sentence = (header + packet[6:]).split(b"\x00", 1)[0]
    raw_sentence = raw_sentence.rstrip(b"\r\n").decode("ascii", errors="strict")
    if validate_checksum:
        _verify_nmea_checksum(raw_sentence)

    fields = raw_sentence.split(",")
    sentence_type = header.decode("ascii")[1:]

    if sentence_type == "GPRMC":
        if len(fields) < 7:
            raise ProtocolError(f"GPRMC 필드 수가 부족합니다: {raw_sentence!r}")
        utc = fields[1]
        valid = fields[2].upper() == "A"
        latitude = _parse_nmea_coordinate(fields[3], fields[4], "latitude")
        longitude = _parse_nmea_coordinate(fields[5], fields[6], "longitude")
        return GpsMeasurement(
            sentence_type=sentence_type,
            utc=utc,
            latitude=latitude,
            longitude=longitude,
            altitude=None,
            status=1 if valid else 0,
            quality=None,
            raw_sentence=raw_sentence,
            packet_length=len(packet),
        )

    if len(fields) < 10:
        raise ProtocolError(f"GPGGA 필드 수가 부족합니다: {raw_sentence!r}")
    utc = fields[1]
    latitude = _parse_nmea_coordinate(fields[2], fields[3], "latitude")
    longitude = _parse_nmea_coordinate(fields[4], fields[5], "longitude")
    try:
        quality = int(_strip_nmea_checksum(fields[6]))
    except ValueError as exc:
        raise ProtocolError(f"GPGGA quality 값이 정수가 아닙니다: {fields[6]!r}") from exc
    altitude = _parse_float(fields[9], "altitude")
    return GpsMeasurement(
        sentence_type=sentence_type,
        utc=utc,
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
        status=quality,
        quality=quality,
        raw_sentence=raw_sentence,
        packet_length=len(packet),
    )


def _check_imu_timestamp(sec: int, nsec: int) -> None:
    if sec < 0 or not 0 <= nsec < 1_000_000_000:
        raise ProtocolError(f"IMU timestamp 오류: sec={sec}, nsec={nsec}")


def parse_imu_packet(
    packet: bytes,
    *,
    expected_packet_size: int = IMU_PACKET_SIZE,
    allow_legacy_107: bool = True,
    require_data_length_80: bool = True,
) -> ImuMeasurement:
    """MORAI IMU UDP 패킷을 명시적 little-endian 구조체로 파싱한다.

    MORAI NetworkModule의 현재 구조체는 115바이트이다. 일부 23.R1 문서에는
    107바이트로 기록되어 있어, 보조 정수 1개만 있는 레거시 형식도 옵션으로
    해석한다. 패킷 길이로 레이아웃을 결정하므로 필드가 밀려서 조용히 잘못
    읽히지 않는다.
    """

    accepted_lengths = {expected_packet_size}
    if allow_legacy_107:
        accepted_lengths.add(LEGACY_IMU_PACKET_SIZE)
    if len(packet) not in accepted_lengths:
        raise ProtocolError(
            f"IMU 패킷 길이 오류: accepted={sorted(accepted_lengths)}, actual={len(packet)}"
        )

    if len(packet) == LEGACY_IMU_PACKET_SIZE:
        values = struct.unpack(_LEGACY_IMU_FORMAT, packet)
        header = values[0]
        data_length = values[1]
        auxiliary = (values[2],)
        sec = values[3]
        nsec = values[4]
        doubles = values[5:15]
        tail = values[15]
        layout = "legacy_107"
    else:
        values = struct.unpack(_IMU_FORMAT, packet)
        header = values[0]
        data_length = values[1]
        auxiliary = tuple(values[2:5])
        sec = values[5]
        nsec = values[6]
        doubles = values[7:17]
        tail = values[17]
        layout = "networkmodule_115"

    if require_data_length_80 and data_length not in IMU_ACCEPTED_DATA_LENGTHS:
        raise ProtocolError(
            "IMU data_length 오류: "
            f"expected={list(IMU_ACCEPTED_DATA_LENGTHS)}, actual={data_length}"
        )
    _check_imu_timestamp(sec, nsec)

    return ImuMeasurement(
        layout=layout,
        header=header,
        data_length=data_length,
        auxiliary=auxiliary,
        sec=sec,
        nsec=nsec,
        quaternion=(doubles[0], doubles[1], doubles[2], doubles[3]),
        angular_velocity=(doubles[4], doubles[5], doubles[6]),
        linear_acceleration=(doubles[7], doubles[8], doubles[9]),
        tail=tail,
        packet_length=len(packet),
    )


def normalize_quaternion(
    quaternion: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    """쿼터니언을 ROS 메시지에 넣기 전에 단위 쿼터니언으로 정규화한다."""

    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12:
        raise ProtocolError("IMU orientation 쿼터니언 norm이 0입니다")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]
