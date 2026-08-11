"""WGS84 위도·경도를 MGeo의 UTM52 local ENU로 변환한다.

외부 pyproj 패키지 없이 실행할 수 있도록 WGS84 UTM Zone 52N 공식을
순수 Python으로 구현했다. MGeo의 global_info.json에 기록된
local_origin_in_global 값을 차감하여 map frame 좌표를 만든다.
"""

from __future__ import annotations

import math
from typing import Iterable, Tuple


WGS84_A = 6378137.0
WGS84_ECC_SQ = 0.0066943799901413165
UTM_K0 = 0.9996
UTM_ZONE = 52
UTM_CENTRAL_MERIDIAN_DEG = 129.0
UTM_FALSE_EASTING = 500000.0


def wgs84_to_utm52(latitude_deg: float, longitude_deg: float) -> Tuple[float, float]:
    """WGS84 경위도를 UTM Zone 52N (easting, northing)으로 변환한다."""

    lat = math.radians(float(latitude_deg))
    lon = math.radians(float(longitude_deg))
    central_meridian = math.radians(UTM_CENTRAL_MERIDIAN_DEG)

    ecc_prime_sq = WGS84_ECC_SQ / (1.0 - WGS84_ECC_SQ)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    tan_lat = math.tan(lat)

    n = WGS84_A / math.sqrt(1.0 - WGS84_ECC_SQ * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = ecc_prime_sq * cos_lat * cos_lat
    a = cos_lat * (lon - central_meridian)

    m = WGS84_A * (
        (1.0 - WGS84_ECC_SQ / 4.0 - 3.0 * WGS84_ECC_SQ**2 / 64.0
         - 5.0 * WGS84_ECC_SQ**3 / 256.0) * lat
        - (3.0 * WGS84_ECC_SQ / 8.0 + 3.0 * WGS84_ECC_SQ**2 / 32.0
           + 45.0 * WGS84_ECC_SQ**3 / 1024.0) * math.sin(2.0 * lat)
        + (15.0 * WGS84_ECC_SQ**2 / 256.0
           + 45.0 * WGS84_ECC_SQ**3 / 1024.0) * math.sin(4.0 * lat)
        - (35.0 * WGS84_ECC_SQ**3 / 3072.0) * math.sin(6.0 * lat)
    )

    easting = UTM_K0 * n * (
        a
        + (1.0 - t + c) * a**3 / 6.0
        + (5.0 - 18.0 * t + t**2 + 72.0 * c - 58.0 * ecc_prime_sq)
        * a**5 / 120.0
    ) + UTM_FALSE_EASTING

    northing = UTM_K0 * (
        m
        + n * tan_lat * (
            a**2 / 2.0
            + (5.0 - t + 9.0 * c + 4.0 * c**2) * a**4 / 24.0
            + (61.0 - 58.0 * t + t**2 + 600.0 * c
               - 330.0 * ecc_prime_sq) * a**6 / 720.0
        )
    )

    return easting, northing


def gps_to_mgeo(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    local_origin_utm: Iterable[float],
) -> Tuple[float, float, float]:
    """GPS를 MGeo local ENU 좌표로 변환한다."""

    easting, northing = wgs84_to_utm52(latitude_deg, longitude_deg)
    origin = list(local_origin_utm)
    if len(origin) != 3:
        raise ValueError("local_origin_utm은 [easting, northing, z] 3개 값이어야 한다.")
    return (
        easting - float(origin[0]),
        northing - float(origin[1]),
        float(altitude_m) - float(origin[2]),
    )


def wrap_angle(angle_rad: float) -> float:
    """각도를 [-pi, pi] 범위로 정규화한다."""

    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))
