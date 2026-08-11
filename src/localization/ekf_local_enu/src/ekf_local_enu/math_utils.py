"""EKF에서 사용하는 작은 각도·쿼터니언 함수."""

from __future__ import annotations

import math
from typing import Optional, Tuple


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> Optional[float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw_rad
    return 0.0, 0.0, math.sin(half), math.cos(half)
