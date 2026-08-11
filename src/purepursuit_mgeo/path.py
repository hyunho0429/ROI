"""MGeo global path 파일과 Pure Pursuit 기하 계산."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    z: float


def load_mgeo_path(path_file: str) -> List[PathPoint]:
    """공백으로 구분된 MGeo x y z 경로를 읽는다."""

    points: List[PathPoint] = []
    with open(path_file, "r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.replace(",", " ").split()
            if len(fields) < 2:
                continue
            try:
                x = float(fields[0])
                y = float(fields[1])
                z = float(fields[2]) if len(fields) >= 3 else 0.0
            except ValueError:
                # 헤더가 있더라도 조용히 건너뛴다.
                if not points:
                    continue
                raise ValueError(f"경로 파일 {line_number}번째 줄을 읽을 수 없다: {raw_line!r}")
            points.append(PathPoint(x, y, z))

    if len(points) < 2:
        raise ValueError(f"MGeo 경로에 사용할 점이 부족하다: {path_file}")
    return points


def nearest_path_index(points: Sequence[PathPoint], x: float, y: float) -> int:
    return min(
        range(len(points)),
        key=lambda index: (points[index].x - x) ** 2 + (points[index].y - y) ** 2,
    )


def target_at_distance(
    points: Sequence[PathPoint], start_index: int, lookahead_distance: float
) -> Tuple[PathPoint, int]:
    """start_index부터 경로를 따라 lookahead 거리 뒤의 점을 반환한다."""

    accumulated = 0.0
    index = max(0, min(start_index, len(points) - 1))
    while index < len(points) - 1:
        current = points[index]
        following = points[index + 1]
        segment = math.hypot(following.x - current.x, following.y - current.y)
        if accumulated + segment >= lookahead_distance and segment > 1e-9:
            ratio = (lookahead_distance - accumulated) / segment
            return PathPoint(
                current.x + ratio * (following.x - current.x),
                current.y + ratio * (following.y - current.y),
                current.z + ratio * (following.z - current.z),
            ), index
        accumulated += segment
        index += 1
    return points[-1], len(points) - 1


class MgeoPurePursuit:
    def __init__(
        self,
        points: Sequence[PathPoint],
        wheelbase_m: float,
        lookahead_min_m: float,
        lookahead_gain: float,
        goal_tolerance_m: float,
        steering_sign: float = 1.0,
    ) -> None:
        if wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m은 0보다 커야 한다.")
        self.points = list(points)
        self.wheelbase_m = wheelbase_m
        self.lookahead_min_m = lookahead_min_m
        self.lookahead_gain = lookahead_gain
        self.goal_tolerance_m = goal_tolerance_m
        self.steering_sign = 1.0 if steering_sign >= 0.0 else -1.0

    def compute(
        self,
        x: float,
        y: float,
        yaw_rad: float,
        speed_mps: float,
    ) -> Tuple[float, bool, PathPoint, int, float]:
        lookahead = max(
            self.lookahead_min_m,
            self.lookahead_min_m + self.lookahead_gain * max(0.0, speed_mps),
        )
        nearest = nearest_path_index(self.points, x, y)
        distance_to_goal = math.hypot(self.points[-1].x - x, self.points[-1].y - y)
        if nearest >= len(self.points) - 2 and distance_to_goal <= self.goal_tolerance_m:
            return 0.0, True, self.points[-1], nearest, lookahead

        target, target_index = target_at_distance(self.points, nearest, lookahead)
        dx = target.x - x
        dy = target.y - y

        # map 좌표를 base_link(뒤 차축 중앙) 좌표로 변환한다.
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        target_x_body = cos_yaw * dx + sin_yaw * dy
        target_y_body = -sin_yaw * dx + cos_yaw * dy
        actual_lookahead = max(math.hypot(target_x_body, target_y_body), 1e-3)

        alpha = math.atan2(target_y_body, target_x_body)
        curvature = 2.0 * math.sin(alpha) / actual_lookahead
        steering = math.atan(self.wheelbase_m * curvature)
        steering *= self.steering_sign
        return steering, False, target, target_index, actual_lookahead
