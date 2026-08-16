#!/usr/bin/env python3
"""Global Path를 기준으로 lateral offset Candidate Path를 생성하는 모듈."""

from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple

from purepursuit_mgeo.path import PathPoint, nearest_path_index


DEFAULT_OFFSETS_M = (-1.5, -0.75, 0.0, 0.75, 1.5)


def _path_tangent(
    points: Sequence[PathPoint],
    index: int,
) -> Tuple[float, float]:
    """현재 PathPoint에서 Global Path의 진행방향 단위벡터를 계산한다."""
    if len(points) < 2:
        return 1.0, 0.0

    if index <= 0:
        p0 = points[0]
        p1 = points[1]
    elif index >= len(points) - 1:
        p0 = points[-2]
        p1 = points[-1]
    else:
        p0 = points[index - 1]
        p1 = points[index + 1]

    dx = p1.x - p0.x
    dy = p1.y - p0.y
    length = math.hypot(dx, dy)

    if length < 1e-9:
        return 1.0, 0.0

    return dx / length, dy / length


def _local_end_index(
    points: Sequence[PathPoint],
    start_index: int,
    length_m: float,
) -> int:
    """start_index부터 path를 따라 length_m만큼 떨어진 index를 찾는다."""
    if length_m <= 0.0:
        return start_index

    accumulated = 0.0
    start_index = max(0, min(start_index, len(points) - 1))

    for i in range(start_index, len(points) - 1):
        p0 = points[i]
        p1 = points[i + 1]
        accumulated += math.hypot(p1.x - p0.x, p1.y - p0.y)

        if accumulated >= length_m:
            return i + 1

    return len(points) - 1


def generate_candidate_path(
    points: Sequence[PathPoint],
    start_index: int,
    lateral_offset_m: float,
    local_length_m: float = 30.0,
) -> List[PathPoint]:
    """
    Global Path의 각 점을 path normal 방향으로 lateral_offset만큼 이동한다.

    +offset: Global Path 기준 왼쪽
    -offset: Global Path 기준 오른쪽
    """
    if len(points) < 2:
        return []

    start_index = max(0, min(start_index, len(points) - 2))
    end_index = _local_end_index(points, start_index, local_length_m)

    candidate: List[PathPoint] = []

    for i in range(start_index, end_index + 1):
        p = points[i]
        tx, ty = _path_tangent(points, i)

        # 진행방향의 왼쪽 법선
        nx = -ty
        ny = tx

        candidate.append(
            PathPoint(
                p.x + nx * lateral_offset_m,
                p.y + ny * lateral_offset_m,
                p.z,
            )
        )

    return candidate


def generate_candidate_paths(
    points: Sequence[PathPoint],
    ego_x: float,
    ego_y: float,
    offsets_m: Sequence[float] = DEFAULT_OFFSETS_M,
    local_length_m: float = 30.0,
) -> Dict[float, List[PathPoint]]:
    """현재 Ego 위치를 기준으로 모든 Candidate Path를 생성한다."""
    nearest = nearest_path_index(points, ego_x, ego_y)

    return {
        float(offset): generate_candidate_path(
            points,
            nearest,
            float(offset),
            local_length_m,
        )
        for offset in offsets_m
    }
