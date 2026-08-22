#!/usr/bin/env python3
"""MGeo Link 기반 차선변경 Candidate Path 생성.

핵심:
- 고정 lateral offset을 사용하지 않는다.
- 현재 Global Path와 가장 가까운 MGeo Link를 찾는다.
- link_set.json의 좌/우 차선변경 가능 정보로 실제 인접 차선만 선택한다.
- 각 인접 차선에 대해 차선변경 시작거리/변경길이를 다르게 샘플링한다.
- 시작/종료 접선이 자연스럽게 이어지는 Cubic Hermite 곡선을 생성한다.

현재 단계에서는 장애물 판단/Cost/Pure Pursuit 제어와 연결하지 않는다.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from purepursuit_mgeo.path import PathPoint, nearest_path_index


@dataclass
class MGeoLink:
    idx: str
    points: List[PathPoint]
    can_move_left_lane: bool
    can_move_right_lane: bool
    left_lane_change_dst_link_idx: Optional[str]
    right_lane_change_dst_link_idx: Optional[str]
    road_id: Optional[str]
    ego_lane: Optional[int]


@dataclass
class LaneChangeCandidate:
    side: str                    # "left" / "right"
    target_link_idx: str
    start_distance_m: float
    change_length_m: float
    path: List[PathPoint]


def _dist2(a: PathPoint, b: PathPoint) -> float:
    return (a.x - b.x) ** 2 + (a.y - b.y) ** 2


def _distance(a: PathPoint, b: PathPoint) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def _path_tangent(points: Sequence[PathPoint], index: int) -> Tuple[float, float]:
    """주어진 polyline에서 index 부근의 진행방향 단위벡터."""
    if len(points) < 2:
        return 1.0, 0.0

    index = max(0, min(index, len(points) - 1))

    if index == 0:
        p0, p1 = points[0], points[1]
    elif index == len(points) - 1:
        p0, p1 = points[-2], points[-1]
    else:
        p0, p1 = points[index - 1], points[index + 1]

    dx = p1.x - p0.x
    dy = p1.y - p0.y
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        return 1.0, 0.0
    return dx / norm, dy / norm


def _index_at_distance(
    points: Sequence[PathPoint],
    start_index: int,
    distance_m: float,
) -> int:
    """start_index부터 path를 따라 distance_m 앞의 index."""
    if not points:
        return 0

    start_index = max(0, min(start_index, len(points) - 1))
    if distance_m <= 0.0:
        return start_index

    accumulated = 0.0
    for i in range(start_index, len(points) - 1):
        segment = _distance(points[i], points[i + 1])
        accumulated += segment
        if accumulated >= distance_m:
            return i + 1

    return len(points) - 1


def _slice_until_distance(
    points: Sequence[PathPoint],
    start_index: int,
    distance_m: float,
) -> List[PathPoint]:
    end = _index_at_distance(points, start_index, distance_m)
    return list(points[start_index : end + 1])


def load_mgeo_links(link_set_file: str) -> Dict[str, MGeoLink]:
    """MGeo link_set.json 로드."""
    with open(link_set_file, "r", encoding="utf-8") as stream:
        raw_links = json.load(stream)

    links: Dict[str, MGeoLink] = {}

    for raw in raw_links:
        raw_points = raw.get("points") or []
        if len(raw_points) < 2:
            continue

        points = [
            PathPoint(
                float(point[0]),
                float(point[1]),
                float(point[2]) if len(point) >= 3 else 0.0,
            )
            for point in raw_points
        ]

        idx = str(raw["idx"])
        links[idx] = MGeoLink(
            idx=idx,
            points=points,
            can_move_left_lane=bool(raw.get("can_move_left_lane", False)),
            can_move_right_lane=bool(raw.get("can_move_right_lane", False)),
            left_lane_change_dst_link_idx=raw.get("left_lane_change_dst_link_idx"),
            right_lane_change_dst_link_idx=raw.get("right_lane_change_dst_link_idx"),
            road_id=raw.get("road_id"),
            ego_lane=raw.get("ego_lane"),
        )

    if not links:
        raise ValueError("link_set.json에서 유효한 MGeo Link를 읽지 못했습니다.")

    return links


class LinkSpatialIndex:
    """MGeo Link point를 격자에 넣어 현재 Link 탐색 비용을 줄인다."""

    def __init__(
        self,
        links: Dict[str, MGeoLink],
        cell_size_m: float = 10.0,
        point_stride: int = 3,
    ) -> None:
        self.links = links
        self.cell_size_m = max(float(cell_size_m), 1.0)
        self.grid: Dict[Tuple[int, int], List[Tuple[str, PathPoint]]] = {}

        stride = max(int(point_stride), 1)
        for link_idx, link in links.items():
            for p in link.points[::stride]:
                key = self._key(p.x, p.y)
                self.grid.setdefault(key, []).append((link_idx, p))

    def _key(self, x: float, y: float) -> Tuple[int, int]:
        return (
            int(math.floor(x / self.cell_size_m)),
            int(math.floor(y / self.cell_size_m)),
        )

    def nearest_link(
        self,
        x: float,
        y: float,
        search_radius_cells: int = 2,
    ) -> Optional[MGeoLink]:
        """
        x,y에 가장 가까운 Link를 찾는다.

        Ego 좌표보다 Global Path의 nearest point를 넣는 것을 권장한다.
        그러면 반대편/인접 차선으로 잘못 매칭될 가능성이 줄어든다.
        """
        cx, cy = self._key(x, y)
        candidates: List[Tuple[str, PathPoint]] = []

        for radius in range(search_radius_cells + 1):
            candidates.clear()
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    candidates.extend(
                        self.grid.get((cx + dx, cy + dy), [])
                    )
            if candidates:
                break

        if not candidates:
            return None

        best_link_idx, _ = min(
            candidates,
            key=lambda item: (item[1].x - x) ** 2 + (item[1].y - y) ** 2,
        )
        return self.links.get(best_link_idx)


def available_adjacent_links(
    current_link: MGeoLink,
    links: Dict[str, MGeoLink],
) -> List[Tuple[str, MGeoLink]]:
    """현재 Link에서 실제 차선변경이 허용된 좌/우 인접 Link만 반환."""
    result: List[Tuple[str, MGeoLink]] = []

    if (
        current_link.can_move_left_lane
        and current_link.left_lane_change_dst_link_idx
        and current_link.left_lane_change_dst_link_idx in links
    ):
        result.append(
            ("left", links[current_link.left_lane_change_dst_link_idx])
        )

    if (
        current_link.can_move_right_lane
        and current_link.right_lane_change_dst_link_idx
        and current_link.right_lane_change_dst_link_idx in links
    ):
        result.append(
            ("right", links[current_link.right_lane_change_dst_link_idx])
        )

    return result


def _nearest_index(points: Sequence[PathPoint], x: float, y: float) -> int:
    return min(
        range(len(points)),
        key=lambda i: (points[i].x - x) ** 2 + (points[i].y - y) ** 2,
    )


def _orient_target_points(
    target_points: Sequence[PathPoint],
    reference_tangent: Tuple[float, float],
    near_index: int,
) -> Tuple[List[PathPoint], int]:
    """
    target Link point 순서가 Global Path 진행방향과 반대면 뒤집는다.
    """
    target = list(target_points)
    tx, ty = _path_tangent(target, near_index)
    rx, ry = reference_tangent

    if tx * rx + ty * ry >= 0.0:
        return target, near_index

    reversed_points = list(reversed(target))
    reversed_index = len(target) - 1 - near_index
    return reversed_points, reversed_index


def _hermite_point(
    p0: PathPoint,
    p1: PathPoint,
    t0: Tuple[float, float],
    t1: Tuple[float, float],
    u: float,
    tangent_scale: float,
) -> PathPoint:
    """Cubic Hermite interpolation."""
    u2 = u * u
    u3 = u2 * u

    h00 = 2.0 * u3 - 3.0 * u2 + 1.0
    h10 = u3 - 2.0 * u2 + u
    h01 = -2.0 * u3 + 3.0 * u2
    h11 = u3 - u2

    m0x = t0[0] * tangent_scale
    m0y = t0[1] * tangent_scale
    m1x = t1[0] * tangent_scale
    m1y = t1[1] * tangent_scale

    x = h00 * p0.x + h10 * m0x + h01 * p1.x + h11 * m1x
    y = h00 * p0.y + h10 * m0y + h01 * p1.y + h11 * m1y
    z = (1.0 - u) * p0.z + u * p1.z

    return PathPoint(x, y, z)


def _sample_hermite_transition(
    start: PathPoint,
    end: PathPoint,
    start_tangent: Tuple[float, float],
    end_tangent: Tuple[float, float],
    change_length_m: float,
    sample_spacing_m: float,
) -> List[PathPoint]:
    """
    현재 차선에서 목표 차선으로 부드럽게 이동하는 transition.
    시작/끝에서 각 차선의 진행방향과 접선이 맞도록 한다.
    """
    direct_distance = max(_distance(start, end), 1.0)
    tangent_scale = max(change_length_m, direct_distance)

    count = max(
        int(math.ceil(max(change_length_m, direct_distance) / sample_spacing_m)),
        8,
    )

    return [
        _hermite_point(
            start,
            end,
            start_tangent,
            end_tangent,
            i / float(count),
            tangent_scale,
        )
        for i in range(count + 1)
    ]


def _target_continuation(
    target_points: Sequence[PathPoint],
    start_index: int,
    distance_m: float,
) -> List[PathPoint]:
    """차선변경 완료 후 목표 차선을 따라 계속 진행."""
    if distance_m <= 0.0:
        return []

    result: List[PathPoint] = []
    accumulated = 0.0

    for i in range(start_index, len(target_points) - 1):
        p = target_points[i]
        q = target_points[i + 1]

        if not result:
            result.append(p)

        result.append(q)
        accumulated += _distance(p, q)

        if accumulated >= distance_m:
            break

    return result


def generate_lane_change_candidate(
    global_path: Sequence[PathPoint],
    ego_nearest_index: int,
    target_link: MGeoLink,
    side: str,
    start_distance_m: float,
    change_length_m: float,
    local_length_m: float,
    sample_spacing_m: float = 0.5,
) -> Optional[LaneChangeCandidate]:
    """
    Global Path를 따라 조금 진행한 뒤 실제 인접 Link 중심선으로
    부드럽게 합류하는 하나의 Candidate Path를 생성.
    """
    if len(global_path) < 2 or len(target_link.points) < 2:
        return None

    start_index = _index_at_distance(
        global_path,
        ego_nearest_index,
        start_distance_m,
    )
    end_reference_index = _index_at_distance(
        global_path,
        start_index,
        change_length_m,
    )

    start_point = global_path[start_index]
    end_reference = global_path[end_reference_index]

    start_tangent = _path_tangent(global_path, start_index)
    reference_end_tangent = _path_tangent(global_path, end_reference_index)

    # 목표 차선에서, 같은 longitudinal 위치에 해당하는 점을 찾는다.
    target_near_index = _nearest_index(
        target_link.points,
        end_reference.x,
        end_reference.y,
    )

    oriented_target_points, target_near_index = _orient_target_points(
        target_link.points,
        reference_end_tangent,
        target_near_index,
    )

    target_end = oriented_target_points[target_near_index]
    target_tangent = _path_tangent(
        oriented_target_points,
        target_near_index,
    )

    # 차량 위치부터 차선변경 시작점까지는 기존 Global Path를 유지한다.
    prefix = list(global_path[ego_nearest_index : start_index + 1])

    transition = _sample_hermite_transition(
        start_point,
        target_end,
        start_tangent,
        target_tangent,
        change_length_m,
        sample_spacing_m,
    )

    remaining_length = max(
        local_length_m - start_distance_m - change_length_m,
        0.0,
    )
    continuation = _target_continuation(
        oriented_target_points,
        target_near_index,
        remaining_length,
    )

    path: List[PathPoint] = []
    path.extend(prefix)

    # 중복되는 시작점 제거
    if transition:
        path.extend(transition[1:] if path else transition)

    # transition 끝점과 target continuation 시작점 중복 제거
    if continuation:
        path.extend(continuation[1:] if path else continuation)

    if len(path) < 2:
        return None

    return LaneChangeCandidate(
        side=side,
        target_link_idx=target_link.idx,
        start_distance_m=float(start_distance_m),
        change_length_m=float(change_length_m),
        path=path,
    )


def generate_lane_change_candidates(
    global_path: Sequence[PathPoint],
    ego_x: float,
    ego_y: float,
    current_link: MGeoLink,
    links: Dict[str, MGeoLink],
    start_distances_m: Sequence[float],
    change_lengths_m: Sequence[float],
    local_length_m: float,
    sample_spacing_m: float = 0.5,
) -> List[LaneChangeCandidate]:
    """
    현재 Link에서 실제로 이동 가능한 인접 차선만 대상으로
    여러 차선변경 타이밍/길이 후보를 생성한다.
    """
    ego_nearest_index = nearest_path_index(global_path, ego_x, ego_y)
    adjacent = available_adjacent_links(current_link, links)

    candidates: List[LaneChangeCandidate] = []

    for side, target_link in adjacent:
        for start_distance in start_distances_m:
            for change_length in change_lengths_m:
                if start_distance < 0.0 or change_length <= 0.0:
                    continue

                # Local planning 구간을 넘어서는 후보는 만들지 않는다.
                if start_distance + change_length > local_length_m:
                    continue

                candidate = generate_lane_change_candidate(
                    global_path=global_path,
                    ego_nearest_index=ego_nearest_index,
                    target_link=target_link,
                    side=side,
                    start_distance_m=float(start_distance),
                    change_length_m=float(change_length),
                    local_length_m=float(local_length_m),
                    sample_spacing_m=float(sample_spacing_m),
                )

                if candidate is not None:
                    candidates.append(candidate)

    return candidates
