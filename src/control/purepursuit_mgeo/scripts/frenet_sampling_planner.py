#!/usr/bin/env python3
"""Frenet candidate generation for MORAI/MGeo avoidance debug stages.

This module keeps two candidate families separate:

1) MGeo lane-change candidates
   - generated only toward a valid left/right adjacent MGeo link.

2) Single-lane/free-space bypass candidates
   - generated around one blocking obstacle when there is no adjacent lane.
   - the lateral target is derived from obstacle width + ego width + margin.
   - this stage does NOT yet prove that the bypass stays inside a road polygon.

No collision rejection, cost function, path switching, or /ctrl_cmd lives here.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from purepursuit_mgeo.path import PathPoint
from frenet_path import ReferencePath


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
    width_start_m: Optional[float]
    width_end_m: Optional[float]


@dataclass
class FrenetLaneChangeCandidate:
    kind: str
    side: str
    target_link_idx: str
    start_distance_m: float
    change_length_m: float
    start_s: float
    end_s: float
    target_d_m: float
    path: List[PathPoint]


@dataclass
class FrenetBypassCandidate:
    kind: str
    side: str
    obstacle_id: int
    target_d_m: float
    departure_start_s: float
    hold_start_s: float
    hold_end_s: float
    return_end_s: float
    extra_clearance_m: float
    path: List[PathPoint]


def _distance(a: PathPoint, b: PathPoint) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def _optional_float(value) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _raw_links_iter(raw_data):
    if isinstance(raw_data, list):
        return raw_data
    if isinstance(raw_data, dict):
        if isinstance(raw_data.get("links"), list):
            return raw_data["links"]
        return list(raw_data.values())
    raise ValueError("Unsupported link_set.json structure")


def load_mgeo_links(link_set_file: str) -> Dict[str, MGeoLink]:
    with open(link_set_file, "r", encoding="utf-8") as stream:
        raw_data = json.load(stream)

    links: Dict[str, MGeoLink] = {}
    for raw in _raw_links_iter(raw_data):
        if not isinstance(raw, dict):
            continue
        raw_points = raw.get("points") or []
        if len(raw_points) < 2 or "idx" not in raw:
            continue
        points = [
            PathPoint(
                float(p[0]),
                float(p[1]),
                float(p[2]) if len(p) >= 3 else 0.0,
            )
            for p in raw_points
        ]
        idx = str(raw["idx"])
        links[idx] = MGeoLink(
            idx=idx,
            points=points,
            can_move_left_lane=bool(raw.get("can_move_left_lane", False)),
            can_move_right_lane=bool(raw.get("can_move_right_lane", False)),
            left_lane_change_dst_link_idx=(
                str(raw["left_lane_change_dst_link_idx"])
                if raw.get("left_lane_change_dst_link_idx") is not None
                else None
            ),
            right_lane_change_dst_link_idx=(
                str(raw["right_lane_change_dst_link_idx"])
                if raw.get("right_lane_change_dst_link_idx") is not None
                else None
            ),
            road_id=(str(raw["road_id"]) if raw.get("road_id") is not None else None),
            ego_lane=(int(raw["ego_lane"]) if raw.get("ego_lane") is not None else None),
            width_start_m=_optional_float(raw.get("width_start")),
            width_end_m=_optional_float(raw.get("width_end")),
        )

    if not links:
        raise ValueError("No valid links were loaded from link_set.json")
    return links


class LinkSpatialIndex:
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
                self.grid.setdefault(self._key(p.x, p.y), []).append((link_idx, p))

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
        cx, cy = self._key(x, y)
        candidates: List[Tuple[str, PathPoint]] = []
        for radius in range(search_radius_cells + 1):
            candidates = []
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    candidates.extend(self.grid.get((cx + dx, cy + dy), []))
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
    result: List[Tuple[str, MGeoLink]] = []
    if (
        current_link.can_move_left_lane
        and current_link.left_lane_change_dst_link_idx
        and current_link.left_lane_change_dst_link_idx in links
    ):
        result.append(("left", links[current_link.left_lane_change_dst_link_idx]))

    if (
        current_link.can_move_right_lane
        and current_link.right_lane_change_dst_link_idx
        and current_link.right_lane_change_dst_link_idx in links
    ):
        result.append(("right", links[current_link.right_lane_change_dst_link_idx]))
    return result


def _quintic_smoothstep(u: float) -> float:
    u = max(0.0, min(1.0, float(u)))
    return 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5


class TargetLaneProfile:
    """Adjacent MGeo link represented as d(s) relative to the global route."""

    def __init__(
        self,
        reference: ReferencePath,
        link: MGeoLink,
        max_projection_distance_m: float = 8.0,
        point_stride: int = 1,
    ) -> None:
        samples: List[Tuple[float, float]] = []
        stride = max(int(point_stride), 1)
        for p in link.points[::stride]:
            projection = reference.project(p.x, p.y)
            if projection.distance_m <= max_projection_distance_m:
                samples.append((projection.s, projection.d))

        samples.sort(key=lambda item: item[0])
        deduped: List[Tuple[float, float]] = []
        for s, d in samples:
            if deduped and abs(s - deduped[-1][0]) < 0.05:
                prev_s, prev_d = deduped[-1]
                deduped[-1] = (0.5 * (prev_s + s), 0.5 * (prev_d + d))
            else:
                deduped.append((s, d))

        self.samples = deduped
        self.link_idx = link.idx

    @property
    def valid(self) -> bool:
        return len(self.samples) >= 2

    @property
    def min_s(self) -> float:
        return self.samples[0][0] if self.samples else 0.0

    @property
    def max_s(self) -> float:
        return self.samples[-1][0] if self.samples else 0.0

    def d_at(self, s: float, max_extrapolation_m: float = 5.0) -> Optional[float]:
        if not self.valid:
            return None
        if s < self.min_s:
            return self.samples[0][1] if self.min_s - s <= max_extrapolation_m else None
        if s > self.max_s:
            return self.samples[-1][1] if s - self.max_s <= max_extrapolation_m else None

        for i in range(len(self.samples) - 1):
            s0, d0 = self.samples[i]
            s1, d1 = self.samples[i + 1]
            if s0 <= s <= s1:
                if s1 - s0 < 1.0e-9:
                    return d0
                ratio = (s - s0) / (s1 - s0)
                return d0 + ratio * (d1 - d0)
        return self.samples[-1][1]


def _append_unique(path: List[PathPoint], point: PathPoint, min_distance_m: float = 0.05) -> None:
    if path and _distance(path[-1], point) < min_distance_m:
        return
    path.append(point)


def generate_frenet_lane_change_candidate(
    reference: ReferencePath,
    ego_s: float,
    target_link: MGeoLink,
    side: str,
    start_distance_m: float,
    change_length_m: float,
    local_length_m: float,
    sample_spacing_m: float = 0.5,
    target_profile_projection_limit_m: float = 8.0,
) -> Optional[FrenetLaneChangeCandidate]:
    if start_distance_m < 0.0 or change_length_m <= 0.0:
        return None
    if start_distance_m + change_length_m > local_length_m:
        return None

    profile = TargetLaneProfile(
        reference,
        target_link,
        max_projection_distance_m=target_profile_projection_limit_m,
    )
    if not profile.valid:
        return None

    start_s = ego_s + start_distance_m
    end_s = start_s + change_length_m
    local_end_s = min(ego_s + local_length_m, reference.total_length_m)

    target_d_end = profile.d_at(end_s)
    if target_d_end is None:
        return None

    if side == "left" and target_d_end < 0.25:
        return None
    if side == "right" and target_d_end > -0.25:
        return None

    spacing = max(float(sample_spacing_m), 0.10)
    path: List[PathPoint] = []
    s = ego_s
    while s <= local_end_s + 1.0e-6:
        if s < start_s:
            d = 0.0
        elif s <= end_s:
            u = (s - start_s) / max(change_length_m, 1.0e-6)
            d = target_d_end * _quintic_smoothstep(u)
        else:
            target_d = profile.d_at(s)
            if target_d is None:
                break
            d = target_d

        _append_unique(path, reference.frenet_to_map(s, d))
        s += spacing

    if end_s <= local_end_s:
        _append_unique(path, reference.frenet_to_map(end_s, target_d_end))

    if len(path) < 2:
        return None

    return FrenetLaneChangeCandidate(
        kind="lane_change",
        side=side,
        target_link_idx=target_link.idx,
        start_distance_m=float(start_distance_m),
        change_length_m=float(change_length_m),
        start_s=float(start_s),
        end_s=float(end_s),
        target_d_m=float(target_d_end),
        path=path,
    )


def generate_frenet_lane_change_candidates(
    reference: ReferencePath,
    ego_s: float,
    current_link: MGeoLink,
    links: Dict[str, MGeoLink],
    start_distances_m: Sequence[float],
    change_lengths_m: Sequence[float],
    local_length_m: float,
    sample_spacing_m: float = 0.5,
) -> List[FrenetLaneChangeCandidate]:
    candidates: List[FrenetLaneChangeCandidate] = []
    for side, target_link in available_adjacent_links(current_link, links):
        for start_distance_m in start_distances_m:
            for change_length_m in change_lengths_m:
                candidate = generate_frenet_lane_change_candidate(
                    reference=reference,
                    ego_s=ego_s,
                    target_link=target_link,
                    side=side,
                    start_distance_m=float(start_distance_m),
                    change_length_m=float(change_length_m),
                    local_length_m=float(local_length_m),
                    sample_spacing_m=float(sample_spacing_m),
                )
                if candidate is not None:
                    candidates.append(candidate)
    return candidates


def generate_frenet_bypass_candidates(
    reference: ReferencePath,
    ego_s: float,
    obstacle_id: int,
    obstacle_s: float,
    obstacle_d: float,
    obstacle_longitudinal_half_m: float,
    obstacle_lateral_half_m: float,
    vehicle_width_m: float,
    lateral_margin_m: float,
    longitudinal_margin_m: float,
    departure_length_m: float,
    return_length_m: float,
    extra_clearances_m: Sequence[float],
    max_abs_d_m: float,
    local_length_m: float,
    sample_spacing_m: float = 0.5,
    min_transition_length_m: float = 4.0,
) -> List[FrenetBypassCandidate]:
    """Generate left/right departure-hold-return candidates around one obstacle.

    The target offset is derived from the minimum lateral separation required
    between the obstacle envelope and the ego envelope.  ``max_abs_d_m`` is
    only a debug guard; it is NOT a road-boundary proof.
    """

    local_end_s = min(ego_s + float(local_length_m), reference.total_length_m)
    obstacle_front_s = obstacle_s - max(0.0, obstacle_longitudinal_half_m)
    obstacle_rear_s = obstacle_s + max(0.0, obstacle_longitudinal_half_m)

    hold_start_s = obstacle_front_s - max(0.0, longitudinal_margin_m)
    hold_end_s = obstacle_rear_s + max(0.0, longitudinal_margin_m)

    departure_start_s = max(ego_s, hold_start_s - max(0.0, departure_length_m))
    departure_actual_m = hold_start_s - departure_start_s
    return_end_s = hold_end_s + max(0.0, return_length_m)

    if departure_actual_m < max(0.5, min_transition_length_m):
        return []
    if hold_end_s <= ego_s:
        return []
    if return_end_s > local_end_s:
        return []

    required_center_separation = (
        max(0.0, obstacle_lateral_half_m)
        + 0.5 * max(0.0, vehicle_width_m)
        + max(0.0, lateral_margin_m)
    )

    spacing = max(float(sample_spacing_m), 0.10)
    candidates: List[FrenetBypassCandidate] = []

    for side in ("left", "right"):
        sign = 1.0 if side == "left" else -1.0
        base_target_d = obstacle_d + sign * required_center_separation

        for extra in extra_clearances_m:
            extra = max(0.0, float(extra))
            target_d = base_target_d + sign * extra
            if abs(target_d) > max(0.1, float(max_abs_d_m)):
                continue

            path: List[PathPoint] = []
            s = ego_s
            while s <= local_end_s + 1.0e-6:
                if s < departure_start_s:
                    d = 0.0
                elif s <= hold_start_s:
                    u = (s - departure_start_s) / max(departure_actual_m, 1.0e-6)
                    d = target_d * _quintic_smoothstep(u)
                elif s <= hold_end_s:
                    d = target_d
                elif s <= return_end_s:
                    u = (s - hold_end_s) / max(return_end_s - hold_end_s, 1.0e-6)
                    d = target_d * (1.0 - _quintic_smoothstep(u))
                else:
                    d = 0.0

                _append_unique(path, reference.frenet_to_map(s, d))
                s += spacing

            # Add exact phase boundaries for clearer RViz inspection.
            for exact_s, exact_d in (
                (departure_start_s, 0.0),
                (hold_start_s, target_d),
                (hold_end_s, target_d),
                (return_end_s, 0.0),
            ):
                if ego_s <= exact_s <= local_end_s:
                    _append_unique(path, reference.frenet_to_map(exact_s, exact_d))

            path.sort(key=lambda p: reference.project(p.x, p.y).s)
            if len(path) < 2:
                continue

            candidates.append(
                FrenetBypassCandidate(
                    kind="bypass",
                    side=side,
                    obstacle_id=int(obstacle_id),
                    target_d_m=float(target_d),
                    departure_start_s=float(departure_start_s),
                    hold_start_s=float(hold_start_s),
                    hold_end_s=float(hold_end_s),
                    return_end_s=float(return_end_s),
                    extra_clearance_m=float(extra),
                    path=path,
                )
            )

    return candidates
