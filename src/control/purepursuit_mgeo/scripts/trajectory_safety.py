#!/usr/bin/env python3
"""2-D candidate safety evaluation for the Frenet avoidance planner.

Stage F6 keeps the strict full-margin OBB check, but adds one narrowly-scoped
escape rule for a vehicle that is already inside the *inflated* safety envelope
when replanning begins:

* the physical (zero-margin) ego OBB is NEVER allowed to overlap an obstacle;
* escape mode is considered only when point 0 is clear physically but overlaps
  with the configured inflated footprint;
* for that obstacle only, margins ramp from zero to full over a short prefix;
* full margins are mandatory after the prefix.

This prevents a safe outward same-side replan from being rejected solely because
its first point starts inside the conservative margin that the previous maneuver
already entered.  It does not waive physical collision checks.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence


@dataclass
class ObstacleBox:
    obstacle_id: int
    center_x: float
    center_y: float
    yaw: float
    length: float
    width: float


@dataclass
class CandidateEvaluation:
    candidate_index: int
    valid: bool
    reason: str
    collision_obstacle_id: Optional[int]
    first_collision_path_index: Optional[int]
    max_curvature_1pm: float
    max_steering_rad: float
    max_lateral_accel_mps2: float
    cost: float
    escape_prefix_used: bool = False
    escape_obstacle_id: Optional[int] = None


def _normalize(vx: float, vy: float):
    n = math.hypot(vx, vy)
    if n < 1.0e-9:
        return 1.0, 0.0
    return vx / n, vy / n


def _path_tangent(path, i: int):
    if len(path) < 2:
        return 1.0, 0.0
    if i <= 0:
        a, b = path[0], path[1]
    elif i >= len(path) - 1:
        a, b = path[-2], path[-1]
    else:
        a, b = path[i - 1], path[i + 1]
    return _normalize(float(b.x) - float(a.x), float(b.y) - float(a.y))


def _obb_overlap(
    ax: float,
    ay: float,
    ayaw: float,
    alen: float,
    awid: float,
    bx: float,
    by: float,
    byaw: float,
    blen: float,
    bwid: float,
) -> bool:
    """SAT overlap test for two planar oriented rectangles."""
    a_u = (math.cos(ayaw), math.sin(ayaw))
    a_v = (-a_u[1], a_u[0])
    b_u = (math.cos(byaw), math.sin(byaw))
    b_v = (-b_u[1], b_u[0])
    dx, dy = bx - ax, by - ay
    ahl, ahw = 0.5 * alen, 0.5 * awid
    bhl, bhw = 0.5 * blen, 0.5 * bwid

    for ux, uy in (a_u, a_v, b_u, b_v):
        center_sep = abs(dx * ux + dy * uy)
        ra = ahl * abs(a_u[0] * ux + a_u[1] * uy) + ahw * abs(a_v[0] * ux + a_v[1] * uy)
        rb = bhl * abs(b_u[0] * ux + b_u[1] * uy) + bhw * abs(b_v[0] * ux + b_v[1] * uy)
        if center_sep > ra + rb:
            return False
    return True


def _max_curvature(path) -> float:
    if len(path) < 3:
        return 0.0
    maximum = 0.0
    for i in range(1, len(path) - 1):
        p0, p1, p2 = path[i - 1], path[i], path[i + 1]
        ax, ay = float(p1.x) - float(p0.x), float(p1.y) - float(p0.y)
        bx, by = float(p2.x) - float(p1.x), float(p2.y) - float(p1.y)
        cx, cy = float(p2.x) - float(p0.x), float(p2.y) - float(p0.y)
        a = math.hypot(ax, ay)
        b = math.hypot(bx, by)
        c = math.hypot(cx, cy)
        denom = a * b * c
        if denom < 1.0e-9:
            continue
        cross = ax * by - ay * bx
        kappa = abs(2.0 * cross / denom)
        maximum = max(maximum, kappa)
    return maximum


def _path_lengths(path):
    lengths = [0.0]
    for i in range(1, len(path)):
        lengths.append(
            lengths[-1]
            + math.hypot(
                float(path[i].x) - float(path[i - 1].x),
                float(path[i].y) - float(path[i - 1].y),
            )
        )
    return lengths


def _ego_pose_on_path(path, i: int, vehicle_center_from_base_m: float):
    p = path[i]
    tx, ty = _path_tangent(path, i)
    yaw = math.atan2(ty, tx)
    center_x = float(p.x) + float(vehicle_center_from_base_m) * tx
    center_y = float(p.y) + float(vehicle_center_from_base_m) * ty
    return center_x, center_y, yaw


def _overlap_with_margins(
    center_x: float,
    center_y: float,
    yaw: float,
    obs: ObstacleBox,
    vehicle_length_m: float,
    vehicle_width_m: float,
    longitudinal_margin_m: float,
    lateral_margin_m: float,
) -> bool:
    ego_length = max(0.10, float(vehicle_length_m) + 2.0 * max(0.0, longitudinal_margin_m))
    ego_width = max(0.10, float(vehicle_width_m) + 2.0 * max(0.0, lateral_margin_m))

    dx = obs.center_x - center_x
    dy = obs.center_y - center_y
    broad = (
        0.5 * math.hypot(ego_length, ego_width)
        + 0.5 * math.hypot(obs.length, obs.width)
        + 0.25
    )
    if dx * dx + dy * dy > broad * broad:
        return False
    return _obb_overlap(
        center_x,
        center_y,
        yaw,
        ego_length,
        ego_width,
        obs.center_x,
        obs.center_y,
        obs.yaw,
        max(0.10, obs.length),
        max(0.10, obs.width),
    )


def _first_collision_with_escape(
    path,
    obstacles: Sequence[ObstacleBox],
    vehicle_length_m: float,
    vehicle_width_m: float,
    vehicle_center_from_base_m: float,
    collision_longitudinal_margin_m: float,
    collision_lateral_margin_m: float,
    collision_sample_stride: int = 1,
    escape_prefix_m: float = 0.0,
):
    """Return (id, index, escape_used, escape_obstacle_id).

    Escape mode is activated independently for each obstacle only when the
    physical footprint is clear at path index 0 but the *full-margin* footprint
    overlaps there.  For that obstacle, safety margins ramp linearly from 0 to
    full over ``escape_prefix_m``.  Physical overlap is always checked first at
    every sample, so this cannot authorize actual contact.
    """
    if len(path) < 2:
        return None, None, False, None

    stride = max(1, int(collision_sample_stride))
    prefix = max(0.0, float(escape_prefix_m))
    lengths = _path_lengths(path)

    # Determine which obstacle, if any, is eligible for start-overlap escape.
    escape_ids = set()
    if prefix > 1.0e-6:
        cx0, cy0, yaw0 = _ego_pose_on_path(path, 0, vehicle_center_from_base_m)
        for obs in obstacles:
            physical_overlap = _overlap_with_margins(
                cx0,
                cy0,
                yaw0,
                obs,
                vehicle_length_m,
                vehicle_width_m,
                0.0,
                0.0,
            )
            if physical_overlap:
                # Actual body overlap is never an escape case.
                continue
            full_overlap = _overlap_with_margins(
                cx0,
                cy0,
                yaw0,
                obs,
                vehicle_length_m,
                vehicle_width_m,
                collision_longitudinal_margin_m,
                collision_lateral_margin_m,
            )
            if full_overlap:
                escape_ids.add(obs.obstacle_id)

    escape_used = bool(escape_ids)
    escape_obstacle_id = min(escape_ids) if escape_ids else None

    for i in range(0, len(path), stride):
        center_x, center_y, yaw = _ego_pose_on_path(path, i, vehicle_center_from_base_m)
        distance_m = lengths[i]

        for obs in obstacles:
            # First: physical footprint must never overlap, including inside the prefix.
            if _overlap_with_margins(
                center_x,
                center_y,
                yaw,
                obs,
                vehicle_length_m,
                vehicle_width_m,
                0.0,
                0.0,
            ):
                return obs.obstacle_id, i, escape_used, escape_obstacle_id

            if obs.obstacle_id in escape_ids and distance_m < prefix:
                u = max(0.0, min(1.0, distance_m / max(prefix, 1.0e-6)))
                # Quadratic ramp: keep the first metre close to the physical
                # footprint, then recover the full safety envelope by prefix end.
                scale = u * u
                long_margin = collision_longitudinal_margin_m * scale
                lat_margin = collision_lateral_margin_m * scale
            else:
                long_margin = collision_longitudinal_margin_m
                lat_margin = collision_lateral_margin_m

            if _overlap_with_margins(
                center_x,
                center_y,
                yaw,
                obs,
                vehicle_length_m,
                vehicle_width_m,
                long_margin,
                lat_margin,
            ):
                return obs.obstacle_id, i, escape_used, escape_obstacle_id

    return None, None, escape_used, escape_obstacle_id


def check_path_collision(
    path,
    obstacles: Sequence[ObstacleBox],
    vehicle_length_m: float,
    vehicle_width_m: float,
    vehicle_center_from_base_m: float,
    collision_longitudinal_margin_m: float,
    collision_lateral_margin_m: float,
    collision_sample_stride: int = 1,
    escape_prefix_m: float = 0.0,
):
    """Return (collision_obstacle_id, path_index) for the first OBB overlap."""
    collision_id, collision_index, _used, _obs_id = _first_collision_with_escape(
        path=path,
        obstacles=obstacles,
        vehicle_length_m=vehicle_length_m,
        vehicle_width_m=vehicle_width_m,
        vehicle_center_from_base_m=vehicle_center_from_base_m,
        collision_longitudinal_margin_m=collision_longitudinal_margin_m,
        collision_lateral_margin_m=collision_lateral_margin_m,
        collision_sample_stride=collision_sample_stride,
        escape_prefix_m=escape_prefix_m,
    )
    return collision_id, collision_index


def evaluate_candidate(
    candidate_index: int,
    candidate,
    obstacles: Sequence[ObstacleBox],
    vehicle_length_m: float,
    vehicle_width_m: float,
    vehicle_center_from_base_m: float,
    collision_longitudinal_margin_m: float,
    collision_lateral_margin_m: float,
    wheelbase_m: float,
    max_steering_rad: float,
    evaluation_speed_mps: float,
    max_lateral_accel_mps2: float,
    collision_sample_stride: int = 1,
    escape_prefix_m: float = 0.0,
) -> CandidateEvaluation:
    path = candidate.path
    if len(path) < 3:
        return CandidateEvaluation(candidate_index, False, "path_too_short", None, None, 0.0, 0.0, 0.0, float("inf"))

    collision_id, collision_index, escape_used, escape_obstacle_id = _first_collision_with_escape(
        path=path,
        obstacles=obstacles,
        vehicle_length_m=vehicle_length_m,
        vehicle_width_m=vehicle_width_m,
        vehicle_center_from_base_m=vehicle_center_from_base_m,
        collision_longitudinal_margin_m=collision_longitudinal_margin_m,
        collision_lateral_margin_m=collision_lateral_margin_m,
        collision_sample_stride=collision_sample_stride,
        escape_prefix_m=escape_prefix_m,
    )

    kappa = _max_curvature(path)
    steering = math.atan(max(0.0, wheelbase_m) * kappa)
    lateral_accel = max(0.0, evaluation_speed_mps) ** 2 * kappa

    if collision_id is not None:
        return CandidateEvaluation(
            candidate_index,
            False,
            "collision",
            collision_id,
            collision_index,
            kappa,
            steering,
            lateral_accel,
            float("inf"),
            escape_prefix_used=escape_used,
            escape_obstacle_id=escape_obstacle_id,
        )
    if steering > max_steering_rad + 1.0e-9:
        return CandidateEvaluation(
            candidate_index,
            False,
            "steering_limit",
            None,
            None,
            kappa,
            steering,
            lateral_accel,
            float("inf"),
            escape_prefix_used=escape_used,
            escape_obstacle_id=escape_obstacle_id,
        )
    if lateral_accel > max_lateral_accel_mps2 + 1.0e-9:
        return CandidateEvaluation(
            candidate_index,
            False,
            "lateral_accel_limit",
            None,
            None,
            kappa,
            steering,
            lateral_accel,
            float("inf"),
            escape_prefix_used=escape_used,
            escape_obstacle_id=escape_obstacle_id,
        )

    target_d = abs(float(getattr(candidate, "target_d_m", 0.0)))
    cost = 20.0 * kappa + 0.8 * target_d
    if getattr(candidate, "kind", "") == "lane_change":
        change_length = max(1.0, float(getattr(candidate, "change_length_m", 1.0)))
        cost += 2.0 / change_length
    else:
        cost += 0.25 * float(getattr(candidate, "extra_clearance_m", 0.0))

    return CandidateEvaluation(
        candidate_index,
        True,
        "ok_escape_prefix" if escape_used else "ok",
        None,
        None,
        kappa,
        steering,
        lateral_accel,
        cost,
        escape_prefix_used=escape_used,
        escape_obstacle_id=escape_obstacle_id,
    )
