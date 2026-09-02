#!/usr/bin/env python3
"""2-D candidate safety evaluation for the Frenet avoidance debug planner.

This module is intentionally independent of ROS messages except for the candidate
object shape (candidate.path must contain x/y points).  It checks:
  * ego OBB vs every tracked obstacle OBB along the whole candidate,
  * geometric curvature / steering feasibility,
  * lateral acceleration at a configurable evaluation speed,
  * a small deterministic cost used only to choose a debug 'best' path.

It does NOT prove road/lane-boundary legality and does NOT predict dynamic obstacles.
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
        # Twice signed triangle area divided by abc gives curvature magnitude.
        cross = ax * by - ay * bx
        kappa = abs(2.0 * cross / denom)
        maximum = max(maximum, kappa)
    return maximum



def check_path_collision(
    path,
    obstacles: Sequence[ObstacleBox],
    vehicle_length_m: float,
    vehicle_width_m: float,
    vehicle_center_from_base_m: float,
    collision_longitudinal_margin_m: float,
    collision_lateral_margin_m: float,
    collision_sample_stride: int = 1,
):
    """Return (collision_obstacle_id, path_index) for the first OBB overlap.

    This is used by the path manager to guard the *remaining committed path*
    during execution.  (None, None) means no current static OBB overlap.
    """
    if len(path) < 2:
        return None, None

    ego_length = max(
        0.10,
        float(vehicle_length_m)
        + 2.0 * max(0.0, collision_longitudinal_margin_m),
    )
    ego_width = max(
        0.10,
        float(vehicle_width_m)
        + 2.0 * max(0.0, collision_lateral_margin_m),
    )
    stride = max(1, int(collision_sample_stride))

    for i in range(0, len(path), stride):
        p = path[i]
        tx, ty = _path_tangent(path, i)
        yaw = math.atan2(ty, tx)
        center_x = float(p.x) + float(vehicle_center_from_base_m) * tx
        center_y = float(p.y) + float(vehicle_center_from_base_m) * ty

        for obs in obstacles:
            dx = obs.center_x - center_x
            dy = obs.center_y - center_y
            broad = (
                0.5 * math.hypot(ego_length, ego_width)
                + 0.5 * math.hypot(obs.length, obs.width)
                + 0.25
            )
            if dx * dx + dy * dy > broad * broad:
                continue
            if _obb_overlap(
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
            ):
                return obs.obstacle_id, i

    return None, None

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
) -> CandidateEvaluation:
    path = candidate.path
    if len(path) < 3:
        return CandidateEvaluation(candidate_index, False, "path_too_short", None, None, 0.0, 0.0, 0.0, float("inf"))

    # Inflate the ego footprint rather than the obstacle.  This is equivalent
    # to a simple symmetric safety margin for static OBB collision checking.
    ego_length = max(0.10, float(vehicle_length_m) + 2.0 * max(0.0, collision_longitudinal_margin_m))
    ego_width = max(0.10, float(vehicle_width_m) + 2.0 * max(0.0, collision_lateral_margin_m))
    stride = max(1, int(collision_sample_stride))

    collision_id = None
    collision_index = None
    for i in range(0, len(path), stride):
        p = path[i]
        tx, ty = _path_tangent(path, i)
        yaw = math.atan2(ty, tx)
        center_x = float(p.x) + float(vehicle_center_from_base_m) * tx
        center_y = float(p.y) + float(vehicle_center_from_base_m) * ty

        for obs in obstacles:
            # Cheap broad phase before SAT.
            dx = obs.center_x - center_x
            dy = obs.center_y - center_y
            broad = 0.5 * math.hypot(ego_length, ego_width) + 0.5 * math.hypot(obs.length, obs.width) + 0.25
            if dx * dx + dy * dy > broad * broad:
                continue
            if _obb_overlap(
                center_x, center_y, yaw, ego_length, ego_width,
                obs.center_x, obs.center_y, obs.yaw,
                max(0.10, obs.length), max(0.10, obs.width),
            ):
                collision_id = obs.obstacle_id
                collision_index = i
                break
        if collision_id is not None:
            break

    kappa = _max_curvature(path)
    steering = math.atan(max(0.0, wheelbase_m) * kappa)
    lateral_accel = max(0.0, evaluation_speed_mps) ** 2 * kappa

    if collision_id is not None:
        return CandidateEvaluation(candidate_index, False, "collision", collision_id, collision_index, kappa, steering, lateral_accel, float("inf"))
    if steering > max_steering_rad + 1.0e-9:
        return CandidateEvaluation(candidate_index, False, "steering_limit", None, None, kappa, steering, lateral_accel, float("inf"))
    if lateral_accel > max_lateral_accel_mps2 + 1.0e-9:
        return CandidateEvaluation(candidate_index, False, "lateral_accel_limit", None, None, kappa, steering, lateral_accel, float("inf"))

    target_d = abs(float(getattr(candidate, "target_d_m", 0.0)))
    # Smoothness dominates; then prefer the smallest lateral excursion.
    cost = 20.0 * kappa + 0.8 * target_d
    if getattr(candidate, "kind", "") == "lane_change":
        # Mildly prefer longer/softer lane changes when otherwise similar.
        change_length = max(1.0, float(getattr(candidate, "change_length_m", 1.0)))
        cost += 2.0 / change_length
    else:
        cost += 0.25 * float(getattr(candidate, "extra_clearance_m", 0.0))

    return CandidateEvaluation(candidate_index, True, "ok", None, None, kappa, steering, lateral_accel, cost)
