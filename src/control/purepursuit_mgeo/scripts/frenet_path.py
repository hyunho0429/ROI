#!/usr/bin/env python3
"""Polyline-based Frenet utilities for the MORAI/MGeo map frame.

Convention
----------
* s: accumulated distance along the global reference path [m]
* d: signed lateral distance from the reference path [m]
* d > 0: left side of the reference path direction
* d < 0: right side of the reference path direction

This module is intentionally independent from ROS messages.  It uses the
existing ``purepursuit_mgeo.path.PathPoint`` type so the output can be passed
back to the current map-frame path code without changing Pure Pursuit.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from purepursuit_mgeo.path import PathPoint


@dataclass(frozen=True)
class FrenetProjection:
    s: float
    d: float
    reference_x: float
    reference_y: float
    reference_z: float
    tangent_x: float
    tangent_y: float
    segment_index: int
    segment_ratio: float
    distance_m: float


class ReferencePath:
    """Global polyline + fast map<->Frenet conversion."""

    def __init__(
        self,
        points: Sequence[PathPoint],
        grid_cell_size_m: float = 10.0,
        grid_point_stride: int = 1,
    ) -> None:
        if len(points) < 2:
            raise ValueError("ReferencePath requires at least two points")

        self.points: List[PathPoint] = list(points)
        self.cumulative_s: List[float] = [0.0]
        for i in range(len(self.points) - 1):
            p = self.points[i]
            q = self.points[i + 1]
            self.cumulative_s.append(
                self.cumulative_s[-1] + math.hypot(q.x - p.x, q.y - p.y)
            )

        self.total_length_m = self.cumulative_s[-1]
        self.grid_cell_size_m = max(float(grid_cell_size_m), 1.0)
        self.grid: Dict[Tuple[int, int], List[int]] = {}
        stride = max(int(grid_point_stride), 1)
        for index in range(0, len(self.points), stride):
            p = self.points[index]
            self.grid.setdefault(self._grid_key(p.x, p.y), []).append(index)

    def _grid_key(self, x: float, y: float) -> Tuple[int, int]:
        return (
            int(math.floor(x / self.grid_cell_size_m)),
            int(math.floor(y / self.grid_cell_size_m)),
        )

    def nearest_point_index(
        self,
        x: float,
        y: float,
        max_search_radius_cells: int = 4,
    ) -> int:
        cx, cy = self._grid_key(x, y)
        candidate_indices: List[int] = []

        for radius in range(max_search_radius_cells + 1):
            candidate_indices = []
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    candidate_indices.extend(self.grid.get((cx + dx, cy + dy), []))
            if candidate_indices:
                break

        if not candidate_indices:
            candidate_indices = list(range(len(self.points)))

        return min(
            candidate_indices,
            key=lambda i: (self.points[i].x - x) ** 2 + (self.points[i].y - y) ** 2,
        )

    def project(
        self,
        x: float,
        y: float,
        segment_search_half_window: int = 6,
    ) -> FrenetProjection:
        """Project a map-frame point to the closest nearby reference segment."""
        nearest_point = self.nearest_point_index(x, y)
        first_segment = max(0, nearest_point - segment_search_half_window)
        last_segment = min(
            len(self.points) - 2,
            nearest_point + segment_search_half_window,
        )

        best: Optional[FrenetProjection] = None
        best_distance2 = float("inf")

        for i in range(first_segment, last_segment + 1):
            p = self.points[i]
            q = self.points[i + 1]
            vx = q.x - p.x
            vy = q.y - p.y
            length2 = vx * vx + vy * vy
            if length2 < 1.0e-12:
                continue

            ratio = ((x - p.x) * vx + (y - p.y) * vy) / length2
            ratio = max(0.0, min(1.0, ratio))
            ref_x = p.x + ratio * vx
            ref_y = p.y + ratio * vy
            ref_z = p.z + ratio * (q.z - p.z)
            dx = x - ref_x
            dy = y - ref_y
            distance2 = dx * dx + dy * dy

            if distance2 >= best_distance2:
                continue

            segment_length = math.sqrt(length2)
            tx = vx / segment_length
            ty = vy / segment_length
            # cross(tangent, point-reference): positive is left of travel direction.
            signed_d = tx * dy - ty * dx
            s = self.cumulative_s[i] + ratio * segment_length

            best_distance2 = distance2
            best = FrenetProjection(
                s=s,
                d=signed_d,
                reference_x=ref_x,
                reference_y=ref_y,
                reference_z=ref_z,
                tangent_x=tx,
                tangent_y=ty,
                segment_index=i,
                segment_ratio=ratio,
                distance_m=math.sqrt(distance2),
            )

        if best is None:
            raise RuntimeError("Could not project point onto the reference path")
        return best

    def point_at_s(self, s: float) -> Tuple[PathPoint, Tuple[float, float], int, float]:
        """Return reference point/tangent at accumulated distance s."""
        s_clamped = max(0.0, min(float(s), self.total_length_m))
        index = bisect.bisect_right(self.cumulative_s, s_clamped) - 1
        index = max(0, min(index, len(self.points) - 2))

        p = self.points[index]
        q = self.points[index + 1]
        vx = q.x - p.x
        vy = q.y - p.y
        segment_length = math.hypot(vx, vy)
        if segment_length < 1.0e-9:
            return p, (1.0, 0.0), index, 0.0

        ratio = (s_clamped - self.cumulative_s[index]) / segment_length
        ratio = max(0.0, min(1.0, ratio))
        point = PathPoint(
            p.x + ratio * vx,
            p.y + ratio * vy,
            p.z + ratio * (q.z - p.z),
        )
        return point, (vx / segment_length, vy / segment_length), index, ratio

    def frenet_to_map(self, s: float, d: float) -> PathPoint:
        reference, tangent, _index, _ratio = self.point_at_s(s)
        tx, ty = tangent
        # left normal = (-ty, tx)
        return PathPoint(
            reference.x - ty * d,
            reference.y + tx * d,
            reference.z,
        )
