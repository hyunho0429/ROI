"""Stanley path tracking over MORAI ENU CSV waypoints."""

import csv
import math
import os
from dataclasses import dataclass

from path_planning.localization import wrap_angle


@dataclass(frozen=True)
class PathPoint:
    x_m: float
    y_m: float
    z_m: float = 0.0


@dataclass(frozen=True)
class StanleyResult:
    steering_rad: float
    heading_error_rad: float
    cross_track_error_m: float
    segment_index: int
    remaining_distance_m: float
    goal_reached: bool


def load_path_csv(filename):
    """Load recorder CSV columns, or a simple x/y/z CSV, as ENU points."""
    filename = os.path.abspath(os.path.expanduser(filename))
    with open(filename, newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise ValueError("path CSV has no header")
        fields = set(reader.fieldnames)
        if {"global_enu_x_m", "global_enu_y_m"}.issubset(fields):
            keys = ("global_enu_x_m", "global_enu_y_m", "global_enu_z_m")
        elif {"x", "y"}.issubset(fields):
            keys = ("x", "y", "z")
        else:
            raise ValueError(
                "path CSV needs global_enu_x_m/global_enu_y_m or x/y columns"
            )
        points = []
        for line_number, row in enumerate(reader, start=2):
            try:
                point = PathPoint(
                    float(row[keys[0]]),
                    float(row[keys[1]]),
                    float(row.get(keys[2], 0.0) or 0.0),
                )
            except (TypeError, ValueError, KeyError) as error:
                raise ValueError("invalid path value at CSV line {}".format(line_number)) from error
            if not all(math.isfinite(value) for value in (point.x_m, point.y_m, point.z_m)):
                raise ValueError("non-finite path value at CSV line {}".format(line_number))
            if not points or math.dist(
                (point.x_m, point.y_m, point.z_m),
                (points[-1].x_m, points[-1].y_m, points[-1].z_m),
            ) > 1e-6:
                points.append(point)
    if len(points) < 2:
        raise ValueError("path CSV must contain at least two distinct points")
    return points


class StanleyController:
    def __init__(
        self,
        points,
        gain=1.2,
        softening_speed_mps=1.0,
        max_steering_deg=36.25,
        control_point_offset_m=0.0,
        z_distance_weight=0.25,
        search_back_segments=10,
        search_forward_segments=250,
        goal_tolerance_m=2.0,
    ):
        if len(points) < 2:
            raise ValueError("StanleyController needs at least two path points")
        self.points = list(points)
        self.gain = float(gain)
        self.softening_speed_mps = float(softening_speed_mps)
        self.max_steering_rad = math.radians(float(max_steering_deg))
        self.control_point_offset_m = float(control_point_offset_m)
        self.z_distance_weight = float(z_distance_weight)
        self.search_back_segments = int(search_back_segments)
        self.search_forward_segments = int(search_forward_segments)
        self.goal_tolerance_m = float(goal_tolerance_m)
        self._last_segment = None
        self._cumulative = [0.0]
        for first, second in zip(self.points, self.points[1:]):
            self._cumulative.append(
                self._cumulative[-1]
                + math.dist((first.x_m, first.y_m, first.z_m), (second.x_m, second.y_m, second.z_m))
            )

    def _nearest_segment(self, x_m, y_m, z_m):
        if self._last_segment is None:
            first_index, last_index = 0, len(self.points) - 2
        else:
            first_index = max(0, self._last_segment - self.search_back_segments)
            last_index = min(
                len(self.points) - 2,
                self._last_segment + self.search_forward_segments,
            )

        best = None
        for index in range(first_index, last_index + 1):
            first, second = self.points[index], self.points[index + 1]
            dx, dy = second.x_m - first.x_m, second.y_m - first.y_m
            length_xy_sq = dx * dx + dy * dy
            if length_xy_sq < 1e-12:
                continue
            fraction = ((x_m - first.x_m) * dx + (y_m - first.y_m) * dy) / length_xy_sq
            fraction = max(0.0, min(1.0, fraction))
            projected_x = first.x_m + fraction * dx
            projected_y = first.y_m + fraction * dy
            projected_z = first.z_m + fraction * (second.z_m - first.z_m)
            distance_sq = (
                (x_m - projected_x) ** 2
                + (y_m - projected_y) ** 2
                + self.z_distance_weight * (z_m - projected_z) ** 2
            )
            if best is None or distance_sq < best[0]:
                best = (distance_sq, index, fraction, projected_x, projected_y, projected_z)
        if best is None:
            raise RuntimeError("path contains no segment with horizontal length")
        self._last_segment = best[1]
        return best

    def compute(self, x_m, y_m, z_m, yaw_rad, speed_mps):
        control_x = x_m + self.control_point_offset_m * math.cos(yaw_rad)
        control_y = y_m + self.control_point_offset_m * math.sin(yaw_rad)
        nearest = self._nearest_segment(control_x, control_y, z_m)
        _, index, fraction, projected_x, projected_y, _projected_z = nearest
        first, second = self.points[index], self.points[index + 1]
        dx, dy = second.x_m - first.x_m, second.y_m - first.y_m
        length_xy = math.hypot(dx, dy)
        tangent_x, tangent_y = dx / length_xy, dy / length_xy
        path_yaw = math.atan2(tangent_y, tangent_x)
        heading_error = wrap_angle(path_yaw - yaw_rad)
        # Positive means the control point is to the left of the directed path.
        cross_track_error = (
            -(control_x - projected_x) * tangent_y
            + (control_y - projected_y) * tangent_x
        )
        correction = math.atan2(
            self.gain * cross_track_error,
            max(0.0, speed_mps) + self.softening_speed_mps,
        )
        steering = max(
            -self.max_steering_rad,
            min(self.max_steering_rad, heading_error - correction),
        )
        segment_3d = self._cumulative[index + 1] - self._cumulative[index]
        progress = self._cumulative[index] + fraction * segment_3d
        remaining = max(0.0, self._cumulative[-1] - progress)
        end = self.points[-1]
        end_distance = math.hypot(control_x - end.x_m, control_y - end.y_m)
        goal_reached = end_distance <= self.goal_tolerance_m and remaining <= 2.0 * self.goal_tolerance_m
        return StanleyResult(
            steering,
            heading_error,
            cross_track_error,
            index,
            remaining,
            goal_reached,
        )
