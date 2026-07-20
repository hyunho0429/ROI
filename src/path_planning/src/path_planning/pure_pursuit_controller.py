"""Pure Pursuit path tracking over MORAI map-local ENU waypoints."""

import bisect
import math
from dataclasses import dataclass

from path_planning.localization import wrap_angle
from path_planning.stanley_controller import preprocess_path_points


@dataclass(frozen=True)
class PurePursuitResult:
    steering_rad: float
    curvature_inv_m: float
    alpha_rad: float
    segment_index: int
    target_index: int
    lookahead_distance_m: float
    target_position_m: tuple
    path_yaw_rad: float
    cross_track_error_m: float
    remaining_distance_m: float
    goal_reached: bool
    target_speed_mps: float = None


class PurePursuitController:
    """Kinematic-bicycle Pure Pursuit using an arc-length lookahead target.

    The supplied reference implementation computes yaw rate as
    ``v * 2*sin(alpha) / Ld``.  MORAI Ego Ctrl Cmd expects steering instead,
    so this controller applies the bicycle relation
    ``steering = atan(wheelbase * 2*sin(alpha) / Ld)``.
    """

    def __init__(
        self,
        points,
        wheelbase_m=2.7,
        lookahead_distance_m=4.0,
        lookahead_speed_gain_s=0.5,
        minimum_lookahead_m=3.0,
        maximum_lookahead_m=12.0,
        max_steering_deg=21.77,
        control_point_offset_m=0.0,
        minimum_waypoint_spacing_m=0.5,
        waypoint_smoothing_window=9,
        z_distance_weight=0.25,
        search_back_segments=0,
        search_forward_segments=50,
        goal_tolerance_m=2.0,
    ):
        if len(points) < 2:
            raise ValueError("PurePursuitController needs at least two path points")
        if wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be positive")
        if lookahead_distance_m <= 0.0:
            raise ValueError("lookahead_distance_m must be positive")
        if minimum_lookahead_m <= 0.0:
            raise ValueError("minimum_lookahead_m must be positive")
        if maximum_lookahead_m < minimum_lookahead_m:
            raise ValueError("maximum_lookahead_m must be >= minimum_lookahead_m")

        self.original_point_count = len(points)
        self.points = preprocess_path_points(
            list(points),
            max(0.0, float(minimum_waypoint_spacing_m)),
            waypoint_smoothing_window,
        )
        self.wheelbase_m = float(wheelbase_m)
        self.lookahead_distance_m = float(lookahead_distance_m)
        self.lookahead_speed_gain_s = max(0.0, float(lookahead_speed_gain_s))
        self.minimum_lookahead_m = float(minimum_lookahead_m)
        self.maximum_lookahead_m = float(maximum_lookahead_m)
        self.max_steering_rad = math.radians(float(max_steering_deg))
        self.control_point_offset_m = float(control_point_offset_m)
        self.z_distance_weight = max(0.0, float(z_distance_weight))
        self.search_back_segments = max(0, int(search_back_segments))
        self.search_forward_segments = max(1, int(search_forward_segments))
        self.goal_tolerance_m = max(0.0, float(goal_tolerance_m))
        self._last_segment = None
        self._cumulative = [0.0]
        for first, second in zip(self.points, self.points[1:]):
            self._cumulative.append(
                self._cumulative[-1]
                + math.dist(
                    (first.x_m, first.y_m, first.z_m),
                    (second.x_m, second.y_m, second.z_m),
                )
            )

    @property
    def path_length_m(self):
        return self._cumulative[-1]

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
            fraction = (
                (x_m - first.x_m) * dx + (y_m - first.y_m) * dy
            ) / length_xy_sq
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
                best = (
                    distance_sq,
                    index,
                    fraction,
                    projected_x,
                    projected_y,
                    projected_z,
                )
        if best is None:
            raise RuntimeError("path contains no segment with horizontal length")
        self._last_segment = best[1]
        return best

    def _target_at_progress(self, progress_m):
        progress = max(0.0, min(self.path_length_m, float(progress_m)))
        if progress >= self.path_length_m:
            index = len(self.points) - 2
            fraction = 1.0
        else:
            index = max(0, bisect.bisect_right(self._cumulative, progress) - 1)
            index = min(index, len(self.points) - 2)
            segment_length = self._cumulative[index + 1] - self._cumulative[index]
            fraction = (
                0.0
                if segment_length <= 1e-12
                else (progress - self._cumulative[index]) / segment_length
            )
        first, second = self.points[index], self.points[index + 1]
        target = (
            first.x_m + fraction * (second.x_m - first.x_m),
            first.y_m + fraction * (second.y_m - first.y_m),
            first.z_m + fraction * (second.z_m - first.z_m),
        )
        if first.target_speed_mps is None:
            target_speed = second.target_speed_mps
        elif second.target_speed_mps is None:
            target_speed = first.target_speed_mps
        else:
            target_speed = first.target_speed_mps + fraction * (
                second.target_speed_mps - first.target_speed_mps
            )
        return index, target, target_speed

    def compute(self, x_m, y_m, z_m, yaw_rad, speed_mps, wheelbase_m=None):
        control_x = x_m + self.control_point_offset_m * math.cos(yaw_rad)
        control_y = y_m + self.control_point_offset_m * math.sin(yaw_rad)
        nearest = self._nearest_segment(control_x, control_y, z_m)
        _, index, fraction, projected_x, projected_y, _projected_z = nearest
        first, second = self.points[index], self.points[index + 1]
        segment_length = self._cumulative[index + 1] - self._cumulative[index]
        progress = self._cumulative[index] + fraction * segment_length
        remaining = max(0.0, self.path_length_m - progress)

        dx = second.x_m - first.x_m
        dy = second.y_m - first.y_m
        path_yaw = math.atan2(dy, dx)
        tangent_x, tangent_y = math.cos(path_yaw), math.sin(path_yaw)
        cross_track_error = (
            -(control_x - projected_x) * tangent_y
            + (control_y - projected_y) * tangent_x
        )

        commanded_lookahead = self.lookahead_distance_m + (
            self.lookahead_speed_gain_s * max(0.0, abs(float(speed_mps)))
        )
        commanded_lookahead = max(
            self.minimum_lookahead_m,
            min(self.maximum_lookahead_m, commanded_lookahead),
        )
        target_index, target, target_speed = self._target_at_progress(
            progress + commanded_lookahead
        )
        target_dx = target[0] - control_x
        target_dy = target[1] - control_y
        actual_lookahead = math.hypot(target_dx, target_dy)
        target_bearing = math.atan2(target_dy, target_dx)
        alpha = wrap_angle(target_bearing - yaw_rad)

        end = self.points[-1]
        end_distance = math.hypot(control_x - end.x_m, control_y - end.y_m)
        goal_reached = (
            end_distance <= self.goal_tolerance_m
            and remaining <= 2.0 * self.goal_tolerance_m
        )
        if goal_reached or actual_lookahead <= 1e-6:
            curvature = 0.0
            steering = 0.0
        else:
            curvature = 2.0 * math.sin(alpha) / actual_lookahead
            active_wheelbase = (
                self.wheelbase_m
                if wheelbase_m is None or wheelbase_m <= 0.0
                else float(wheelbase_m)
            )
            steering = math.atan(active_wheelbase * curvature)
            steering = max(
                -self.max_steering_rad,
                min(self.max_steering_rad, steering),
            )

        return PurePursuitResult(
            steering_rad=steering,
            curvature_inv_m=curvature,
            alpha_rad=alpha,
            segment_index=index,
            target_index=target_index,
            lookahead_distance_m=actual_lookahead,
            target_position_m=target,
            path_yaw_rad=path_yaw,
            cross_track_error_m=cross_track_error,
            remaining_distance_m=remaining,
            goal_reached=goal_reached,
            target_speed_mps=target_speed,
        )
