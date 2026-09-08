"""Frenet candidate generation and collision checking for MORAI ROS1.

The reference path and every obstacle are expressed in the same ``map``
frame.  Path points represent the vehicle rear-axle centre, matching the
existing Pure Pursuit controller.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

from purepursuit_mgeo.path import PathPoint


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class FrenetProjection:
    s: float
    d: float
    yaw: float
    segment_index: int


@dataclass(frozen=True)
class TrackedObstacle:
    track_id: int
    center_x: float
    center_y: float
    length: float
    width: float
    velocity_x: float
    velocity_y: float
    speed: float
    motion_state: str
    yaw: float
    yaw_valid: bool

    @classmethod
    def from_dict(cls, values: dict) -> "TrackedObstacle":
        velocity_x = float(values.get("velocity_x_map", 0.0))
        velocity_y = float(values.get("velocity_y_map", 0.0))
        return cls(
            track_id=int(values.get("id", -1)),
            center_x=float(values["center_x_map"]),
            center_y=float(values["center_y_map"]),
            length=max(0.0, float(values.get("length", 0.0))),
            width=max(0.0, float(values.get("width", 0.0))),
            velocity_x=velocity_x,
            velocity_y=velocity_y,
            speed=float(values.get("speed_mps", math.hypot(velocity_x, velocity_y))),
            motion_state=str(values.get("motion_state", "UNKNOWN")),
            yaw=float(values.get("yaw", 0.0)),
            yaw_valid=bool(values.get("yaw_valid", False)),
        )

    def predicted(self, seconds: float) -> Tuple[float, float]:
        seconds = max(0.0, seconds)
        return (
            self.center_x + self.velocity_x * seconds,
            self.center_y + self.velocity_y * seconds,
        )


@dataclass
class FrenetCandidate:
    points: List[PathPoint]
    times: List[float]
    lateral_offsets: List[float]
    yaws: List[float]
    target_offset: float
    planning_time: float
    jerk_cost: float
    safety_cost: float
    offset_cost: float
    final_offset_cost: float
    time_cost: float
    side_cost: float
    total_cost: float
    minimum_clearance: float


class ReferencePath:
    """Polyline reference path with Cartesian/Frenet projection helpers."""

    def __init__(self, points: Sequence[PathPoint]) -> None:
        filtered: List[PathPoint] = []
        for point in points:
            if not filtered or math.hypot(
                point.x - filtered[-1].x, point.y - filtered[-1].y
            ) > 1.0e-6:
                filtered.append(point)
        if len(filtered) < 2:
            raise ValueError("reference path needs at least two distinct points")

        self.points = filtered
        self.cumulative_s = [0.0]
        self.segment_lengths: List[float] = []
        self.segment_yaws: List[float] = []
        for first, second in zip(filtered[:-1], filtered[1:]):
            dx = second.x - first.x
            dy = second.y - first.y
            length = math.hypot(dx, dy)
            self.segment_lengths.append(length)
            self.segment_yaws.append(math.atan2(dy, dx))
            self.cumulative_s.append(self.cumulative_s[-1] + length)
        self.length = self.cumulative_s[-1]

    def project(self, x: float, y: float) -> FrenetProjection:
        best_distance_sq = float("inf")
        best = None
        for index, (first, length, yaw) in enumerate(
            zip(self.points[:-1], self.segment_lengths, self.segment_yaws)
        ):
            tangent_x = math.cos(yaw)
            tangent_y = math.sin(yaw)
            relative_x = x - first.x
            relative_y = y - first.y
            ratio = _clamp(
                (relative_x * tangent_x + relative_y * tangent_y) / length,
                0.0,
                1.0,
            )
            projected_x = first.x + ratio * length * tangent_x
            projected_y = first.y + ratio * length * tangent_y
            error_x = x - projected_x
            error_y = y - projected_y
            distance_sq = error_x * error_x + error_y * error_y
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                signed_offset = -tangent_y * error_x + tangent_x * error_y
                best = FrenetProjection(
                    s=self.cumulative_s[index] + ratio * length,
                    d=signed_offset,
                    yaw=yaw,
                    segment_index=index,
                )
        assert best is not None
        return best

    def sample(self, s_value: float, lateral_offset: float = 0.0) -> PathPoint:
        s_value = _clamp(s_value, 0.0, self.length)
        index = bisect.bisect_right(self.cumulative_s, s_value) - 1
        index = max(0, min(index, len(self.segment_lengths) - 1))
        segment_s = s_value - self.cumulative_s[index]
        length = self.segment_lengths[index]
        ratio = _clamp(segment_s / length, 0.0, 1.0)
        first = self.points[index]
        second = self.points[index + 1]
        reference_x = first.x + ratio * (second.x - first.x)
        reference_y = first.y + ratio * (second.y - first.y)
        reference_z = first.z + ratio * (second.z - first.z)
        yaw = self.segment_yaws[index]
        return PathPoint(
            reference_x - math.sin(yaw) * lateral_offset,
            reference_y + math.cos(yaw) * lateral_offset,
            reference_z,
        )


def _quintic_coefficients(
    start_position: float,
    start_velocity: float,
    start_acceleration: float,
    end_position: float,
    end_velocity: float,
    end_acceleration: float,
    duration: float,
) -> Tuple[float, float, float, float, float, float]:
    if duration <= 0.0:
        raise ValueError("duration must be positive")
    t = duration
    c0 = start_position
    c1 = start_velocity
    c2 = 0.5 * start_acceleration
    c3 = (
        20.0 * (end_position - start_position)
        - (8.0 * end_velocity + 12.0 * start_velocity) * t
        - (3.0 * start_acceleration - end_acceleration) * t * t
    ) / (2.0 * t ** 3)
    c4 = (
        30.0 * (start_position - end_position)
        + (14.0 * end_velocity + 16.0 * start_velocity) * t
        + (3.0 * start_acceleration - 2.0 * end_acceleration) * t * t
    ) / (2.0 * t ** 4)
    c5 = (
        12.0 * (end_position - start_position)
        - (6.0 * end_velocity + 6.0 * start_velocity) * t
        - (start_acceleration - end_acceleration) * t * t
    ) / (2.0 * t ** 5)
    return c0, c1, c2, c3, c4, c5


def _evaluate_quintic(
    coefficients: Sequence[float], time_value: float
) -> Tuple[float, float, float, float]:
    c0, c1, c2, c3, c4, c5 = coefficients
    t = time_value
    position = c0 + c1 * t + c2 * t ** 2 + c3 * t ** 3 + c4 * t ** 4 + c5 * t ** 5
    velocity = c1 + 2.0 * c2 * t + 3.0 * c3 * t ** 2 + 4.0 * c4 * t ** 3 + 5.0 * c5 * t ** 4
    acceleration = 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t ** 2 + 20.0 * c5 * t ** 3
    jerk = 6.0 * c3 + 24.0 * c4 * t + 60.0 * c5 * t ** 2
    return position, velocity, acceleration, jerk


def _projection_radius(
    axis_x: float,
    axis_y: float,
    rectangle_yaw: float,
    half_length: float,
    half_width: float,
) -> float:
    forward_x = math.cos(rectangle_yaw)
    forward_y = math.sin(rectangle_yaw)
    left_x = -forward_y
    left_y = forward_x
    return (
        half_length * abs(axis_x * forward_x + axis_y * forward_y)
        + half_width * abs(axis_x * left_x + axis_y * left_y)
    )


def _rectangles_overlap(
    first_x: float,
    first_y: float,
    first_yaw: float,
    first_half_length: float,
    first_half_width: float,
    second_x: float,
    second_y: float,
    second_yaw: float,
    second_half_length: float,
    second_half_width: float,
) -> bool:
    delta_x = second_x - first_x
    delta_y = second_y - first_y
    axes = (
        (math.cos(first_yaw), math.sin(first_yaw)),
        (-math.sin(first_yaw), math.cos(first_yaw)),
        (math.cos(second_yaw), math.sin(second_yaw)),
        (-math.sin(second_yaw), math.cos(second_yaw)),
    )
    for axis_x, axis_y in axes:
        centre_distance = abs(delta_x * axis_x + delta_y * axis_y)
        first_radius = _projection_radius(
            axis_x,
            axis_y,
            first_yaw,
            first_half_length,
            first_half_width,
        )
        second_radius = _projection_radius(
            axis_x,
            axis_y,
            second_yaw,
            second_half_length,
            second_half_width,
        )
        if centre_distance > first_radius + second_radius:
            return False
    return True


class FrenetPlanner:
    def __init__(
        self,
        reference: ReferencePath,
        vehicle_length: float = 4.635,
        vehicle_width: float = 1.892,
        rear_axle_to_center: float = 1.35,
        safety_margin: float = 0.45,
        minimum_obstacle_length: float = 0.50,
        minimum_obstacle_width: float = 0.35,
        sample_time: float = 0.10,
        safety_sigma: float = 2.0,
        maximum_lateral_acceleration: float = 3.0,
        maximum_curvature: float = 0.28,
        preferred_side: str = "left",
    ) -> None:
        self.reference = reference
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width
        self.rear_axle_to_center = rear_axle_to_center
        self.safety_margin = safety_margin
        self.minimum_obstacle_length = minimum_obstacle_length
        self.minimum_obstacle_width = minimum_obstacle_width
        self.sample_time = sample_time
        self.safety_sigma = safety_sigma
        self.maximum_lateral_acceleration = maximum_lateral_acceleration
        self.maximum_curvature = maximum_curvature
        self.preferred_side = preferred_side.lower()

        self.weight_jerk = 0.20
        self.weight_safety = 4.00
        self.weight_offset = 0.35
        self.weight_final_offset = 0.20
        self.weight_time = 0.05
        self.side_preference_cost = 0.15

    def obstacle_lateral_interval(
        self, obstacle: TrackedObstacle, prediction_time: float
    ) -> Tuple[float, float]:
        """Return min/max Frenet d of the four predicted OBB corners."""
        center_x, center_y = obstacle.predicted(prediction_time)
        center_projection = self.reference.project(center_x, center_y)
        yaw = obstacle.yaw if obstacle.yaw_valid else center_projection.yaw
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        left_x = -forward_y
        left_y = forward_x
        half_length = 0.5 * max(obstacle.length, self.minimum_obstacle_length)
        half_width = 0.5 * max(obstacle.width, self.minimum_obstacle_width)
        corner_offsets = (
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, half_width),
            (-half_length, -half_width),
        )
        corner_d_values = []
        for longitudinal, lateral in corner_offsets:
            corner_x = center_x + longitudinal * forward_x + lateral * left_x
            corner_y = center_y + longitudinal * forward_y + lateral * left_y
            corner_d_values.append(self.reference.project(corner_x, corner_y).d)
        return min(corner_d_values), max(corner_d_values)

    @staticmethod
    def nearest_lateral_edge(d_min: float, d_max: float) -> float:
        """Signed BBox edge nearest d=0; return zero when it straddles it."""
        if d_min <= 0.0 <= d_max:
            return 0.0
        return d_min if d_min > 0.0 else d_max

    def clearance_target_offsets(
        self,
        obstacles: Sequence[TrackedObstacle],
        obstacle_age: float,
        configured_offsets: Sequence[float],
    ) -> List[float]:
        """Keep only target d values fully outside all blocking BBoxes.

        A left pass must place the vehicle centre beyond the largest d edge;
        a right pass must place it beyond the smallest d edge.  The hard OBB
        collision test still validates the full time-varying trajectory.
        """
        if not obstacles:
            return [float(value) for value in configured_offsets]
        intervals = [
            self.obstacle_lateral_interval(obstacle, obstacle_age)
            for obstacle in obstacles
        ]
        rightmost_edge = max(d_max for _d_min, d_max in intervals)
        leftmost_edge = min(d_min for d_min, _d_max in intervals)
        centre_clearance = 0.5 * self.vehicle_width + self.safety_margin
        left_required = rightmost_edge + centre_clearance
        right_required = leftmost_edge - centre_clearance
        tolerance = 1.0e-6
        return [
            float(offset)
            for offset in configured_offsets
            if float(offset) >= left_required - tolerance
            or float(offset) <= right_required + tolerance
        ]

    def relevant_obstacles(
        self,
        obstacles: Sequence[TrackedObstacle],
        ego_s: float,
        ego_speed: float,
        vehicle_width: float,
        trigger_distance: float,
        rear_distance: float,
        obstacle_age: float,
    ) -> List[TrackedObstacle]:
        relevant = []
        prediction_speed = max(ego_speed, 1.0)
        for obstacle in obstacles:
            current_x, current_y = obstacle.predicted(obstacle_age)
            current_projection = self.reference.project(current_x, current_y)
            delta_s = current_projection.s - ego_s
            if delta_s < -rear_distance or delta_s > trigger_distance:
                continue

            arrival_time = max(0.0, delta_s) / prediction_speed
            current_d_min, current_d_max = self.obstacle_lateral_interval(
                obstacle, obstacle_age
            )
            future_d_min, future_d_max = self.obstacle_lateral_interval(
                obstacle, obstacle_age + arrival_time
            )
            current_nearest_d = self.nearest_lateral_edge(
                current_d_min, current_d_max
            )
            future_nearest_d = self.nearest_lateral_edge(
                future_d_min, future_d_max
            )
            trigger_clearance = 0.5 * vehicle_width + self.safety_margin
            if (
                abs(current_nearest_d) <= trigger_clearance
                or abs(future_nearest_d) <= trigger_clearance
            ):
                relevant.append(obstacle)
        return relevant

    def plan(
        self,
        vehicle_x: float,
        vehicle_y: float,
        vehicle_yaw: float,
        vehicle_speed: float,
        target_speed: float,
        obstacles: Sequence[TrackedObstacle],
        obstacle_age: float,
        target_offsets: Sequence[float],
        planning_times: Sequence[float],
    ) -> Optional[FrenetCandidate]:
        projection = self.reference.project(vehicle_x, vehicle_y)
        heading_error = _wrap_angle(vehicle_yaw - projection.yaw)
        start_lateral_speed = vehicle_speed * math.sin(heading_error)
        planning_speed = max(1.0, vehicle_speed, target_speed)
        remaining_path = self.reference.length - projection.s
        if remaining_path < max(5.0, planning_speed):
            return None

        unique_offsets = []
        for value in target_offsets:
            value = float(value)
            if not any(abs(value - existing) < 1.0e-6 for existing in unique_offsets):
                unique_offsets.append(value)

        candidates: List[FrenetCandidate] = []
        maximum_time = max(float(value) for value in planning_times)
        maximum_offset = max(1.0, max(abs(value) for value in unique_offsets))
        for duration_value in planning_times:
            duration = float(duration_value)
            if duration <= 0.5:
                continue
            for target_offset in unique_offsets:
                coefficients = _quintic_coefficients(
                    projection.d,
                    start_lateral_speed,
                    0.0,
                    target_offset,
                    0.0,
                    0.0,
                    duration,
                )
                candidate = self._build_candidate(
                    projection.s,
                    planning_speed,
                    coefficients,
                    target_offset,
                    duration,
                    maximum_time,
                    maximum_offset,
                    obstacles,
                    obstacle_age,
                )
                if candidate is not None:
                    candidates.append(candidate)
        if not candidates:
            return None
        return min(candidates, key=lambda candidate: candidate.total_cost)

    def _build_candidate(
        self,
        start_s: float,
        planning_speed: float,
        coefficients: Sequence[float],
        target_offset: float,
        duration: float,
        maximum_time: float,
        maximum_offset: float,
        obstacles: Sequence[TrackedObstacle],
        obstacle_age: float,
    ) -> Optional[FrenetCandidate]:
        step_count = max(2, int(math.ceil(duration / self.sample_time)))
        times = [duration * index / step_count for index in range(step_count + 1)]
        points: List[PathPoint] = []
        offsets: List[float] = []
        jerks: List[float] = []
        for time_value in times:
            offset, _offset_speed, _offset_acceleration, offset_jerk = _evaluate_quintic(
                coefficients, time_value
            )
            s_value = min(self.reference.length, start_s + planning_speed * time_value)
            point = self.reference.sample(s_value, offset)
            if points and math.hypot(point.x - points[-1].x, point.y - points[-1].y) < 1.0e-5:
                return None
            points.append(point)
            offsets.append(offset)
            jerks.append(offset_jerk)

        yaws = self._candidate_yaws(points)
        if not self._kinematically_valid(points, yaws, planning_speed):
            return None

        clearance_values: List[float] = []
        for point, yaw, time_value in zip(points, yaws, times):
            vehicle_center_x = point.x + self.rear_axle_to_center * math.cos(yaw)
            vehicle_center_y = point.y + self.rear_axle_to_center * math.sin(yaw)
            for obstacle in obstacles:
                obstacle_x, obstacle_y = obstacle.predicted(obstacle_age + time_value)
                obstacle_yaw = obstacle.yaw if obstacle.yaw_valid else 0.0
                obstacle_length = max(obstacle.length, self.minimum_obstacle_length)
                obstacle_width = max(obstacle.width, self.minimum_obstacle_width)
                if _rectangles_overlap(
                    vehicle_center_x,
                    vehicle_center_y,
                    yaw,
                    0.5 * self.vehicle_length + self.safety_margin,
                    0.5 * self.vehicle_width + self.safety_margin,
                    obstacle_x,
                    obstacle_y,
                    obstacle_yaw,
                    0.5 * obstacle_length,
                    0.5 * obstacle_width,
                ):
                    return None
                centre_distance = math.hypot(
                    obstacle_x - vehicle_center_x, obstacle_y - vehicle_center_y
                )
                approximate_clearance = max(
                    0.0,
                    centre_distance
                    - 0.5 * math.hypot(obstacle_length, obstacle_width)
                    - 0.5 * self.vehicle_width,
                )
                clearance_values.append(approximate_clearance)

        speed_normalizer = max(planning_speed, 1.0)
        jerk_cost = sum((value / speed_normalizer) ** 2 for value in jerks) / len(jerks)
        if clearance_values:
            safety_cost = sum(
                math.exp(-value / max(self.safety_sigma, 1.0e-3))
                for value in clearance_values
            ) / len(clearance_values)
            minimum_clearance = min(clearance_values)
        else:
            safety_cost = 0.0
            minimum_clearance = float("inf")
        offset_cost = sum((value / maximum_offset) ** 2 for value in offsets) / len(offsets)
        final_offset_cost = (offsets[-1] / maximum_offset) ** 2
        time_cost = duration / maximum_time
        side_cost = 0.0
        if self.preferred_side == "left" and target_offset < -1.0e-3:
            side_cost = self.side_preference_cost
        elif self.preferred_side == "right" and target_offset > 1.0e-3:
            side_cost = self.side_preference_cost
        total_cost = (
            self.weight_jerk * jerk_cost
            + self.weight_safety * safety_cost
            + self.weight_offset * offset_cost
            + self.weight_final_offset * final_offset_cost
            + self.weight_time * time_cost
            + side_cost
        )
        return FrenetCandidate(
            points=points,
            times=times,
            lateral_offsets=offsets,
            yaws=yaws,
            target_offset=target_offset,
            planning_time=duration,
            jerk_cost=jerk_cost,
            safety_cost=safety_cost,
            offset_cost=offset_cost,
            final_offset_cost=final_offset_cost,
            time_cost=time_cost,
            side_cost=side_cost,
            total_cost=total_cost,
            minimum_clearance=minimum_clearance,
        )

    @staticmethod
    def _candidate_yaws(points: Sequence[PathPoint]) -> List[float]:
        yaws = []
        for index, point in enumerate(points):
            if index < len(points) - 1:
                other = points[index + 1]
                yaws.append(math.atan2(other.y - point.y, other.x - point.x))
            else:
                other = points[index - 1]
                yaws.append(math.atan2(point.y - other.y, point.x - other.x))
        return yaws

    def _kinematically_valid(
        self,
        points: Sequence[PathPoint],
        yaws: Sequence[float],
        speed: float,
    ) -> bool:
        for index in range(1, len(points)):
            distance = math.hypot(
                points[index].x - points[index - 1].x,
                points[index].y - points[index - 1].y,
            )
            if distance < 1.0e-5:
                return False
            curvature = abs(_wrap_angle(yaws[index] - yaws[index - 1])) / distance
            if curvature > self.maximum_curvature:
                return False
            if speed * speed * curvature > self.maximum_lateral_acceleration:
                return False
        return True
