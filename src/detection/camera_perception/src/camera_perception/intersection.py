"""Pure intersection-detection logic shared by the ROS node and tests."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Mapping


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def perpendicular_dynamic_obstacles(
    obstacles: Iterable[Mapping[str, object]],
    ego_x_map: float,
    ego_y_map: float,
    ego_yaw: float,
    minimum_speed_mps: float = 1.0,
    maximum_range_m: float = 40.0,
    maximum_perpendicular_error_deg: float = 20.0,
):
    """Return MOVING obstacles whose map velocity is nearly normal to ego yaw."""

    selected = []
    maximum_error = math.radians(maximum_perpendicular_error_deg)
    for obstacle in obstacles:
        if str(obstacle.get("motion_state", "")).upper() != "MOVING":
            continue
        try:
            center_x = float(obstacle["center_x_map"])
            center_y = float(obstacle["center_y_map"])
            velocity_x = float(obstacle["velocity_x_map"])
            velocity_y = float(obstacle["velocity_y_map"])
        except (KeyError, TypeError, ValueError):
            continue
        values = (center_x, center_y, velocity_x, velocity_y)
        if not all(math.isfinite(value) for value in values):
            continue
        speed = math.hypot(velocity_x, velocity_y)
        if speed < minimum_speed_mps:
            continue
        if math.hypot(center_x - ego_x_map, center_y - ego_y_map) > maximum_range_m:
            continue
        velocity_yaw = math.atan2(velocity_y, velocity_x)
        heading_difference = abs(normalize_angle(velocity_yaw - ego_yaw))
        perpendicular_error = abs(heading_difference - math.pi / 2.0)
        if perpendicular_error <= maximum_error:
            selected.append(obstacle)
    return selected


@dataclass(frozen=True)
class IntersectionDecision:
    state: str
    detected: bool
    driving_allowed: bool
    driving_unavailable: bool


class IntersectionStateMachine:
    """Latch a crossing encounter until the camera is clear long enough."""

    def __init__(self, camera_clear_confirmation_s: float = 0.5, clear_hold_s: float = 2.0):
        if camera_clear_confirmation_s < 0.0:
            raise ValueError("camera_clear_confirmation_s must be non-negative")
        if clear_hold_s < 0.0:
            raise ValueError("clear_hold_s must be non-negative")
        self.camera_clear_confirmation_s = float(camera_clear_confirmation_s)
        self.clear_hold_s = float(clear_hold_s)
        self.state = "IDLE"
        self.camera_clear_since = None
        self.clear_started_at = None

    def update(
        self,
        camera_vehicle_detected: bool,
        perpendicular_dynamic_detected: bool,
        now: float,
        camera_fresh: bool = True,
    ) -> IntersectionDecision:
        now = float(now)
        if self.state == "IDLE":
            if camera_fresh and camera_vehicle_detected and perpendicular_dynamic_detected:
                self.state = "BLOCKED"
                self.camera_clear_since = None

        elif self.state == "BLOCKED":
            # A stale camera must never release an already-blocked intersection.
            if not camera_fresh or camera_vehicle_detected:
                self.camera_clear_since = None
            else:
                if self.camera_clear_since is None:
                    self.camera_clear_since = now
                if now - self.camera_clear_since >= self.camera_clear_confirmation_s:
                    self.state = "CLEAR"
                    self.clear_started_at = now

        elif self.state == "CLEAR":
            # The perpendicular LiDAR condition is required only to recognize
            # the encounter initially. Once recognized, any camera vehicle
            # makes the intersection unavailable again.
            if camera_fresh and camera_vehicle_detected:
                self.state = "BLOCKED"
                self.camera_clear_since = None
                self.clear_started_at = None
            elif self.clear_started_at is not None and now - self.clear_started_at >= self.clear_hold_s:
                self.state = "IDLE"
                self.clear_started_at = None

        return IntersectionDecision(
            state=self.state,
            detected=self.state != "IDLE",
            driving_allowed=self.state == "CLEAR",
            driving_unavailable=self.state == "BLOCKED",
        )
