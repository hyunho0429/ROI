"""Pure intersection-detection and crossing-vehicle tracking logic."""

import math
from dataclasses import dataclass


def left_to_right_crossing_obstacles(
    obstacles,
    ego_x_map,
    ego_y_map,
    ego_yaw,
    minimum_speed_mps=1.0,
    minimum_rightward_speed_mps=0.5,
    maximum_forward_distance_m=40.0,
    maximum_abs_lateral_distance_m=20.0,
):
    """Return moving front objects travelling toward the ego-frame right.

    Input positions and velocities remain in ``map``. Each returned record
    additionally carries ego-frame longitudinal/lateral position and lateral
    speed. Ego-frame positive lateral is left, so left-to-right motion has a
    negative lateral speed.
    """
    cosine = math.cos(float(ego_yaw))
    sine = math.sin(float(ego_yaw))
    selected = []
    for obstacle in obstacles:
        if str(obstacle.get("motion_state", "")).upper() != "MOVING":
            continue
        try:
            track_id = int(obstacle["id"])
            center_x = float(obstacle["center_x_map"])
            center_y = float(obstacle["center_y_map"])
            velocity_x = float(obstacle["velocity_x_map"])
            velocity_y = float(obstacle["velocity_y_map"])
            length = max(0.0, float(obstacle.get("length", 0.0)))
            width = max(0.0, float(obstacle.get("width", 0.0)))
        except (KeyError, TypeError, ValueError):
            continue
        values = (center_x, center_y, velocity_x, velocity_y, length, width)
        if not all(math.isfinite(value) for value in values):
            continue

        delta_x = center_x - float(ego_x_map)
        delta_y = center_y - float(ego_y_map)
        longitudinal = cosine * delta_x + sine * delta_y
        lateral = -sine * delta_x + cosine * delta_y
        lateral_speed = -sine * velocity_x + cosine * velocity_y
        speed = math.hypot(velocity_x, velocity_y)
        try:
            obstacle_yaw = float(obstacle.get("yaw", math.nan))
        except (TypeError, ValueError):
            obstacle_yaw = math.nan
        if bool(obstacle.get("yaw_valid", False)) and math.isfinite(obstacle_yaw):
            relative_yaw = obstacle_yaw - float(ego_yaw)
            lateral_half_extent = 0.5 * (
                abs(math.sin(relative_yaw)) * length
                + abs(math.cos(relative_yaw)) * width
            )
        else:
            # Unknown box orientation: max dimension is the conservative
            # lateral projection for a crossing-clearance decision.
            lateral_half_extent = 0.5 * max(length, width)
        if not 0.0 <= longitudinal <= float(maximum_forward_distance_m):
            continue
        if abs(lateral) > float(maximum_abs_lateral_distance_m):
            continue
        if speed < float(minimum_speed_mps):
            continue
        if lateral_speed > -float(minimum_rightward_speed_mps):
            continue
        selected.append(
            {
                "id": track_id,
                "longitudinal_m": longitudinal,
                "lateral_m": lateral,
                "lateral_speed_mps": lateral_speed,
                "lateral_half_extent_m": lateral_half_extent,
            }
        )
    return selected


class LeftToRightCrossingTracker:
    """Remember left-side track IDs until their boxes clear ego's right side."""

    def __init__(self, ego_vehicle_width_m=1.892, clearance_m=0.2):
        if ego_vehicle_width_m <= 0.0 or clearance_m < 0.0:
            raise ValueError("vehicle width must be positive and clearance non-negative")
        self.ego_vehicle_width_m = float(ego_vehicle_width_m)
        self.clearance_m = float(clearance_m)
        self.target_ids = set()
        self.passed_ids = set()

    def reset(self):
        self.target_ids.clear()
        self.passed_ids.clear()

    def update(self, observations, active=True):
        if not active:
            self.reset()
            return False, (), ()

        right_boundary = -(0.5 * self.ego_vehicle_width_m + self.clearance_m)
        for observation in observations:
            track_id = int(observation["id"])
            lateral = float(observation["lateral_m"])
            lateral_half_extent = max(
                0.0,
                float(observation.get("lateral_half_extent_m", 0.0)),
            )
            # Enrol only a right-moving object first observed on the ego-left
            # or centre. This prevents unrelated right-side traffic from
            # falsely releasing the brake.
            if lateral >= 0.0:
                self.target_ids.add(track_id)
            left_edge = lateral + lateral_half_extent
            if track_id in self.target_ids and left_edge <= right_boundary:
                self.passed_ids.add(track_id)

        waiting_ids = self.target_ids - self.passed_ids
        all_passed = bool(self.target_ids) and not waiting_ids
        return (
            all_passed,
            tuple(sorted(waiting_ids)),
            tuple(sorted(self.passed_ids)),
        )


@dataclass(frozen=True)
class IntersectionDecision:
    state: str
    detected: bool
    driving_allowed: bool
    driving_unavailable: bool


class IntersectionStateMachine:
    """Recognize ``Car AND left yellow solid AND right solid`` intersections.

    After recognition, a tracked left-to-right vehicle clears the stop once
    its complete box passes the ego-right boundary. Camera disappearance is
    retained as a conservative fallback; stale camera data never releases a
    blocked intersection.
    """

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
        left_yellow_solid_lane_detected: bool,
        right_solid_lane_detected: bool,
        now: float,
        camera_fresh: bool = True,
        lane_fresh: bool = True,
        crossing_vehicle_passed_right: bool = False,
        crossing_vehicle_waiting: bool = False,
    ) -> IntersectionDecision:
        now = float(now)
        recognition_conditions_met = bool(
            camera_fresh
            and lane_fresh
            and camera_vehicle_detected
            and left_yellow_solid_lane_detected
            and right_solid_lane_detected
        )

        if self.state == "IDLE":
            if recognition_conditions_met:
                self.state = "BLOCKED"
                self.camera_clear_since = None

        elif self.state == "BLOCKED":
            if crossing_vehicle_passed_right:
                self.state = "CLEAR"
                self.camera_clear_since = None
                self.clear_started_at = None
            # A stale camera must never release an already-blocked intersection.
            elif not camera_fresh or camera_vehicle_detected:
                self.camera_clear_since = None
            else:
                if self.camera_clear_since is None:
                    self.camera_clear_since = now
                if now - self.camera_clear_since >= self.camera_clear_confirmation_s:
                    self.state = "CLEAR"
                    self.clear_started_at = now

        elif self.state == "CLEAR":
            if recognition_conditions_met and (
                crossing_vehicle_waiting or not crossing_vehicle_passed_right
            ):
                self.state = "BLOCKED"
                self.camera_clear_since = None
                self.clear_started_at = None
            elif camera_fresh and not camera_vehicle_detected:
                if self.clear_started_at is None:
                    self.clear_started_at = now
                elif now - self.clear_started_at >= self.clear_hold_s:
                    self.state = "IDLE"
                    self.clear_started_at = None
            else:
                self.clear_started_at = None

        return IntersectionDecision(
            state=self.state,
            detected=self.state != "IDLE",
            driving_allowed=self.state == "CLEAR",
            driving_unavailable=self.state == "BLOCKED",
        )
