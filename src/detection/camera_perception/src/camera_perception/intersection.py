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
        except (KeyError, TypeError, ValueError):
            continue
        values = (center_x, center_y, velocity_x, velocity_y)
        if not all(math.isfinite(value) for value in values):
            continue

        delta_x = center_x - float(ego_x_map)
        delta_y = center_y - float(ego_y_map)
        longitudinal = cosine * delta_x + sine * delta_y
        lateral = -sine * delta_x + cosine * delta_y
        lateral_speed = -sine * velocity_x + cosine * velocity_y
        speed = math.hypot(velocity_x, velocity_y)
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
            }
        )
    return selected


class FrontCrossingVehicleLatch:
    """Latch IDs as soon as right-moving traffic is observed in front."""

    def __init__(self):
        self.observed_ids = set()

    def reset(self):
        self.observed_ids.clear()

    def update(self, observations, active=True):
        if not active:
            self.reset()
            return False, ()

        for observation in observations:
            self.observed_ids.add(int(observation["id"]))
        return bool(self.observed_ids), tuple(sorted(self.observed_ids))


@dataclass(frozen=True)
class IntersectionDecision:
    state: str
    detected: bool
    driving_allowed: bool
    driving_unavailable: bool


class IntersectionStateMachine:
    """Recognize ``Car AND left yellow solid AND right solid`` intersections.

    After recognition, observing a tracked left-to-right moving vehicle in
    front immediately clears the stop. Camera disappearance is retained as a
    fallback; stale camera data never releases a blocked intersection.
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
        crossing_vehicle_seen_in_front: bool = False,
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
                self.state = (
                    "CLEAR" if crossing_vehicle_seen_in_front else "BLOCKED"
                )
                self.camera_clear_since = None
                self.clear_started_at = None

        elif self.state == "BLOCKED":
            if crossing_vehicle_seen_in_front:
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
            if recognition_conditions_met and not crossing_vehicle_seen_in_front:
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
