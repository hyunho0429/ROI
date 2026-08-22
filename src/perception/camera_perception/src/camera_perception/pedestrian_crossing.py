"""Pure pedestrian-crossing fusion geometry and state-machine logic."""

from dataclasses import dataclass
import math


def quaternion_to_yaw(x, y, z, w):
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def obstacle_relative_geometry(obstacle, ego_x, ego_y, ego_yaw):
    """Return forward, left, and approximate closest LiDAR distance."""

    delta_x = float(obstacle["center_x_map"]) - float(ego_x)
    delta_y = float(obstacle["center_y_map"]) - float(ego_y)
    cosine = math.cos(float(ego_yaw))
    sine = math.sin(float(ego_yaw))
    forward_m = cosine * delta_x + sine * delta_y
    left_m = -sine * delta_x + cosine * delta_y

    length_m = max(0.0, float(obstacle.get("length", 0.0)))
    width_m = max(0.0, float(obstacle.get("width", 0.0)))
    half_diagonal_m = 0.5 * math.hypot(length_m, width_m)
    closest_distance_m = max(
        0.0, math.hypot(forward_m, left_m) - half_diagonal_m
    )
    return forward_m, left_m, closest_distance_m


def pedestrian_lidar_candidates(
    obstacles,
    ego_x,
    ego_y,
    ego_yaw,
    detection_distance_m=1.5,
    rear_allowance_m=0.5,
    max_object_length_m=1.5,
    max_object_width_m=1.5,
):
    """Select human-sized clusters within the front/side safety distance."""

    candidates = []
    for obstacle in obstacles:
        try:
            length_m = max(0.0, float(obstacle.get("length", 0.0)))
            width_m = max(0.0, float(obstacle.get("width", 0.0)))
            forward_m, left_m, distance_m = obstacle_relative_geometry(
                obstacle, ego_x, ego_y, ego_yaw
            )
        except (KeyError, TypeError, ValueError):
            continue

        if length_m > max_object_length_m or width_m > max_object_width_m:
            continue
        if forward_m < -rear_allowance_m:
            continue
        if distance_m > detection_distance_m:
            continue

        candidate = dict(obstacle)
        candidate["forward_m"] = float(forward_m)
        candidate["left_m"] = float(left_m)
        candidate["distance_m"] = float(distance_m)
        candidates.append(candidate)

    candidates.sort(key=lambda obstacle: obstacle["distance_m"])
    return candidates


@dataclass(frozen=True)
class PedestrianDecision:
    stop_required: bool
    resume_allowed: bool
    transition: str


class PedestrianStopStateMachine:
    """Latch a stop and require camera and LiDAR clearance before resuming."""

    def __init__(self, stop_confirmation_s=0.2, clear_confirmation_s=1.0):
        self.stop_confirmation_s = float(stop_confirmation_s)
        self.clear_confirmation_s = float(clear_confirmation_s)
        if self.stop_confirmation_s < 0.0:
            raise ValueError("stop_confirmation_s cannot be negative")
        if self.clear_confirmation_s < 0.0:
            raise ValueError("clear_confirmation_s cannot be negative")
        self.stop_required = False
        self.hazard_since = None
        self.clear_since = None

    def update(self, now, inputs_ready, trigger_hazard, clear_for_resume):
        transition = "NONE"

        if not self.stop_required:
            self.clear_since = None
            if inputs_ready and trigger_hazard:
                if self.hazard_since is None:
                    self.hazard_since = now
                if now - self.hazard_since >= self.stop_confirmation_s:
                    self.stop_required = True
                    self.hazard_since = None
                    transition = "STOP"
            else:
                self.hazard_since = None
        else:
            self.hazard_since = None
            if inputs_ready and clear_for_resume:
                if self.clear_since is None:
                    self.clear_since = now
                if now - self.clear_since >= self.clear_confirmation_s:
                    self.stop_required = False
                    self.clear_since = None
                    transition = "RESUME"
            else:
                # Sensor staleness, a camera person, or any nearby LiDAR
                # cluster keeps the stop latched.
                self.clear_since = None

        return PedestrianDecision(
            stop_required=self.stop_required,
            resume_allowed=not self.stop_required,
            transition=transition,
        )
