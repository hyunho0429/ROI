"""Distance-aware stop-line approach logic without ROS dependencies."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StoplineApproachDecision:
    armed: bool
    distance_m: float
    target_speed_mps: float
    full_stop: bool
    reason: str


class StoplineApproachController:
    """Use a stop line only while a traffic/intersection stop is requested."""

    def __init__(
        self,
        target_distance_m=2.0,
        comfortable_decel_mps2=1.5,
        stale_timeout_s=0.5,
        maximum_detection_distance_m=40.0,
    ):
        self.target_distance_m = float(target_distance_m)
        self.comfortable_decel_mps2 = float(comfortable_decel_mps2)
        self.stale_timeout_s = float(stale_timeout_s)
        self.maximum_detection_distance_m = float(maximum_detection_distance_m)
        if self.target_distance_m < 0.0:
            raise ValueError("target_distance_m must be non-negative")
        if self.comfortable_decel_mps2 <= 0.0:
            raise ValueError("comfortable_decel_mps2 must be positive")
        if self.stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be positive")
        if self.maximum_detection_distance_m <= self.target_distance_m:
            raise ValueError("maximum_detection_distance_m must exceed target distance")
        self.last_distance_m = None
        self.last_distance_at = None
        self.has_seen_stopline = False

    def reset(self):
        self.last_distance_m = None
        self.last_distance_at = None
        self.has_seen_stopline = False

    def observe_distance(self, distance_m, timestamp_sec):
        try:
            distance = float(distance_m)
            timestamp = float(timestamp_sec)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(distance) or not math.isfinite(timestamp):
            return False
        if distance < 0.0 or distance > self.maximum_detection_distance_m:
            return False
        self.last_distance_m = distance
        self.last_distance_at = timestamp
        self.has_seen_stopline = True
        return True

    def update(self, stop_trigger, cruise_speed_mps, timestamp_sec):
        cruise_speed = max(0.0, float(cruise_speed_mps))
        now = float(timestamp_sec)
        if not bool(stop_trigger):
            self.reset()
            return StoplineApproachDecision(
                armed=False,
                distance_m=float("nan"),
                target_speed_mps=cruise_speed,
                full_stop=False,
                reason="INACTIVE",
            )

        if not self.has_seen_stopline:
            return StoplineApproachDecision(
                armed=True,
                distance_m=float("nan"),
                target_speed_mps=cruise_speed,
                full_stop=False,
                reason="WAITING_FOR_STOPLINE",
            )

        if self.last_distance_at is None or now - self.last_distance_at > self.stale_timeout_s:
            return StoplineApproachDecision(
                armed=True,
                distance_m=float(self.last_distance_m),
                target_speed_mps=0.0,
                full_stop=True,
                reason="STOPLINE_STALE",
            )

        remaining = max(0.0, self.last_distance_m - self.target_distance_m)
        target_speed = min(
            cruise_speed,
            math.sqrt(2.0 * self.comfortable_decel_mps2 * remaining),
        )
        full_stop = remaining <= 0.0
        return StoplineApproachDecision(
            armed=True,
            distance_m=float(self.last_distance_m),
            target_speed_mps=0.0 if full_stop else target_speed,
            full_stop=full_stop,
            reason="TARGET_REACHED" if full_stop else "APPROACHING",
        )
