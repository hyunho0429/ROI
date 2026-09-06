"""Pure intersection-detection logic shared by the ROS node and tests."""

from dataclasses import dataclass


@dataclass(frozen=True)
class IntersectionDecision:
    state: str
    detected: bool
    driving_allowed: bool
    driving_unavailable: bool


class IntersectionStateMachine:
    """Recognize ``Car AND left solid AND right solid`` intersections.

    After recognition, the camera vehicle state controls STOP/GO. Stale
    camera data can never release a blocked intersection.
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
        left_solid_lane_detected: bool,
        right_solid_lane_detected: bool,
        now: float,
        camera_fresh: bool = True,
        lane_fresh: bool = True,
    ) -> IntersectionDecision:
        now = float(now)
        recognition_conditions_met = bool(
            camera_fresh
            and lane_fresh
            and camera_vehicle_detected
            and left_solid_lane_detected
            and right_solid_lane_detected
        )

        if self.state == "IDLE":
            if recognition_conditions_met:
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
            if recognition_conditions_met:
                self.state = "BLOCKED"
                self.camera_clear_since = None
                self.clear_started_at = None
            elif (
                self.clear_started_at is not None
                and now - self.clear_started_at >= self.clear_hold_s
            ):
                self.state = "IDLE"
                self.clear_started_at = None

        return IntersectionDecision(
            state=self.state,
            detected=self.state != "IDLE",
            driving_allowed=self.state == "CLEAR",
            driving_unavailable=self.state == "BLOCKED",
        )
