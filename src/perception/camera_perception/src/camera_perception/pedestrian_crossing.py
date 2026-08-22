"""Pure camera-triggered pedestrian stop state-machine logic."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PedestrianDecision:
    stop_required: bool
    resume_allowed: bool
    transition: str


class PedestrianStopStateMachine:
    """Latch a stop and require a fresh, continuously clear camera to resume."""

    def __init__(self, stop_confirmation_s=0.0, clear_confirmation_s=0.5):
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
                # Sensor staleness or a detected person keeps the stop latched.
                self.clear_since = None

        return PedestrianDecision(
            stop_required=self.stop_required,
            resume_allowed=not self.stop_required,
            transition=transition,
        )
