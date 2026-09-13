"""Continuous lateral progress and steering command limits (no ROS required)."""
import math


def diagonal_progress(u: float, ramp: float = 0.2) -> float:
    """C2 lateral shift: eased entry/exit and constant slope in the middle.

    The maximum normalized slope is 1/(1-ramp), versus 1.875 for a
    quintic smoothstep. The middle 60% is diagonal when ramp=0.2.
    """
    if not 0.0 < ramp < 0.5:
        raise ValueError("ramp must be between 0 and 0.5")
    u = max(0.0, min(1.0, u))
    if u > 1.0-ramp:
        return 1.0-diagonal_progress(1.0-u, ramp)
    if u < ramp:
        s = u/ramp
        return ramp/(1.0-ramp) * (s**3-0.5*s**4)
    return (u-0.5*ramp)/(1.0-ramp)


class SteeringRateLimiter:
    """Limit road-wheel command changes in rad/s, independent of frame rate."""
    def __init__(self, rate_rad_s: float, nominal_dt: float = 0.05):
        self.rate = max(0.0, float(rate_rad_s))
        self.nominal_dt = nominal_dt
        self.angle = 0.0
        self.last_time = None

    def reset(self, now: float) -> float:
        self.angle = 0.0
        self.last_time = now
        return self.angle

    def update(self, target: float, now: float, enabled: bool = True) -> float:
        dt = self.nominal_dt if self.last_time is None else max(0.0, min(0.1, now-self.last_time))
        self.last_time = now
        if not math.isfinite(target):
            return self.reset(now)
        step = self.rate*dt
        self.angle = target if self.rate == 0.0 or not enabled else self.angle + max(-step, min(step, target-self.angle))
        return self.angle
