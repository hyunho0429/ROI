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


def speed_adaptive_steering_profile(
    speed_mps: float,
    minimum_lookahead_m: float,
    lookahead_speed_gain: float,
    maximum_rate_rad_s: float,
    reference_speed_mps: float,
    minimum_rate_rad_s: float,
):
    """Return a stable look-ahead and steering slew rate for a fast manoeuvre.

    A fixed short look-ahead becomes over-responsive when the actual vehicle
    speed is higher than the planned speed.  Grow preview distance with speed
    and reduce steering slew above the reference speed to prevent repeated
    left/right overshoot.
    """
    speed = max(0.0, float(speed_mps))
    reference = max(0.1, float(reference_speed_mps))
    maximum_rate = max(0.0, float(maximum_rate_rad_s))
    minimum_rate = max(0.0, min(float(minimum_rate_rad_s), maximum_rate))
    lookahead = max(
        float(minimum_lookahead_m),
        float(lookahead_speed_gain)*speed,
    )
    rate = maximum_rate*min(1.0, reference/max(speed, reference))
    return lookahead, max(minimum_rate, rate)


def lateral_acceleration_steering_limit(
    speed_mps: float,
    wheelbase_m: float,
    maximum_lateral_accel_mps2: float,
    hardware_limit_rad: float,
) -> float:
    """Return a speed-dependent road-wheel limit for highway control.

    A noisy lane-centre update can make Pure Pursuit request a large recovery
    angle.  At highway speed that reverses yaw too quickly and starts a
    left/right correction cycle.  The bicycle-model relation
    ``a_y = v^2*tan(delta)/wheelbase`` provides the appropriate bound.
    """
    hardware_limit = max(0.0, float(hardware_limit_rad))
    speed = max(0.0, float(speed_mps))
    wheelbase = max(1e-3, float(wheelbase_m))
    maximum_lateral_accel = max(0.0, float(maximum_lateral_accel_mps2))
    if speed < 0.1 or maximum_lateral_accel <= 0.0:
        return hardware_limit
    dynamic_limit = math.atan(maximum_lateral_accel*wheelbase/(speed*speed))
    return min(hardware_limit, dynamic_limit)


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

    def update(
        self,
        target: float,
        now: float,
        enabled: bool = True,
        rate_rad_s: float = None,
    ) -> float:
        dt = self.nominal_dt if self.last_time is None else max(0.0, min(0.1, now-self.last_time))
        self.last_time = now
        if not math.isfinite(target):
            return self.reset(now)
        rate = self.rate if rate_rad_s is None else max(0.0, float(rate_rad_s))
        step = rate*dt
        self.angle = target if rate == 0.0 or not enabled else self.angle + max(-step, min(step, target-self.angle))
        return self.angle
