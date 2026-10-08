"""Shared actuator steering limits (no planning or stop suppression)."""
import math


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
