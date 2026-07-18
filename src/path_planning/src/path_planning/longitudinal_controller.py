"""Small PI speed controller for competition-required pedal control."""

import math


class PedalSpeedController:
    def __init__(
        self,
        kp=0.12,
        ki=0.04,
        deadband_mps=0.15,
        integral_limit=8.0,
        max_accel=0.65,
        max_brake=0.8,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.deadband_mps = float(deadband_mps)
        self.integral_limit = float(integral_limit)
        self.max_accel = float(max_accel)
        self.max_brake = float(max_brake)
        self._integral = 0.0
        self._last_timestamp = None

    def reset(self):
        self._integral = 0.0
        self._last_timestamp = None

    def compute(self, target_speed_mps, measured_speed_mps, timestamp):
        values = (target_speed_mps, measured_speed_mps, timestamp)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("speed controller inputs must be finite")
        target = max(0.0, float(target_speed_mps))
        measured = max(0.0, float(measured_speed_mps))
        timestamp = float(timestamp)

        dt = 0.0
        if self._last_timestamp is not None:
            dt = max(0.0, min(0.2, timestamp - self._last_timestamp))
        self._last_timestamp = timestamp

        error = target - measured
        if abs(error) <= self.deadband_mps:
            error = 0.0
        self._integral = max(
            -self.integral_limit,
            min(self.integral_limit, self._integral + error * dt),
        )
        effort = self.kp * error + self.ki * self._integral
        if effort >= 0.0:
            return min(self.max_accel, effort), 0.0
        return 0.0, min(self.max_brake, -effort)
