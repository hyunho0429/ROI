"""PID speed controller for competition-required MORAI pedal control."""

import math


class PedalSpeedController:
    def __init__(
        self,
        kp=0.075,
        ki=0.0001,
        kd=0.025,
        nominal_dt=1.0 / 30.0,
        deadband_mps=0.0,
        integral_limit=8.0,
        max_accel=1.0,
        max_brake=1.0,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.nominal_dt = float(nominal_dt)
        self.deadband_mps = float(deadband_mps)
        self.integral_limit = float(integral_limit)
        self.max_accel = float(max_accel)
        self.max_brake = float(max_brake)
        if self.nominal_dt <= 0.0:
            raise ValueError("nominal_dt must be positive")
        self._integral = 0.0
        self._last_timestamp = None
        self._previous_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._last_timestamp = None
        self._previous_error = 0.0

    def compute(self, target_speed_mps, measured_speed_mps, timestamp):
        values = (target_speed_mps, measured_speed_mps, timestamp)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("speed controller inputs must be finite")
        target = max(0.0, float(target_speed_mps))
        measured = max(0.0, float(measured_speed_mps))
        timestamp = float(timestamp)

        dt = self.nominal_dt
        if self._last_timestamp is not None:
            elapsed = timestamp - self._last_timestamp
            if elapsed > 0.0:
                dt = min(0.2, elapsed)
        self._last_timestamp = timestamp

        error = target - measured
        if abs(error) <= self.deadband_mps:
            error = 0.0
        self._integral = max(
            -self.integral_limit,
            min(self.integral_limit, self._integral + error * dt),
        )
        derivative = (error - self._previous_error) / dt
        self._previous_error = error
        effort = (
            self.kp * error
            + self.ki * self._integral
            + self.kd * derivative
        )
        if effort >= 0.0:
            return min(self.max_accel, effort), 0.0
        return 0.0, min(self.max_brake, -effort)
