"""Vehicle-speed-aided 3-D dead reckoning with GPS position correction."""

import math

import numpy as np

from path_planning.inertial_math import (
    apply_body_rotation,
    normalize_quaternion,
    quaternion_error,
    quaternion_to_matrix,
    quaternion_yaw,
)
from path_planning.localization import LocalizationState


class SpeedAidedDeadReckoning:
    """Integrate forward vehicle speed along the IMU attitude direction."""

    def __init__(
        self,
        gps_position_sigma_m=1.5,
        gps_altitude_sigma_m=3.0,
        position_drift_sigma_mps=0.25,
        orientation_correction_gain=0.12,
        gyro_bias_gain=0.002,
        gps_gate_sigma=8.0,
    ):
        self.gps_position_sigma_m = float(gps_position_sigma_m)
        self.gps_altitude_sigma_m = float(gps_altitude_sigma_m)
        self.position_drift_sigma_mps = float(position_drift_sigma_mps)
        self.orientation_correction_gain = float(orientation_correction_gain)
        self.gyro_bias_gain = float(gyro_bias_gain)
        self.gps_gate_sigma = float(gps_gate_sigma)
        self.position = np.zeros(3)
        self.position_covariance = np.diag((100.0, 100.0, 25.0))
        self.orientation = np.array((0.0, 0.0, 0.0, 1.0))
        self.gyro_bias = np.zeros(3)
        self.last_gyro = np.zeros(3)
        self.signed_speed_mps = 0.0
        self.timestamp = None
        self.position_initialized = False
        self.orientation_initialized = False
        self.speed_initialized = False

    @property
    def ready(self):
        return self.position_initialized and self.orientation_initialized

    def _predict_to(self, timestamp):
        timestamp = float(timestamp)
        if self.timestamp is None:
            self.timestamp = timestamp
            return
        remaining = timestamp - self.timestamp
        if remaining <= 0.0:
            return
        while remaining > 1e-9:
            dt = min(remaining, 0.02)
            corrected_gyro = self.last_gyro - self.gyro_bias
            self.orientation = apply_body_rotation(
                self.orientation, corrected_gyro * dt
            )
            forward_enu = quaternion_to_matrix(self.orientation)[:, 0]
            self.position += forward_enu * self.signed_speed_mps * dt
            drift_variance = self.position_drift_sigma_mps ** 2 * dt
            self.position_covariance += np.eye(3) * drift_variance
            remaining -= dt
        self.timestamp = timestamp

    def add_imu(
        self,
        timestamp,
        orientation_xyzw,
        angular_velocity_radps,
        _linear_acceleration_mps2,
    ):
        measured_orientation = normalize_quaternion(orientation_xyzw)
        if not self.orientation_initialized:
            self.orientation = measured_orientation
            self.orientation_initialized = True
            self.timestamp = float(timestamp)
        else:
            previous_timestamp = self.timestamp
            self._predict_to(timestamp)
            error = quaternion_error(measured_orientation, self.orientation)
            self.orientation = apply_body_rotation(
                self.orientation, self.orientation_correction_gain * error
            )
            dt = max(1e-3, float(timestamp) - float(previous_timestamp))
            self.gyro_bias -= self.gyro_bias_gain * error / dt
            self.gyro_bias = np.clip(self.gyro_bias, -0.2, 0.2)
        self.last_gyro = np.asarray(angular_velocity_radps, dtype=float).reshape(3)

    def add_vehicle_speed(self, timestamp, signed_speed_mps):
        self._predict_to(timestamp)
        measured = float(signed_speed_mps)
        if not self.speed_initialized:
            self.signed_speed_mps = measured
            self.speed_initialized = True
        else:
            self.signed_speed_mps += 0.45 * (measured - self.signed_speed_mps)

    def add_gps(
        self,
        timestamp,
        x_m,
        y_m,
        z_m=None,
        speed_mps=None,
        course_deg=None,
    ):
        del course_deg
        self._predict_to(timestamp)
        dimensions = 3 if z_m is not None and math.isfinite(z_m) else 2
        measured = np.array((float(x_m), float(y_m), float(z_m or 0.0)))
        if not self.position_initialized:
            self.position[:dimensions] = measured[:dimensions]
            initial_sigmas = np.array(
                (self.gps_position_sigma_m, self.gps_position_sigma_m,
                 self.gps_altitude_sigma_m)
            )
            for index in range(dimensions):
                self.position_covariance[index, index] = initial_sigmas[index] ** 2
            self.position_initialized = True
        else:
            sigmas = np.array(
                (self.gps_position_sigma_m, self.gps_position_sigma_m,
                 self.gps_altitude_sigma_m)
            )[:dimensions]
            measurement_covariance = np.diag(sigmas ** 2)
            covariance = self.position_covariance[:dimensions, :dimensions]
            innovation = measured[:dimensions] - self.position[:dimensions]
            residual_covariance = covariance + measurement_covariance
            normalized_error = float(
                innovation.T @ np.linalg.solve(residual_covariance, innovation)
            )
            if normalized_error > self.gps_gate_sigma ** 2 * dimensions:
                return False
            gain = covariance @ np.linalg.inv(residual_covariance)
            self.position[:dimensions] += gain @ innovation
            identity = np.eye(dimensions)
            corrected = (
                (identity - gain) @ covariance @ (identity - gain).T
                + gain @ measurement_covariance @ gain.T
            )
            self.position_covariance[:dimensions, :dimensions] = corrected
        if not self.speed_initialized and speed_mps is not None:
            self.signed_speed_mps = max(0.0, float(speed_mps))
            self.speed_initialized = True
        return True

    def state_at(self, timestamp):
        self._predict_to(timestamp)
        if not self.ready:
            return None
        return LocalizationState(
            x_m=float(self.position[0]),
            y_m=float(self.position[1]),
            z_m=float(self.position[2]),
            yaw_rad=quaternion_yaw(self.orientation),
            speed_mps=abs(self.signed_speed_mps),
            timestamp=self.timestamp,
        )
