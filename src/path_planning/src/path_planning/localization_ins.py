"""3-D strapdown INS with a 15-state error-state Kalman filter."""

import math

import numpy as np

from path_planning.inertial_math import (
    apply_body_rotation,
    normalize_quaternion,
    quaternion_error,
    quaternion_to_matrix,
    quaternion_yaw,
    skew,
)
from path_planning.localization import LocalizationState


GRAVITY_ENU_MPS2 = np.array((0.0, 0.0, -9.80665), dtype=float)


class InsErrorStateEkf:
    """Loosely coupled GPS/IMU/vehicle-speed ESKF in map-origin ENU.

    Nominal state: position, velocity, body-to-ENU quaternion, gyro bias and
    accelerometer bias.  Error state: [dp, dv, dtheta, dbg, dba].
    """

    def __init__(
        self,
        gps_position_sigma_m=1.5,
        gps_altitude_sigma_m=3.0,
        gps_speed_sigma_mps=0.8,
        imu_orientation_sigma_deg=4.0,
        gyro_noise_sigma_degps=0.8,
        accel_noise_sigma_mps2=0.25,
        gyro_bias_walk_sigma_degps=0.03,
        accel_bias_walk_sigma_mps2=0.02,
        vehicle_speed_sigma_mps=0.25,
        nhc_lateral_sigma_mps=0.35,
        nhc_vertical_sigma_mps=0.25,
        gps_gate_sigma=6.0,
        alignment_duration_s=0.0,
        alignment_min_samples=1,
    ):
        self.gps_position_sigma_m = float(gps_position_sigma_m)
        self.gps_altitude_sigma_m = float(gps_altitude_sigma_m)
        self.gps_speed_sigma_mps = float(gps_speed_sigma_mps)
        self.imu_orientation_sigma_rad = math.radians(imu_orientation_sigma_deg)
        self.gyro_noise_sigma_radps = math.radians(gyro_noise_sigma_degps)
        self.accel_noise_sigma_mps2 = float(accel_noise_sigma_mps2)
        self.gyro_bias_walk_sigma_radps = math.radians(gyro_bias_walk_sigma_degps)
        self.accel_bias_walk_sigma_mps2 = float(accel_bias_walk_sigma_mps2)
        self.vehicle_speed_sigma_mps = float(vehicle_speed_sigma_mps)
        self.nhc_lateral_sigma_mps = float(nhc_lateral_sigma_mps)
        self.nhc_vertical_sigma_mps = float(nhc_vertical_sigma_mps)
        self.gps_gate_sigma = float(gps_gate_sigma)
        self.alignment_duration_s = max(0.0, float(alignment_duration_s))
        self.alignment_min_samples = max(1, int(alignment_min_samples))

        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.array((0.0, 0.0, 0.0, 1.0))
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        initial_variance = np.array(
            (100.0, 100.0, 25.0, 25.0, 25.0, 9.0, 1.0, 1.0, 1.0,
             0.05, 0.05, 0.05, 0.5, 0.5, 0.5),
            dtype=float,
        )
        self.covariance = np.diag(initial_variance)
        self.timestamp = None
        self.last_gyro = np.zeros(3)
        self.last_specific_force = np.array((0.0, 0.0, 9.80665))
        self.last_vehicle_speed_mps = None
        self.position_initialized = False
        self.orientation_initialized = False
        self.alignment_complete = self.alignment_duration_s <= 0.0
        self.alignment_start_timestamp = None
        self._alignment_gyro_samples = []
        self._alignment_force_samples = []

    @property
    def ready(self):
        return (
            self.position_initialized
            and self.orientation_initialized
            and self.alignment_complete
        )

    @property
    def alignment_sample_count(self):
        return len(self._alignment_gyro_samples)

    def _predict_to(self, timestamp):
        timestamp = float(timestamp)
        if not self.alignment_complete:
            self.timestamp = timestamp
            return
        if self.timestamp is None:
            self.timestamp = timestamp
            return
        remaining = timestamp - self.timestamp
        if remaining <= 0.0:
            return
        while remaining > 1e-9:
            dt = min(remaining, 0.02)
            self._predict_step(dt)
            remaining -= dt
        self.timestamp = timestamp

    def _predict_step(self, dt):
        corrected_gyro = self.last_gyro - self.gyro_bias
        corrected_force = self.last_specific_force - self.accel_bias
        rotation = quaternion_to_matrix(self.orientation)
        acceleration_enu = rotation @ corrected_force + GRAVITY_ENU_MPS2
        self.position += self.velocity * dt + 0.5 * acceleration_enu * dt * dt
        self.velocity += acceleration_enu * dt
        self.orientation = apply_body_rotation(
            self.orientation, corrected_gyro * dt
        )

        transition_rate = np.zeros((15, 15))
        transition_rate[0:3, 3:6] = np.eye(3)
        transition_rate[3:6, 6:9] = -rotation @ skew(corrected_force)
        transition_rate[3:6, 12:15] = -rotation
        transition_rate[6:9, 6:9] = -skew(corrected_gyro)
        transition_rate[6:9, 9:12] = -np.eye(3)
        transition = np.eye(15) + transition_rate * dt

        noise_map = np.zeros((15, 12))
        noise_map[3:6, 0:3] = rotation
        noise_map[6:9, 3:6] = -np.eye(3)
        noise_map[9:12, 6:9] = np.eye(3)
        noise_map[12:15, 9:12] = np.eye(3)
        noise_variance = np.diag(
            [self.accel_noise_sigma_mps2 ** 2] * 3
            + [self.gyro_noise_sigma_radps ** 2] * 3
            + [self.gyro_bias_walk_sigma_radps ** 2] * 3
            + [self.accel_bias_walk_sigma_mps2 ** 2] * 3
        )
        process_noise = noise_map @ noise_variance @ noise_map.T * dt
        self.covariance = (
            transition @ self.covariance @ transition.T + process_noise
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)

    def _update(self, innovation, observation, measurement_covariance):
        innovation = np.asarray(innovation, dtype=float).reshape(-1)
        observation = np.asarray(observation, dtype=float)
        measurement_covariance = np.asarray(measurement_covariance, dtype=float)
        residual_covariance = (
            observation @ self.covariance @ observation.T
            + measurement_covariance
        )
        gain = self.covariance @ observation.T @ np.linalg.inv(residual_covariance)
        correction = gain @ innovation
        identity = np.eye(15)
        remainder = identity - gain @ observation
        self.covariance = (
            remainder @ self.covariance @ remainder.T
            + gain @ measurement_covariance @ gain.T
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.position += correction[0:3]
        self.velocity += correction[3:6]
        self.orientation = apply_body_rotation(
            self.orientation, correction[6:9]
        )
        self.gyro_bias += correction[9:12]
        self.accel_bias += correction[12:15]

    def add_imu(
        self,
        timestamp,
        orientation_xyzw,
        angular_velocity_radps,
        linear_acceleration_mps2,
    ):
        measured_orientation = normalize_quaternion(orientation_xyzw)
        measured_gyro = np.asarray(angular_velocity_radps, dtype=float).reshape(3)
        measured_force = np.asarray(
            linear_acceleration_mps2, dtype=float
        ).reshape(3)
        if not self.alignment_complete:
            timestamp = float(timestamp)
            if self.alignment_start_timestamp is None:
                self.alignment_start_timestamp = timestamp
            self.orientation = measured_orientation
            self.orientation_initialized = True
            self.timestamp = timestamp
            self.last_gyro = measured_gyro
            self.last_specific_force = measured_force
            self._alignment_gyro_samples.append(measured_gyro.copy())
            self._alignment_force_samples.append(measured_force.copy())
            elapsed = timestamp - self.alignment_start_timestamp
            if (
                elapsed >= self.alignment_duration_s
                and self.alignment_sample_count >= self.alignment_min_samples
            ):
                self.gyro_bias = np.mean(
                    np.vstack(self._alignment_gyro_samples), axis=0
                )
                mean_force = np.mean(
                    np.vstack(self._alignment_force_samples), axis=0
                )
                rotation = quaternion_to_matrix(self.orientation)
                expected_specific_force = rotation.T @ (-GRAVITY_ENU_MPS2)
                self.accel_bias = mean_force - expected_specific_force
                self.covariance[9:12, 9:12] = np.eye(3) * 1e-4
                self.covariance[12:15, 12:15] = np.eye(3) * 1e-2
                self.alignment_complete = True
            return
        if not self.orientation_initialized:
            self.orientation = measured_orientation
            self.covariance[6:9, 6:9] = (
                np.eye(3) * self.imu_orientation_sigma_rad ** 2
            )
            self.orientation_initialized = True
            self.timestamp = float(timestamp)
        else:
            self._predict_to(timestamp)
            attitude_innovation = quaternion_error(
                measured_orientation, self.orientation
            )
            observation = np.zeros((3, 15))
            observation[:, 6:9] = np.eye(3)
            variance = self.imu_orientation_sigma_rad ** 2
            self._update(attitude_innovation, observation, np.eye(3) * variance)
        self.last_gyro = measured_gyro
        self.last_specific_force = measured_force
        if (
            self.last_vehicle_speed_mps is not None
            and abs(self.last_vehicle_speed_mps) < 0.08
        ):
            rotation = quaternion_to_matrix(self.orientation)
            expected_specific_force = rotation.T @ (-GRAVITY_ENU_MPS2)
            target_accel_bias = self.last_specific_force - expected_specific_force
            self.accel_bias += 0.01 * (target_accel_bias - self.accel_bias)
            self.gyro_bias += 0.01 * (self.last_gyro - self.gyro_bias)

    def add_gps(
        self,
        timestamp,
        x_m,
        y_m,
        z_m=None,
        speed_mps=None,
        course_deg=None,
    ):
        self._predict_to(timestamp)
        measured = np.array((float(x_m), float(y_m), float(z_m or 0.0)))
        dimensions = 3 if z_m is not None and math.isfinite(z_m) else 2
        if not self.position_initialized:
            self.position[:dimensions] = measured[:dimensions]
            self.covariance[0, 0] = self.gps_position_sigma_m ** 2
            self.covariance[1, 1] = self.gps_position_sigma_m ** 2
            if dimensions == 3:
                self.covariance[2, 2] = self.gps_altitude_sigma_m ** 2
            self.position_initialized = True
        else:
            observation = np.zeros((dimensions, 15))
            observation[:, :dimensions] = np.eye(dimensions)
            sigmas = [self.gps_position_sigma_m] * 2
            if dimensions == 3:
                sigmas.append(self.gps_altitude_sigma_m)
            measurement_covariance = np.diag(np.square(sigmas))
            innovation = measured[:dimensions] - self.position[:dimensions]
            residual_covariance = (
                observation @ self.covariance @ observation.T
                + measurement_covariance
            )
            normalized_error = float(
                innovation.T @ np.linalg.solve(residual_covariance, innovation)
            )
            if normalized_error > self.gps_gate_sigma ** 2 * dimensions:
                return False
            self._update(innovation, observation, measurement_covariance)

        if speed_mps is not None and course_deg is not None:
            course_yaw = math.radians(90.0 - float(course_deg))
            measured_velocity = np.array(
                (
                    float(speed_mps) * math.cos(course_yaw),
                    float(speed_mps) * math.sin(course_yaw),
                )
            )
            observation = np.zeros((2, 15))
            observation[:, 3:5] = np.eye(2)
            self._update(
                measured_velocity - self.velocity[:2],
                observation,
                np.eye(2) * self.gps_speed_sigma_mps ** 2,
            )
        return True

    def add_vehicle_speed(self, timestamp, signed_speed_mps):
        self.last_vehicle_speed_mps = float(signed_speed_mps)
        if not self.alignment_complete:
            self.timestamp = float(timestamp)
            return
        self._predict_to(timestamp)
        rotation = quaternion_to_matrix(self.orientation)
        body_axes = rotation
        observations = []
        innovations = []
        variances = []
        for axis, measured, sigma in (
            (0, self.last_vehicle_speed_mps, self.vehicle_speed_sigma_mps),
            (1, 0.0, self.nhc_lateral_sigma_mps),
            (2, 0.0, self.nhc_vertical_sigma_mps),
        ):
            direction = body_axes[:, axis]
            row = np.zeros(15)
            row[3:6] = direction
            predicted = float(direction @ self.velocity)
            observations.append(row)
            innovations.append(measured - predicted)
            variances.append(sigma ** 2)
        self._update(
            innovations, np.vstack(observations), np.diag(variances)
        )

    def state_at(self, timestamp):
        self._predict_to(timestamp)
        if not self.ready:
            return None
        return LocalizationState(
            x_m=float(self.position[0]),
            y_m=float(self.position[1]),
            z_m=float(self.position[2]),
            yaw_rad=quaternion_yaw(self.orientation),
            speed_mps=float(np.linalg.norm(self.velocity)),
            timestamp=self.timestamp,
        )
