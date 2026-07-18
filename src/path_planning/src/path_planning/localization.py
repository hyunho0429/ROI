"""Compact GPS/IMU planar EKF for MORAI path tracking."""

import math
from dataclasses import dataclass


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _matmul(left, right):
    columns = len(right[0])
    return [
        [sum(left[i][k] * right[k][j] for k in range(len(right))) for j in range(columns)]
        for i in range(len(left))
    ]


def _transpose(matrix):
    return [list(row) for row in zip(*matrix)]


@dataclass(frozen=True)
class LocalizationState:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    speed_mps: float
    timestamp: float


class PlanarGpsImuEkf:
    """Fuse noisy GPS position/ground speed with IMU yaw and yaw rate.

    State: [x, y, vx, vy, yaw, gyro_z_bias].  This deliberately avoids the
    old full Earth-frame strapdown mechanization: MORAI already supplies an
    absolute quaternion, while Stanley only needs horizontal pose and speed.
    """

    def __init__(
        self,
        gps_position_sigma_m=1.5,
        gps_speed_sigma_mps=0.8,
        imu_yaw_sigma_deg=3.0,
        process_accel_sigma_mps2=2.0,
        gyro_noise_sigma_degps=1.0,
        gyro_bias_walk_sigma_degps=0.05,
        gps_outlier_threshold_m=15.0,
    ):
        self.gps_position_variance = float(gps_position_sigma_m) ** 2
        self.gps_speed_variance = float(gps_speed_sigma_mps) ** 2
        self.imu_yaw_variance = math.radians(float(imu_yaw_sigma_deg)) ** 2
        self.process_accel_variance = float(process_accel_sigma_mps2) ** 2
        self.gyro_noise_variance = math.radians(float(gyro_noise_sigma_degps)) ** 2
        self.bias_walk_variance = math.radians(float(gyro_bias_walk_sigma_degps)) ** 2
        self.gps_outlier_threshold_m = float(gps_outlier_threshold_m)
        self._state = [0.0] * 6
        self._covariance = [
            [100.0 if i == j and i < 2 else 25.0 if i == j and i < 4 else 1.0 if i == j else 0.0
             for j in range(6)]
            for i in range(6)
        ]
        self._timestamp = None
        self._last_gyro_z = 0.0
        self._z_m = 0.0
        self._z_initialized = False
        self._position_initialized = False
        self._yaw_initialized = False

    @property
    def ready(self):
        return self._position_initialized and self._yaw_initialized and self._timestamp is not None

    def _predict(self, timestamp):
        timestamp = float(timestamp)
        if self._timestamp is None:
            self._timestamp = timestamp
            return
        dt = timestamp - self._timestamp
        if dt <= 0.0:
            return
        # Long pauses are treated as a restart rather than integrated blindly.
        dt = min(dt, 0.25)
        x, y, vx, vy, yaw, bias = self._state
        self._state = [
            x + vx * dt,
            y + vy * dt,
            vx,
            vy,
            wrap_angle(yaw + (self._last_gyro_z - bias) * dt),
            bias,
        ]
        transition = [[1.0 if i == j else 0.0 for j in range(6)] for i in range(6)]
        transition[0][2] = dt
        transition[1][3] = dt
        transition[4][5] = -dt
        covariance = _matmul(_matmul(transition, self._covariance), _transpose(transition))
        accel_q = self.process_accel_variance
        process_diagonal = (
            0.25 * dt ** 4 * accel_q,
            0.25 * dt ** 4 * accel_q,
            dt * dt * accel_q,
            dt * dt * accel_q,
            dt * dt * self.gyro_noise_variance,
            dt * self.bias_walk_variance,
        )
        for index, value in enumerate(process_diagonal):
            covariance[index][index] += value
        self._covariance = covariance
        self._timestamp = timestamp

    def _update_direct(self, state_index, measurement, variance, angle=False):
        innovation = float(measurement) - self._state[state_index]
        if angle:
            innovation = wrap_angle(innovation)
        innovation_variance = self._covariance[state_index][state_index] + variance
        if innovation_variance <= 0.0:
            return
        source_row = list(self._covariance[state_index])
        gain = [self._covariance[i][state_index] / innovation_variance for i in range(6)]
        for i in range(6):
            self._state[i] += gain[i] * innovation
            for j in range(6):
                self._covariance[i][j] -= gain[i] * source_row[j]
        self._state[4] = wrap_angle(self._state[4])
        # Round-off can make an otherwise symmetric covariance slightly asymmetric.
        for i in range(6):
            for j in range(i + 1, 6):
                value = 0.5 * (self._covariance[i][j] + self._covariance[j][i])
                self._covariance[i][j] = value
                self._covariance[j][i] = value

    def add_imu(self, timestamp, yaw_rad, gyro_z_radps):
        self._predict(timestamp)
        if not self._yaw_initialized:
            self._state[4] = wrap_angle(yaw_rad)
            self._covariance[4][4] = self.imu_yaw_variance
            self._yaw_initialized = True
        else:
            self._update_direct(4, yaw_rad, self.imu_yaw_variance, angle=True)
        self._last_gyro_z = float(gyro_z_radps)

    def add_gps(self, timestamp, x_m, y_m, z_m=None, speed_mps=None, course_deg=None):
        self._predict(timestamp)
        if not self._position_initialized:
            self._state[0] = float(x_m)
            self._state[1] = float(y_m)
            self._covariance[0][0] = self.gps_position_variance
            self._covariance[1][1] = self.gps_position_variance
            self._position_initialized = True
        else:
            if math.hypot(float(x_m) - self._state[0], float(y_m) - self._state[1]) > self.gps_outlier_threshold_m:
                return False
            self._update_direct(0, x_m, self.gps_position_variance)
            self._update_direct(1, y_m, self.gps_position_variance)

        if z_m is not None and math.isfinite(z_m):
            if not self._z_initialized:
                self._z_m = float(z_m)
                self._z_initialized = True
            else:
                self._z_m += 0.15 * (float(z_m) - self._z_m)

        if speed_mps is not None and math.isfinite(speed_mps):
            speed_mps = max(0.0, float(speed_mps))
            if speed_mps < 0.2 or course_deg is None:
                measured_vx = measured_vy = 0.0
            else:
                # NMEA course: clockwise from north. ENU yaw: CCW from east.
                course_yaw = math.radians(90.0 - float(course_deg))
                measured_vx = speed_mps * math.cos(course_yaw)
                measured_vy = speed_mps * math.sin(course_yaw)
            self._update_direct(2, measured_vx, self.gps_speed_variance)
            self._update_direct(3, measured_vy, self.gps_speed_variance)
        return True

    def state_at(self, timestamp):
        self._predict(timestamp)
        if not self.ready:
            return None
        return LocalizationState(
            x_m=self._state[0],
            y_m=self._state[1],
            z_m=self._z_m,
            yaw_rad=wrap_angle(self._state[4]),
            speed_mps=math.hypot(self._state[2], self._state[3]),
            timestamp=self._timestamp,
        )
