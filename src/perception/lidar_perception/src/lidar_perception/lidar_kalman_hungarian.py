#!/usr/bin/env python3
"""Constant-velocity Kalman tracking with pure-Python Hungarian matching."""

import math

import numpy as np


def hungarian_assignment(cost_matrix):
    """Return minimum-cost ``(row, column)`` assignments for a rectangle.

    This shortest augmenting-path implementation does not require SciPy.
    It assigns every row when rows <= columns, otherwise it solves the
    transposed problem and converts the result back.
    """

    costs = np.asarray(cost_matrix, dtype=float)
    if costs.ndim != 2:
        raise ValueError("cost_matrix must be two-dimensional")
    row_count, column_count = costs.shape
    if row_count == 0 or column_count == 0:
        return []
    if not np.all(np.isfinite(costs)):
        raise ValueError("cost_matrix values must be finite")

    transposed = row_count > column_count
    working = costs.T if transposed else costs
    rows, columns = working.shape

    row_potential = np.zeros(rows + 1, dtype=float)
    column_potential = np.zeros(columns + 1, dtype=float)
    column_match = np.zeros(columns + 1, dtype=int)
    previous_column = np.zeros(columns + 1, dtype=int)

    for row in range(1, rows + 1):
        column_match[0] = row
        minimum_values = np.full(columns + 1, np.inf, dtype=float)
        used = np.zeros(columns + 1, dtype=bool)
        current_column = 0

        while True:
            used[current_column] = True
            current_row = column_match[current_column]
            delta = np.inf
            next_column = 0

            for column in range(1, columns + 1):
                if used[column]:
                    continue
                reduced_cost = (
                    working[current_row - 1, column - 1]
                    - row_potential[current_row]
                    - column_potential[column]
                )
                if reduced_cost < minimum_values[column]:
                    minimum_values[column] = reduced_cost
                    previous_column[column] = current_column
                if minimum_values[column] < delta:
                    delta = minimum_values[column]
                    next_column = column

            for column in range(columns + 1):
                if used[column]:
                    row_potential[column_match[column]] += delta
                    column_potential[column] -= delta
                else:
                    minimum_values[column] -= delta

            current_column = next_column
            if column_match[current_column] == 0:
                break

        while True:
            previous = previous_column[current_column]
            column_match[current_column] = column_match[previous]
            current_column = previous
            if current_column == 0:
                break

    assignments = []
    for column in range(1, columns + 1):
        row = column_match[column]
        if row == 0:
            continue
        if transposed:
            assignments.append((column - 1, row - 1))
        else:
            assignments.append((row - 1, column - 1))
    assignments.sort()
    return assignments


class _KalmanTrack:
    def __init__(self, track_id, detection, timestamp_s, measurement_noise_m):
        self.track_id = int(track_id)
        self.state = np.array(
            [
                float(detection["center_x_m"]),
                float(detection["center_y_m"]),
                0.0,
                0.0,
            ],
            dtype=float,
        )
        self.covariance = np.diag([1.0, 1.0, 25.0, 25.0]).astype(float)
        self.measurement_noise_m = float(measurement_noise_m)
        self.last_timestamp_s = float(timestamp_s)
        self.hits = 1
        self.misses = 0
        self.age = 1
        self.box = dict(detection)

    def predict(self, timestamp_s, process_accel_std_mps2):
        dt_s = max(1.0e-3, min(0.5, float(timestamp_s) - self.last_timestamp_s))
        transition = np.array(
            [
                [1.0, 0.0, dt_s, 0.0],
                [0.0, 1.0, 0.0, dt_s],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        dt2 = dt_s * dt_s
        dt3 = dt2 * dt_s
        dt4 = dt2 * dt2
        variance = float(process_accel_std_mps2) ** 2
        process_noise = variance * np.array(
            [
                [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
                [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
                [dt3 / 2.0, 0.0, dt2, 0.0],
                [0.0, dt3 / 2.0, 0.0, dt2],
            ],
            dtype=float,
        )
        self.state = transition.dot(self.state)
        self.covariance = (
            transition.dot(self.covariance).dot(transition.T) + process_noise
        )
        self.last_timestamp_s = float(timestamp_s)
        self.age += 1
        self.misses += 1
        self.box["center_x_m"] = float(self.state[0])
        self.box["center_y_m"] = float(self.state[1])

    def update(self, detection):
        observation = np.array(
            [float(detection["center_x_m"]), float(detection["center_y_m"])],
            dtype=float,
        )
        measurement = np.array(
            [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]],
            dtype=float,
        )
        measurement_noise = np.eye(2, dtype=float) * (
            self.measurement_noise_m ** 2
        )
        innovation = observation - measurement.dot(self.state)
        innovation_covariance = (
            measurement.dot(self.covariance).dot(measurement.T)
            + measurement_noise
        )
        gain = self.covariance.dot(measurement.T).dot(
            np.linalg.inv(innovation_covariance)
        )
        self.state = self.state + gain.dot(innovation)
        identity = np.eye(4, dtype=float)
        self.covariance = (identity - gain.dot(measurement)).dot(self.covariance)
        self.hits += 1
        self.misses = 0
        self.box = dict(detection)
        self.box["center_x_m"] = float(self.state[0])
        self.box["center_y_m"] = float(self.state[1])

    def as_dict(self, min_hits):
        result = dict(self.box)
        result.update(
            {
                "track_id": self.track_id,
                "center_x_m": float(self.state[0]),
                "center_y_m": float(self.state[1]),
                "velocity_x_mps": float(self.state[2]),
                "velocity_y_mps": float(self.state[3]),
                "speed_mps": math.hypot(float(self.state[2]), float(self.state[3])),
                "hits": self.hits,
                "misses": self.misses,
                "age": self.age,
                "confirmed": self.hits >= int(min_hits),
            }
        )
        return result


class KalmanHungarianTracker:
    """Multi-object tracker for ego-local AABB detections.

    Velocity is relative to Ego because detections are in ``morai_lidar``.
    Intra-scan motion is deskewed by the UDP source node; world-frame ego
    motion compensation is deliberately outside this standalone demo.
    """

    def __init__(
        self,
        match_distance_m=3.0,
        max_missed=6,
        min_hits=3,
        process_accel_std_mps2=4.0,
        measurement_noise_m=0.35,
    ):
        self.match_distance_m = float(match_distance_m)
        self.max_missed = int(max_missed)
        self.min_hits = int(min_hits)
        self.process_accel_std_mps2 = float(process_accel_std_mps2)
        self.measurement_noise_m = float(measurement_noise_m)
        if self.match_distance_m <= 0.0:
            raise ValueError("match_distance_m must be positive")
        if self.max_missed < 0:
            raise ValueError("max_missed cannot be negative")
        if self.min_hits < 1:
            raise ValueError("min_hits must be at least 1")
        if self.process_accel_std_mps2 <= 0.0:
            raise ValueError("process_accel_std_mps2 must be positive")
        if self.measurement_noise_m <= 0.0:
            raise ValueError("measurement_noise_m must be positive")
        self._tracks = []
        self._next_track_id = 1

    def _new_track(self, detection, timestamp_s):
        track = _KalmanTrack(
            self._next_track_id,
            detection,
            timestamp_s,
            self.measurement_noise_m,
        )
        self._next_track_id += 1
        self._tracks.append(track)

    def update(self, detections, timestamp_s):
        detections = [dict(detection) for detection in detections]
        timestamp_s = float(timestamp_s)

        for track in self._tracks:
            track.predict(timestamp_s, self.process_accel_std_mps2)

        track_count = len(self._tracks)
        detection_count = len(detections)
        matched_detections = set()

        if track_count and detection_count:
            size = track_count + detection_count
            unmatched_cost = self.match_distance_m
            blocked_cost = unmatched_cost * 1000.0
            costs = np.full((size, size), blocked_cost, dtype=float)

            for track_index, track in enumerate(self._tracks):
                for detection_index, detection in enumerate(detections):
                    distance_m = math.hypot(
                        float(track.state[0]) - float(detection["center_x_m"]),
                        float(track.state[1]) - float(detection["center_y_m"]),
                    )
                    if distance_m <= self.match_distance_m:
                        costs[track_index, detection_index] = distance_m
                costs[track_index, detection_count + track_index] = unmatched_cost

            for detection_index in range(detection_count):
                dummy_row = track_count + detection_index
                costs[dummy_row, detection_index] = unmatched_cost
                costs[dummy_row, detection_count:] = 0.0

            for track_index, detection_index in hungarian_assignment(costs):
                if track_index >= track_count or detection_index >= detection_count:
                    continue
                if costs[track_index, detection_index] > self.match_distance_m:
                    continue
                self._tracks[track_index].update(detections[detection_index])
                matched_detections.add(detection_index)

        for detection_index, detection in enumerate(detections):
            if detection_index not in matched_detections:
                self._new_track(detection, timestamp_s)

        self._tracks = [
            track for track in self._tracks if track.misses <= self.max_missed
        ]
        return [track.as_dict(self.min_hits) for track in self._tracks]
