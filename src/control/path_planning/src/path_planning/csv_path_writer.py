#!/usr/bin/env python3
"""Spatial resampling and durable CSV writing helpers."""

import csv
import math
import os


CSV_FIELDNAMES = [
    "sequence",
    "recorded_at_utc",
    "receive_time_sec",
    "message_time_sec",
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "projection_crs",
    "east_offset_m",
    "north_offset_m",
    "up_offset_m",
    "global_enu_x_m",
    "global_enu_y_m",
    "global_enu_z_m",
    "roll_deg",
    "pitch_deg",
    "heading_deg",
    "velocity_x_mps",
    "velocity_y_mps",
    "velocity_z_mps",
    "speed_mps",
    "signed_speed_mps",
    "ctrl_mode",
    "gear",
    "map_data_id",
    "wheelbase_m",
    "overhang_m",
    "rear_overhang_m",
    "link_id",
]


def spatial_distance(first, second):
    """Return the 3-D Euclidean distance between two ENU positions."""
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(first, second)))


def resample_segment(start, end, distance_to_next, sample_distance):
    """Return interpolation fractions that uniformly sample a 3-D segment.

    ``distance_to_next`` carries the unsampled distance from earlier segments.
    The returned fractions are measured from ``start`` to ``end`` and the
    second return value is the distance required on the following segment.
    """
    if sample_distance <= 0.0:
        raise ValueError("sample_distance must be greater than zero")
    if distance_to_next <= 0.0 or distance_to_next > sample_distance:
        raise ValueError("distance_to_next must be in (0, sample_distance]")

    segment_distance = spatial_distance(start, end)
    if segment_distance <= 1e-12:
        return [], distance_to_next

    fractions = []
    travelled = 0.0
    next_distance = distance_to_next
    tolerance = 1e-9

    while segment_distance - travelled + tolerance >= next_distance:
        travelled += next_distance
        fractions.append(min(1.0, travelled / segment_distance))
        next_distance = sample_distance

    remaining = max(0.0, segment_distance - travelled)
    next_distance -= remaining
    if next_distance <= tolerance:
        next_distance = sample_distance
    return fractions, next_distance


class CsvPathWriter:
    """Write one path sample per row and flush it immediately."""

    def __init__(self, output_file, append=False):
        self.output_file = os.path.abspath(os.path.expanduser(output_file))
        output_dir = os.path.dirname(self.output_file)
        os.makedirs(output_dir, exist_ok=True)

        has_content = append and os.path.exists(self.output_file) and os.path.getsize(self.output_file) > 0
        if has_content:
            with open(self.output_file, newline="", encoding="utf-8") as existing_stream:
                existing_header = next(csv.reader(existing_stream), [])
            if existing_header != CSV_FIELDNAMES:
                raise ValueError("cannot append because the existing CSV header does not match")
        mode = "a" if append else "w"
        self._stream = open(self.output_file, mode, newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._stream, fieldnames=CSV_FIELDNAMES)
        if not has_content:
            self._writer.writeheader()
            self._sync()

    def write(self, sample):
        row = {field: sample.get(field, "") for field in CSV_FIELDNAMES}
        self._writer.writerow(row)
        self._stream.flush()

    def _sync(self):
        self._stream.flush()
        os.fsync(self._stream.fileno())

    def close(self):
        if not self._stream.closed:
            self._sync()
            self._stream.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
