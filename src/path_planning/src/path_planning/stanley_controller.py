"""Stanley path tracking over MORAI ENU CSV waypoints."""

import csv
import math
import os
from dataclasses import dataclass

from path_planning.coordinates import GpsToMapEnu, MapProjection
from path_planning.localization import wrap_angle


@dataclass(frozen=True)
class PathPoint:
    x_m: float
    y_m: float
    z_m: float = 0.0


@dataclass(frozen=True)
class StanleyResult:
    steering_rad: float
    heading_error_rad: float
    cross_track_error_m: float
    segment_index: int
    remaining_distance_m: float
    goal_reached: bool


def _split_sensor_line(line):
    if "," in line:
        return next(csv.reader([line], skipinitialspace=True))
    if "\t" in line:
        return [value.strip() for value in line.split("\t")]
    return line.split()


def _normalized_name(value):
    return "".join(character for character in value.lower() if character.isalnum())


def _numeric(value):
    try:
        float(value)
        return True
    except (TypeError, ValueError):
        return False


def _field_index(header, aliases):
    normalized = {_normalized_name(value): index for index, value in enumerate(header)}
    for alias in aliases:
        if alias in normalized:
            return normalized[alias]
    return None


def _value(values, index, line_number, name, required=True, default=0.0):
    if index is None:
        if required:
            raise ValueError("path sensor CSV is missing {}".format(name))
        return float(default)
    try:
        value = float(values[index])
    except (IndexError, TypeError, ValueError) as error:
        raise ValueError(
            "invalid {} at path file line {}".format(name, line_number)
        ) from error
    if not math.isfinite(value):
        raise ValueError(
            "non-finite {} at path file line {}".format(name, line_number)
        )
    return value


def _append_unique(points, point):
    if not points or math.dist(
        (point.x_m, point.y_m, point.z_m),
        (points[-1].x_m, points[-1].y_m, points[-1].z_m),
    ) > 1e-6:
        points.append(point)


def _load_enu_rows(header, rows, line_numbers):
    x_index = _field_index(header, {"globalenuxm", "x"})
    y_index = _field_index(header, {"globalenuym", "y"})
    z_index = _field_index(header, {"globalenuzm", "z"})
    if x_index is None or y_index is None:
        return None

    points = []
    for values, line_number in zip(rows, line_numbers):
        point = PathPoint(
            _value(values, x_index, line_number, "ENU x"),
            _value(values, y_index, line_number, "ENU y"),
            _value(values, z_index, line_number, "ENU z", required=False),
        )
        _append_unique(points, point)
    return points


def _load_gps_sensor_rows(header, rows, line_numbers, gps_projection):
    latitude_index = _field_index(
        header, {"latitude", "lat", "gpslatitude", "latitudedeg"}
    )
    longitude_index = _field_index(
        header,
        {"longitude", "lon", "lng", "gpslongitude", "longitudedeg"},
    )
    altitude_index = _field_index(
        header, {"altitude", "alt", "gpsaltitude", "altitudem"}
    )
    east_offset_index = _field_index(
        header, {"eastoffset", "eastingoffset", "eoffset"}
    )
    north_offset_index = _field_index(
        header, {"northoffset", "northingoffset", "noffset"}
    )
    if latitude_index is None or longitude_index is None:
        return None
    if (east_offset_index is None) != (north_offset_index is None):
        raise ValueError("GPS path must provide both eastOffset and northOffset")

    records = []
    for values, line_number in zip(rows, line_numbers):
        latitude = _value(values, latitude_index, line_number, "latitude")
        longitude = _value(values, longitude_index, line_number, "longitude")
        altitude = _value(values, altitude_index, line_number, "altitude")
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise ValueError(
                "invalid latitude/longitude at path file line {}".format(line_number)
            )
        east_offset = (
            None
            if east_offset_index is None
            else _value(values, east_offset_index, line_number, "eastOffset")
        )
        north_offset = (
            None
            if north_offset_index is None
            else _value(values, north_offset_index, line_number, "northOffset")
        )
        records.append((latitude, longitude, altitude, east_offset, north_offset))

    offsets_present = [record[3] is not None and record[4] is not None for record in records]
    if any(offsets_present) and not all(offsets_present):
        raise ValueError("eastOffset and northOffset must be present on every GPS row")

    crs = "EPSG:32652" if gps_projection is None else gps_projection.crs
    origin_z = 0.0 if gps_projection is None else gps_projection.origin_z_m
    if all(offsets_present):
        origin_x, origin_y = records[0][3], records[0][4]
        for record in records[1:]:
            if abs(record[3] - origin_x) > 1e-3 or abs(record[4] - origin_y) > 1e-3:
                raise ValueError("GPS eastOffset/northOffset must be constant for one path")
    elif gps_projection is not None:
        origin_x = gps_projection.origin_x_m
        origin_y = gps_projection.origin_y_m
    else:
        raise ValueError(
            "GPS path needs eastOffset/northOffset columns or an MGeo projection"
        )

    converter = GpsToMapEnu(
        MapProjection(crs, float(origin_x), float(origin_y), float(origin_z))
    )
    points = []
    for latitude, longitude, altitude, _east_offset, _north_offset in records:
        _append_unique(
            points,
            PathPoint(*converter.convert(latitude, longitude, altitude)),
        )
    return points


def load_path_csv(filename, gps_projection=None):
    """Load ENU CSV or MORAI GPS sensor-export rows as map-local ENU points.

    GPS sensor rows may be a headered CSV containing latitude, longitude,
    altitude, eastOffset and northOffset, or the documented headerless
    five-value text format. Extra IMU columns in a combined CSV are ignored;
    live IMU UDP remains the vehicle-state input during tracking.
    """
    filename = os.path.abspath(os.path.expanduser(filename))
    with open(filename, encoding="utf-8-sig") as stream:
        source_lines = [
            (line_number, line.strip())
            for line_number, line in enumerate(stream, start=1)
            if line.strip() and not line.lstrip().startswith("#")
        ]
    if not source_lines:
        raise ValueError("path file is empty")

    parsed = [(line_number, _split_sensor_line(line)) for line_number, line in source_lines]
    first_values = parsed[0][1]
    has_header = not all(_numeric(value) for value in first_values)
    if has_header:
        header = first_values
        data = parsed[1:]
    else:
        # Official MORAI GPS sensor save format:
        # latitude longitude altitude eastOffset northOffset
        header = ["latitude", "longitude", "altitude", "eastOffset", "northOffset"]
        data = parsed
    if not data:
        raise ValueError("path file contains a header but no data rows")

    rows = [values for _line_number, values in data]
    line_numbers = [line_number for line_number, _values in data]
    points = _load_enu_rows(header, rows, line_numbers)
    if points is None:
        points = _load_gps_sensor_rows(
            header, rows, line_numbers, gps_projection
        )
    if points is None:
        raise ValueError(
            "path file needs ENU x/y columns or GPS latitude/longitude/altitude "
            "with eastOffset/northOffset"
        )
    if len(points) < 2:
        raise ValueError("path CSV must contain at least two distinct points")
    return points


class StanleyController:
    def __init__(
        self,
        points,
        gain=1.2,
        softening_speed_mps=1.0,
        max_steering_deg=36.25,
        control_point_offset_m=0.0,
        z_distance_weight=0.25,
        search_back_segments=10,
        search_forward_segments=250,
        goal_tolerance_m=2.0,
    ):
        if len(points) < 2:
            raise ValueError("StanleyController needs at least two path points")
        self.points = list(points)
        self.gain = float(gain)
        self.softening_speed_mps = float(softening_speed_mps)
        self.max_steering_rad = math.radians(float(max_steering_deg))
        self.control_point_offset_m = float(control_point_offset_m)
        self.z_distance_weight = float(z_distance_weight)
        self.search_back_segments = int(search_back_segments)
        self.search_forward_segments = int(search_forward_segments)
        self.goal_tolerance_m = float(goal_tolerance_m)
        self._last_segment = None
        self._cumulative = [0.0]
        for first, second in zip(self.points, self.points[1:]):
            self._cumulative.append(
                self._cumulative[-1]
                + math.dist((first.x_m, first.y_m, first.z_m), (second.x_m, second.y_m, second.z_m))
            )

    def _nearest_segment(self, x_m, y_m, z_m):
        if self._last_segment is None:
            first_index, last_index = 0, len(self.points) - 2
        else:
            first_index = max(0, self._last_segment - self.search_back_segments)
            last_index = min(
                len(self.points) - 2,
                self._last_segment + self.search_forward_segments,
            )

        best = None
        for index in range(first_index, last_index + 1):
            first, second = self.points[index], self.points[index + 1]
            dx, dy = second.x_m - first.x_m, second.y_m - first.y_m
            length_xy_sq = dx * dx + dy * dy
            if length_xy_sq < 1e-12:
                continue
            fraction = ((x_m - first.x_m) * dx + (y_m - first.y_m) * dy) / length_xy_sq
            fraction = max(0.0, min(1.0, fraction))
            projected_x = first.x_m + fraction * dx
            projected_y = first.y_m + fraction * dy
            projected_z = first.z_m + fraction * (second.z_m - first.z_m)
            distance_sq = (
                (x_m - projected_x) ** 2
                + (y_m - projected_y) ** 2
                + self.z_distance_weight * (z_m - projected_z) ** 2
            )
            if best is None or distance_sq < best[0]:
                best = (distance_sq, index, fraction, projected_x, projected_y, projected_z)
        if best is None:
            raise RuntimeError("path contains no segment with horizontal length")
        self._last_segment = best[1]
        return best

    def compute(self, x_m, y_m, z_m, yaw_rad, speed_mps):
        control_x = x_m + self.control_point_offset_m * math.cos(yaw_rad)
        control_y = y_m + self.control_point_offset_m * math.sin(yaw_rad)
        nearest = self._nearest_segment(control_x, control_y, z_m)
        _, index, fraction, projected_x, projected_y, _projected_z = nearest
        first, second = self.points[index], self.points[index + 1]
        dx, dy = second.x_m - first.x_m, second.y_m - first.y_m
        length_xy = math.hypot(dx, dy)
        tangent_x, tangent_y = dx / length_xy, dy / length_xy
        path_yaw = math.atan2(tangent_y, tangent_x)
        heading_error = wrap_angle(path_yaw - yaw_rad)
        # Positive means the control point is to the left of the directed path.
        cross_track_error = (
            -(control_x - projected_x) * tangent_y
            + (control_y - projected_y) * tangent_x
        )
        correction = math.atan2(
            self.gain * cross_track_error,
            max(0.0, speed_mps) + self.softening_speed_mps,
        )
        steering = max(
            -self.max_steering_rad,
            min(self.max_steering_rad, heading_error - correction),
        )
        segment_3d = self._cumulative[index + 1] - self._cumulative[index]
        progress = self._cumulative[index] + fraction * segment_3d
        remaining = max(0.0, self._cumulative[-1] - progress)
        end = self.points[-1]
        end_distance = math.hypot(control_x - end.x_m, control_y - end.y_m)
        goal_reached = end_distance <= self.goal_tolerance_m and remaining <= 2.0 * self.goal_tolerance_m
        return StanleyResult(
            steering,
            heading_error,
            cross_track_error,
            index,
            remaining,
            goal_reached,
        )
