"""Stanley path tracking over MORAI ENU CSV waypoints."""

import csv
import math
import os
from dataclasses import dataclass

from path_planning.coordinates import GeodeticOrigin, GpsToMapEnu, MapProjection
from path_planning.localization import wrap_angle


_GPS_LATITUDE_FIELDS = {
    "latitude", "lat", "gpslatitude", "latitudedeg", "위도", "위도deg"
}
_GPS_LONGITUDE_FIELDS = {
    "longitude", "lon", "lng", "gpslongitude", "longitudedeg", "경도", "경도deg"
}
_GPS_ALTITUDE_FIELDS = {
    "altitude", "alt", "gpsaltitude", "altitudem", "고도", "고도m"
}
_GPS_EAST_OFFSET_FIELDS = {
    "eastoffset",
    "eastoffsetm",
    "eastingoffset",
    "eoffset",
    "eastcoordinate",
    "동쪽좌표",
    "동쪽좌표m",
    "동쪽오프셋",
    "동쪽오프셋m",
}
_GPS_NORTH_OFFSET_FIELDS = {
    "northoffset",
    "northoffsetm",
    "northingoffset",
    "noffset",
    "northcoordinate",
    "북쪽좌표",
    "북쪽좌표m",
    "북쪽오프셋",
    "북쪽오프셋m",
}
_GPS_UP_OFFSET_FIELDS = {
    "upoffset",
    "upoffsetm",
    "originz",
    "originzm",
    "maporiginz",
    "maporiginzm",
}
_GPS_CRS_FIELDS = {"crs", "utmcrs", "projectioncrs"}


@dataclass(frozen=True)
class PathPoint:
    x_m: float
    y_m: float
    z_m: float = 0.0
    target_speed_mps: float = None


@dataclass(frozen=True)
class StanleyResult:
    steering_rad: float
    heading_error_rad: float
    cross_track_error_m: float
    segment_index: int
    remaining_distance_m: float
    goal_reached: bool
    path_yaw_rad: float
    target_speed_mps: float = None


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


def _headerless_path_header(values):
    """Infer the two documented headerless path layouts by column count."""
    if len(values) == 2:
        return ["x", "y"]
    if len(values) == 3:
        return ["x", "y", "z"]
    if len(values) == 5:
        return [
            "latitude",
            "longitude",
            "altitude",
            "eastOffset",
            "northOffset",
        ]
    raise ValueError(
        "headerless path needs 2/3 ENU columns or 5 MORAI GPS columns"
    )


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


def _target_speed(values, header, line_number):
    speed_mps_index = _field_index(
        header, {"targetspeed", "targetspeedmps", "speedmps"}
    )
    speed_kmh_index = _field_index(
        header, {"targetspeedkmh", "targetspeedkph", "speedkmh", "speedkph"}
    )
    index = speed_mps_index if speed_mps_index is not None else speed_kmh_index
    if index is None or index >= len(values) or not values[index].strip():
        return None
    speed = _value(values, index, line_number, "target speed")
    if speed < 0.0:
        raise ValueError(
            "target speed cannot be negative at path file line {}".format(line_number)
        )
    return speed if speed_mps_index is not None else speed / 3.6


def _recorded_origin(header, rows, line_numbers):
    """Read the fixed GPS origin stored in an AutoVehicle-style path CSV."""
    origin_latitude_index = _field_index(
        header, {"originlat", "originlatitude", "originlatitudedeg"}
    )
    origin_longitude_index = _field_index(
        header, {"originlon", "originlng", "originlongitude", "originlongitudedeg"}
    )
    origin_altitude_index = _field_index(
        header, {"originalt", "originaltitude", "originaltitudem"}
    )
    if origin_latitude_index is None or origin_longitude_index is None:
        return None

    origins = []
    for values, line_number in zip(rows, line_numbers):
        try:
            latitude_text = values[origin_latitude_index].strip()
            longitude_text = values[origin_longitude_index].strip()
        except IndexError as error:
            raise ValueError(
                "missing origin coordinate at path file line {}".format(line_number)
            ) from error
        if not latitude_text and not longitude_text:
            continue
        if not latitude_text or not longitude_text:
            raise ValueError(
                "origin_lat and origin_lon must both be set at path file line {}".format(
                    line_number
                )
            )
        latitude = _value(
            values, origin_latitude_index, line_number, "origin latitude"
        )
        longitude = _value(
            values, origin_longitude_index, line_number, "origin longitude"
        )
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise ValueError(
                "invalid origin latitude/longitude at path file line {}".format(
                    line_number
                )
            )
        altitude = 0.0
        if (
            origin_altitude_index is not None
            and origin_altitude_index < len(values)
            and values[origin_altitude_index].strip()
        ):
            altitude = _value(
                values, origin_altitude_index, line_number, "origin altitude"
            )
        origins.append(GeodeticOrigin(latitude, longitude, altitude))
    if not origins:
        raise ValueError("origin_lat/origin_lon path has no populated origin row")

    origin = origins[0]
    for candidate in origins[1:]:
        if (
            abs(candidate.latitude_deg - origin.latitude_deg) > 1e-8
            or abs(candidate.longitude_deg - origin.longitude_deg) > 1e-8
            or abs(candidate.altitude_m - origin.altitude_m) > 1e-8
        ):
            raise ValueError("origin_lat/origin_lon must be constant for one path")
    return origin


def _load_origin_anchored_rows(header, rows, line_numbers, _gps_projection):
    """Load x/y/z directly in the GPS origin frame used while recording."""
    x_index = _field_index(header, {"x", "localx", "enuoriginx"})
    y_index = _field_index(header, {"y", "localy", "enuoriginy"})
    z_index = _field_index(header, {"z", "localz", "enuoriginz"})
    if x_index is None or y_index is None:
        return None
    if _recorded_origin(header, rows, line_numbers) is None:
        return None
    points = []
    for values, line_number in zip(rows, line_numbers):
        point = PathPoint(
            _value(values, x_index, line_number, "local x"),
            _value(values, y_index, line_number, "local y"),
            _value(values, z_index, line_number, "local z", required=False),
            _target_speed(values, header, line_number),
        )
        _append_unique(points, point)
    return points


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
            _target_speed(values, header, line_number),
        )
        _append_unique(points, point)
    return points


def _load_gps_sensor_rows(header, rows, line_numbers, gps_projection):
    latitude_index = _field_index(header, _GPS_LATITUDE_FIELDS)
    longitude_index = _field_index(header, _GPS_LONGITUDE_FIELDS)
    altitude_index = _field_index(header, _GPS_ALTITUDE_FIELDS)
    east_offset_index = _field_index(header, _GPS_EAST_OFFSET_FIELDS)
    north_offset_index = _field_index(header, _GPS_NORTH_OFFSET_FIELDS)
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
        records.append(
            (
                latitude,
                longitude,
                altitude,
                east_offset,
                north_offset,
                _target_speed(values, header, line_number),
            )
        )

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
    for (
        latitude,
        longitude,
        altitude,
        _east_offset,
        _north_offset,
        target_speed_mps,
    ) in records:
        x_m, y_m, z_m = converter.convert(latitude, longitude, altitude)
        _append_unique(
            points,
            PathPoint(x_m, y_m, z_m, target_speed_mps),
        )
    return points


def load_path_csv(filename, gps_projection=None):
    """Load ENU CSV/TXT or MORAI GPS sensor-export rows as map-local ENU.

    Headerless two/three-value rows are interpreted as map-local ENU x/y[/z].
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
        header = _headerless_path_header(first_values)
        data = parsed
        expected_columns = len(first_values)
        for line_number, values in data:
            if len(values) != expected_columns:
                raise ValueError(
                    "inconsistent column count at path file line {}".format(
                        line_number
                    )
                )
    if not data:
        raise ValueError("path file contains a header but no data rows")

    rows = [values for _line_number, values in data]
    line_numbers = [line_number for line_number, _values in data]
    points = _load_origin_anchored_rows(
        header, rows, line_numbers, gps_projection
    )
    if points is None:
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


def load_gps_path_projection(filename, fallback_projection=None):
    """Return the fixed UTM origin embedded in a MORAI five-column GPS path.

    The documented EastOffset/NorthOffset values are map constants, not each
    sample's easting/northing.  Using this projection for live GPS guarantees
    that the vehicle state and the recorded path share one map-local ENU frame.
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

    parsed = [
        (line_number, _split_sensor_line(line))
        for line_number, line in source_lines
    ]
    first_values = parsed[0][1]
    if all(_numeric(value) for value in first_values):
        header = _headerless_path_header(first_values)
        data = parsed
    else:
        header = first_values
        data = parsed[1:]

    latitude_index = _field_index(header, _GPS_LATITUDE_FIELDS)
    longitude_index = _field_index(header, _GPS_LONGITUDE_FIELDS)
    east_offset_index = _field_index(header, _GPS_EAST_OFFSET_FIELDS)
    north_offset_index = _field_index(header, _GPS_NORTH_OFFSET_FIELDS)
    up_offset_index = _field_index(header, _GPS_UP_OFFSET_FIELDS)
    crs_index = _field_index(header, _GPS_CRS_FIELDS)
    if latitude_index is None or longitude_index is None:
        return None
    if east_offset_index is None and north_offset_index is None:
        return None
    if east_offset_index is None or north_offset_index is None:
        raise ValueError("GPS path must provide both eastOffset and northOffset")
    if not data:
        raise ValueError("path file contains a header but no data rows")

    offsets = []
    for line_number, values in data:
        offsets.append(
            (
                _value(values, east_offset_index, line_number, "eastOffset"),
                _value(values, north_offset_index, line_number, "northOffset"),
            )
        )
    origin_x, origin_y = offsets[0]
    for east_offset, north_offset in offsets[1:]:
        if abs(east_offset - origin_x) > 1e-3 or abs(north_offset - origin_y) > 1e-3:
            raise ValueError("GPS eastOffset/northOffset must be constant for one path")

    crs = "EPSG:32652" if fallback_projection is None else fallback_projection.crs
    if crs_index is not None:
        crs_values = []
        for line_number, values in data:
            try:
                value = values[crs_index].strip()
            except IndexError as error:
                raise ValueError(
                    "missing projection CRS at path file line {}".format(line_number)
                ) from error
            if not value:
                raise ValueError(
                    "empty projection CRS at path file line {}".format(line_number)
                )
            crs_values.append(value)
        if any(value != crs_values[0] for value in crs_values[1:]):
            raise ValueError("projection CRS must be constant for one path")
        crs = crs_values[0]

    origin_z = 0.0 if fallback_projection is None else fallback_projection.origin_z_m
    if up_offset_index is not None:
        up_offsets = [
            _value(values, up_offset_index, line_number, "upOffset")
            for line_number, values in data
        ]
        if any(abs(value - up_offsets[0]) > 1e-3 for value in up_offsets[1:]):
            raise ValueError("GPS upOffset must be constant for one path")
        origin_z = up_offsets[0]
    return MapProjection(crs, origin_x, origin_y, origin_z)


def load_recorded_path_origin(filename):
    """Return a recorded CSV's fixed GPS origin, or None for other formats."""
    filename = os.path.abspath(os.path.expanduser(filename))
    with open(filename, encoding="utf-8-sig") as stream:
        source_lines = [
            (line_number, line.strip())
            for line_number, line in enumerate(stream, start=1)
            if line.strip() and not line.lstrip().startswith("#")
        ]
    if not source_lines:
        raise ValueError("path file is empty")
    parsed = [
        (line_number, _split_sensor_line(line))
        for line_number, line in source_lines
    ]
    header = parsed[0][1]
    if all(_numeric(value) for value in header):
        return None
    x_index = _field_index(header, {"x", "localx", "enuoriginx"})
    y_index = _field_index(header, {"y", "localy", "enuoriginy"})
    if x_index is None or y_index is None or len(parsed) < 2:
        return None
    rows = [values for _line_number, values in parsed[1:]]
    line_numbers = [line_number for line_number, _values in parsed[1:]]
    return _recorded_origin(header, rows, line_numbers)


class SteeringCommandFilter:
    """AutoVehicle-style low-pass and steering-rate limiter."""

    def __init__(self, alpha=0.25, max_rate_radps=0.4, max_abs_rad=math.inf):
        self.alpha = max(0.0, min(1.0, float(alpha)))
        self.max_rate_radps = max(0.0, float(max_rate_radps))
        self.max_abs_rad = abs(float(max_abs_rad))
        self._last_steering = None
        self._last_timestamp = None

    def reset(self):
        self._last_steering = None
        self._last_timestamp = None

    def update(self, steering_rad, timestamp):
        steering = max(
            -self.max_abs_rad, min(self.max_abs_rad, float(steering_rad))
        )
        timestamp = float(timestamp)
        if self._last_steering is None:
            self._last_steering = steering
            self._last_timestamp = timestamp
            return steering

        dt = max(1e-3, timestamp - self._last_timestamp)
        filtered = (
            self.alpha * steering + (1.0 - self.alpha) * self._last_steering
        )
        maximum_delta = self.max_rate_radps * dt
        delta = max(
            -maximum_delta,
            min(maximum_delta, filtered - self._last_steering),
        )
        self._last_steering = max(
            -self.max_abs_rad,
            min(self.max_abs_rad, self._last_steering + delta),
        )
        self._last_timestamp = timestamp
        return self._last_steering


def _preprocess_path_points(points, minimum_spacing_m, smoothing_window):
    """Apply AutoVehicle waypoint spacing and moving-average smoothing."""
    spaced = []
    for point in points:
        if not spaced or math.hypot(
            point.x_m - spaced[-1].x_m, point.y_m - spaced[-1].y_m
        ) >= minimum_spacing_m:
            spaced.append(point)
    if len(spaced) < 2:
        raise ValueError("path needs at least two points after spacing")

    window = max(1, int(smoothing_window))
    if window <= 1 or len(spaced) < 3:
        return spaced
    half_window = window // 2
    smoothed = []
    for index, point in enumerate(spaced):
        if index == 0 or index == len(spaced) - 1:
            smoothed.append(point)
            continue
        first = max(0, index - half_window)
        last = min(len(spaced), index + half_window + 1)
        neighbors = spaced[first:last]
        smoothed.append(
            PathPoint(
                sum(item.x_m for item in neighbors) / len(neighbors),
                sum(item.y_m for item in neighbors) / len(neighbors),
                sum(item.z_m for item in neighbors) / len(neighbors),
                point.target_speed_mps,
            )
        )
    return smoothed


class StanleyController:
    def __init__(
        self,
        points,
        gain=0.22,
        softening_speed_mps=3.0,
        max_steering_deg=21.77,
        control_point_offset_m=3.0,
        heading_error_gain=1.0,
        cross_track_error_gain=0.55,
        cross_track_deadband_m=0.05,
        minimum_waypoint_spacing_m=0.5,
        waypoint_smoothing_window=9,
        z_distance_weight=0.25,
        search_back_segments=0,
        search_forward_segments=50,
        goal_tolerance_m=2.0,
    ):
        if len(points) < 2:
            raise ValueError("StanleyController needs at least two path points")
        self.original_point_count = len(points)
        self.points = _preprocess_path_points(
            list(points),
            max(0.0, float(minimum_waypoint_spacing_m)),
            waypoint_smoothing_window,
        )
        self.gain = float(gain)
        self.softening_speed_mps = float(softening_speed_mps)
        self.max_steering_rad = math.radians(float(max_steering_deg))
        self.control_point_offset_m = float(control_point_offset_m)
        self.heading_error_gain = float(heading_error_gain)
        self.cross_track_error_gain = float(cross_track_error_gain)
        self.cross_track_deadband_m = max(0.0, float(cross_track_deadband_m))
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
        segment_3d = self._cumulative[index + 1] - self._cumulative[index]
        progress = self._cumulative[index] + fraction * segment_3d
        # Strict Stanley: the nearest path segment supplies the desired
        # heading.  No Pure Pursuit-style look-ahead target is used.
        path_yaw = math.atan2(tangent_y, tangent_x)
        heading_error = wrap_angle(path_yaw - yaw_rad)
        # Positive means the control point is to the left of the directed path.
        cross_track_error = (
            -(control_x - projected_x) * tangent_y
            + (control_y - projected_y) * tangent_x
        )
        if abs(cross_track_error) <= self.cross_track_deadband_m:
            cross_track_error = 0.0
        else:
            cross_track_error = math.copysign(
                abs(cross_track_error) - self.cross_track_deadband_m,
                cross_track_error,
            )
        correction = math.atan2(
            self.gain * cross_track_error,
            max(0.0, speed_mps) + self.softening_speed_mps,
        )
        steering = max(
            -self.max_steering_rad,
            min(
                self.max_steering_rad,
                self.heading_error_gain * heading_error
                - self.cross_track_error_gain * correction,
            ),
        )
        remaining = max(0.0, self._cumulative[-1] - progress)
        end = self.points[-1]
        end_distance = math.hypot(control_x - end.x_m, control_y - end.y_m)
        goal_reached = end_distance <= self.goal_tolerance_m and remaining <= 2.0 * self.goal_tolerance_m
        first_speed = first.target_speed_mps
        second_speed = second.target_speed_mps
        if first_speed is None:
            target_speed_mps = second_speed
        elif second_speed is None:
            target_speed_mps = first_speed
        else:
            target_speed_mps = first_speed + fraction * (second_speed - first_speed)
        return StanleyResult(
            steering,
            heading_error,
            cross_track_error,
            index,
            remaining,
            goal_reached,
            path_yaw,
            target_speed_mps,
        )
