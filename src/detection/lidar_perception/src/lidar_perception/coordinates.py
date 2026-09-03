"""Coordinate conversion helpers for MORAI GPS and local ENU map paths."""

import json
import math
import os
from dataclasses import dataclass


@dataclass(frozen=True)
class MapProjection:
    crs: str
    origin_x_m: float
    origin_y_m: float
    origin_z_m: float

    @classmethod
    def from_mgeo_global_info(cls, filename):
        with open(os.path.abspath(filename), encoding="utf-8-sig") as stream:
            data = json.load(stream)
        origin = data["local_origin_in_global"]
        return cls(
            data.get("global_coordinate_system", "EPSG:32652"),
            float(origin[0]),
            float(origin[1]),
            float(origin[2]),
        )


class GpsToMapEnu:
    """Project WGS84 latitude/longitude then subtract the MGeo map origin."""

    def __init__(self, projection):
        try:
            from pyproj import CRS, Transformer
        except ImportError as error:
            raise RuntimeError(
                "GPS localization requires pyproj; install it with "
                "'sudo apt install python3-pyproj'"
            ) from error
        target = CRS.from_user_input(projection.crs)
        self._transformer = Transformer.from_crs("EPSG:4326", target, always_xy=True)
        self._projection = projection

    def convert(self, latitude_deg, longitude_deg, altitude_m=None):
        easting, northing = self._transformer.transform(longitude_deg, latitude_deg)
        z_m = 0.0 if altitude_m is None else altitude_m - self._projection.origin_z_m
        return (
            easting - self._projection.origin_x_m,
            northing - self._projection.origin_y_m,
            z_m,
        )


@dataclass(frozen=True)
class GeodeticOrigin:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float = 0.0


class GpsToRecordedLocalEnu:
    """Match AutoVehicle's GPS-origin local ENU conversion for recorded CSVs."""

    EARTH_RADIUS_M = 6378137.0

    def __init__(self, origin):
        self.origin = origin
        self._origin_latitude_rad = math.radians(origin.latitude_deg)

    def convert(self, latitude_deg, longitude_deg, altitude_m=None):
        delta_latitude_rad = math.radians(
            float(latitude_deg) - self.origin.latitude_deg
        )
        delta_longitude_rad = math.radians(
            float(longitude_deg) - self.origin.longitude_deg
        )
        x_east_m = (
            self.EARTH_RADIUS_M
            * delta_longitude_rad
            * math.cos(self._origin_latitude_rad)
        )
        y_north_m = self.EARTH_RADIUS_M * delta_latitude_rad
        z_up_m = (
            0.0
            if altitude_m is None
            else float(altitude_m) - self.origin.altitude_m
        )
        return x_east_m, y_north_m, z_up_m
