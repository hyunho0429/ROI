"""Coordinate conversion helpers for MORAI GPS and local ENU map paths."""

import json
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
                "'python3 -m pip install -r src/path_planning/requirements.txt'"
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
