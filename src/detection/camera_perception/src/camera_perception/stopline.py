"""Pure stop-line control helpers shared by the camera runtime and tests."""

import math


def stopline_requires_stop(distance_m, maximum_distance_m=1.0):
    """Return True only for a valid stop line at or inside the stop distance."""
    maximum_distance = float(maximum_distance_m)
    if not math.isfinite(maximum_distance) or maximum_distance <= 0.0:
        raise ValueError("maximum_distance_m must be positive and finite")
    if distance_m is None:
        return False
    distance = float(distance_m)
    return math.isfinite(distance) and 0.0 <= distance <= maximum_distance
