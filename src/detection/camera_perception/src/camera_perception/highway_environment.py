"""Pure state logic for the highway-environment gate."""

import math


def _finite_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _polyval(coefficients, x):
    value = 0.0
    try:
        for coefficient in coefficients:
            coefficient = float(coefficient)
            if not math.isfinite(coefficient):
                return None
            value = value * x + coefficient
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def adjacent_left_lane_type(
    info,
    eval_x_m=7.0,
    min_y_m=0.15,
    max_y_m=2.6,
    min_track_age=2,
):
    """Return the freshly measured nearest-left boundary type, if adjacent.

    ``left_lane`` is the six-class detector's closest boundary on the left.
    The extra geometry check prevents a farther lane marking from authorizing a
    merge when the actual adjacent boundary is missing.  HELD/guide/coasted
    observations may support short-term steering continuity, but may not start
    or repeat a lane change.
    """
    if not isinstance(info, dict):
        return None
    if not bool(info.get("lane_valid", False)):
        return None
    if str(info.get("output_status", "")).upper() != "FRESH":
        return None

    lane = info.get("left_lane")
    if not isinstance(lane, dict) or not bool(lane.get("detected", False)):
        return None
    if bool(lane.get("from_guide", False)) or bool(lane.get("coasted", False)):
        return None

    age = lane.get("age")
    if age is not None:
        try:
            if int(age) < int(min_track_age):
                return None
        except (TypeError, ValueError):
            return None

    eval_x = _finite_float(eval_x_m)
    if eval_x is None:
        return None

    x_range = lane.get("x_range_m")
    if isinstance(x_range, (list, tuple)) and len(x_range) >= 2:
        lo = _finite_float(x_range[0])
        hi = _finite_float(x_range[1])
        if lo is None or hi is None or eval_x < lo - 3.0 or eval_x > hi + 3.0:
            return None

    y = _polyval(lane.get("coef") or [], eval_x)
    if y is None:
        return None
    if not float(min_y_m) <= y <= float(max_y_m):
        return None

    lane_type = lane.get("type")
    return str(lane_type) if lane_type else None


def adjacent_left_lane_semantics(info, **kwargs):
    """Return mutually consistent dashed/solid semantics for one boundary."""
    lane_type = adjacent_left_lane_type(info, **kwargs)
    return {
        "type": lane_type,
        "dashed": lane_type == "white_dashed",
        "solid": lane_type in ("white_solid", "yellow"),
        "yellow_solid": lane_type == "yellow",
    }


class HighwayEnvironmentLatch:
    """Optionally keep the highway state active after its first detection."""

    def __init__(self, latch_once=True):
        self.latch_once = bool(latch_once)
        self.latched = False

    def update(self, conditions_met):
        conditions_met = bool(conditions_met)
        if conditions_met:
            self.latched = True
        return self.latched if self.latch_once else conditions_met


class AdjacentDashedHold:
    """Bridge missed dashed frames but clear on positive solid evidence."""

    def __init__(self, hold_s):
        self.hold_s = float(hold_s)
        if self.hold_s <= 0.0:
            raise ValueError("hold_s must be positive")
        self.last_dashed_at = None

    def observe_dashed(self, detected, now):
        if bool(detected):
            self.last_dashed_at = float(now)

    def observe_solid(self, detected):
        if bool(detected):
            self.last_dashed_at = None

    def active(self, now):
        return bool(
            self.last_dashed_at is not None
            and float(now)-self.last_dashed_at <= self.hold_s
        )


def exclusive_highway_active(highway_candidate, intersection_active):
    """Give intersection state priority over the highway/merge state."""
    return bool(highway_candidate) and not bool(intersection_active)
