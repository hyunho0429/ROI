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


def multilane_highway_pattern(info, eval_x_m=7.0, min_width_m=2.7,
                              max_width_m=4.5):
    """Classify a fresh, measured pair of left-hand lane boundaries.

    A closely paired dashed+solid marking is strong highway evidence. Two
    dashed boundaries one full lane apart are weaker evidence. Actual permission
    to cross still checks the nearest boundary.
    """
    near_type = adjacent_left_lane_type(info, eval_x_m=eval_x_m)
    if near_type not in ("white_dashed", "white_solid"):
        return None
    if (info.get("straddling_lane") or {}).get("detected", False):
        return None
    width = _finite_float(info.get("lane_width_m"))
    if width is None or not min_width_m <= width <= max_width_m:
        return None
    # At the highway entrance the nearest LEFT line becomes dashed while the
    # RIGHT shoulder stays solid. The outer-left line can be outside the
    # camera view there, so requiring it would miss the entire first gap.
    right = info.get("right_lane") or {}
    near_y = _polyval((info.get("left_lane") or {}).get("coef") or [], eval_x_m)
    right_y = _polyval(right.get("coef") or [], eval_x_m)
    right_range = right.get("x_range_m")
    right_in_range = True
    if isinstance(right_range, (list, tuple)) and len(right_range) >= 2:
        right_lo = _finite_float(right_range[0])
        right_hi = _finite_float(right_range[1])
        right_in_range = (
            right_lo is not None and right_hi is not None
            and right_lo-3.0 <= eval_x_m <= right_hi+3.0
        )
    right_edge_dashed = (
        near_type == "white_dashed"
        and right.get("detected", False)
        and right.get("type") == "white_solid"
        and not right.get("from_guide", False)
        and not right.get("coasted", False)
        and _finite_float(right.get("age")) is not None
        and _finite_float(right.get("age")) >= 2
        and right_in_range
        and near_y is not None and right_y is not None
        and right_y < -0.15
        and abs((near_y-right_y)-width) <= 0.5
    )
    outer = info.get("left_outer_lane") or {}
    if (not outer.get("detected", False) or outer.get("from_guide", False)
            or outer.get("coasted", False)):
        return "right_edge_dashed" if right_edge_dashed else None
    try:
        if int(outer.get("age", 0)) < 2:
            return None
    except (TypeError, ValueError):
        return "right_edge_dashed" if right_edge_dashed else None
    x_range = outer.get("x_range_m")
    if isinstance(x_range, (list, tuple)) and len(x_range) >= 2:
        lo, hi = _finite_float(x_range[0]), _finite_float(x_range[1])
        if lo is None or hi is None or eval_x_m < lo-3.0 or eval_x_m > hi+3.0:
            return "right_edge_dashed" if right_edge_dashed else None
    outer_y = _polyval(outer.get("coef") or [], eval_x_m)
    if near_y is None or outer_y is None:
        return "right_edge_dashed" if right_edge_dashed else None
    separation = outer_y-near_y
    outer_type = outer.get("type")
    if (0.08 <= separation <= 0.75
            and {near_type, outer_type} == {"white_dashed", "white_solid"}):
        return "paired_dashed_solid"
    if near_type != "white_dashed" or not 2.0 <= separation <= 4.8:
        return "right_edge_dashed" if right_edge_dashed else None
    if outer_type == "white_dashed":
        return "double_dashed"
    return "right_edge_dashed" if right_edge_dashed else None


class ConsecutiveLanePattern:
    """Count distinct fresh camera observations, not timer ticks."""

    def __init__(self, minimum_observations=3):
        self.minimum_observations = max(1, int(minimum_observations))
        self.last_stamp = None
        self.pattern = None
        self.count = 0

    def observe(self, stamp, pattern):
        if stamp is None or stamp == self.last_stamp:
            return self.count
        self.last_stamp = stamp
        self.count = self.count+1 if pattern and pattern == self.pattern else (1 if pattern else 0)
        self.pattern = pattern
        return self.count

    def ready(self, pattern):
        return bool(pattern and pattern == self.pattern
                    and self.count >= self.minimum_observations)


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
