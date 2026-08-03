"""Geometry-only adjacent-lane gap checks using ego-frame LiDAR clusters."""


LANES = (("left", 1.0), ("right", -1.0))


def _reason(side_obstacle, front_clearance, rear_clearance, longitudinal_margin,
            lateral_clearance, lateral_margin):
    if side_obstacle is not None:
        return "obstacle_alongside"
    front_short = front_clearance < longitudinal_margin
    rear_short = rear_clearance < longitudinal_margin
    if front_short and rear_short:
        return "front_and_rear_margin_short"
    if front_short:
        return "front_margin_short"
    if rear_short:
        return "rear_margin_short"
    if lateral_clearance < lateral_margin:
        return "lane_width_short"
    return "available"


def assess_merge_gaps(
    clusters,
    vehicle_length_m,
    vehicle_width_m,
    vehicle_height_m,
    lane_width_m,
    lane_lateral_allowance_m,
    longitudinal_margin_m,
    lateral_margin_m,
    detection_range_m,
):
    """Assess the visible free interval around the ego position in each lane.

    The closest observed obstacle surfaces are used as gap boundaries. When no
    obstacle exists on one side, the configured LiDAR detection boundary is
    used. This reports physical fit only; it does not evaluate relative speed,
    TTC, road curvature, or a lane-change trajectory.
    """
    vehicle_length = float(vehicle_length_m)
    vehicle_width = float(vehicle_width_m)
    vehicle_height = float(vehicle_height_m)
    lane_width = float(lane_width_m)
    lane_allowance = float(lane_lateral_allowance_m)
    longitudinal_margin = float(longitudinal_margin_m)
    lateral_margin = float(lateral_margin_m)
    detection_range = float(detection_range_m)
    if min(vehicle_length, vehicle_width, vehicle_height) <= 0.0:
        raise ValueError("vehicle xyz dimensions must be positive")
    if lane_width <= 0.0:
        raise ValueError("lane width must be positive")
    if lane_allowance < 0.0:
        raise ValueError("lane lateral allowance cannot be negative")
    if longitudinal_margin < 0.0 or lateral_margin < 0.0:
        raise ValueError("gap margins cannot be negative")
    if detection_range <= 0.5 * vehicle_length:
        raise ValueError("detection range must exceed half the vehicle length")

    half_length = 0.5 * vehicle_length
    ego_front_x = half_length
    ego_rear_x = -half_length
    lane_center_tolerance = (
        0.5 * max(0.0, lane_width - vehicle_width) + lane_allowance
    )
    lateral_clearance = 0.5 * (lane_width - vehicle_width)
    required_gap = vehicle_length + 2.0 * longitudinal_margin

    assessments = {}
    for side, lateral_sign in LANES:
        lane_center_y = lateral_sign * lane_width
        lane_clusters = [
            cluster
            for cluster in clusters
            if abs(cluster["centroid_y_m"] - lane_center_y)
            <= lane_center_tolerance
        ]
        front_candidates = [
            cluster
            for cluster in lane_clusters
            if cluster["min_x_m"] > ego_front_x
        ]
        rear_candidates = [
            cluster
            for cluster in lane_clusters
            if cluster["max_x_m"] < ego_rear_x
        ]
        side_candidates = [
            cluster
            for cluster in lane_clusters
            if cluster["min_x_m"] <= ego_front_x
            and cluster["max_x_m"] >= ego_rear_x
        ]

        front_obstacle = min(
            front_candidates,
            key=lambda cluster: cluster["min_x_m"],
            default=None,
        )
        rear_obstacle = max(
            rear_candidates,
            key=lambda cluster: cluster["max_x_m"],
            default=None,
        )
        side_obstacle = min(
            side_candidates,
            key=lambda cluster: abs(cluster["centroid_x_m"]),
            default=None,
        )

        front_boundary = (
            front_obstacle["min_x_m"]
            if front_obstacle is not None
            else detection_range
        )
        rear_boundary = (
            rear_obstacle["max_x_m"]
            if rear_obstacle is not None
            else -detection_range
        )
        front_clearance = front_boundary - ego_front_x
        rear_clearance = ego_rear_x - rear_boundary
        visible_gap = (
            0.0
            if side_obstacle is not None
            else front_boundary - rear_boundary
        )
        reason = _reason(
            side_obstacle,
            front_clearance,
            rear_clearance,
            longitudinal_margin,
            lateral_clearance,
            lateral_margin,
        )

        assessments[side] = {
            "side": side,
            "available": reason == "available",
            "reason": reason,
            "lane_center_y_m": lane_center_y,
            "lane_center_tolerance_m": lane_center_tolerance,
            "vehicle_length_m": vehicle_length,
            "vehicle_width_m": vehicle_width,
            "vehicle_height_m": vehicle_height,
            "front_obstacle": front_obstacle,
            "rear_obstacle": rear_obstacle,
            "side_obstacle": side_obstacle,
            "front_boundary_m": front_boundary,
            "rear_boundary_m": rear_boundary,
            "front_boundary_source": (
                "obstacle" if front_obstacle is not None else "range_limit"
            ),
            "rear_boundary_source": (
                "obstacle" if rear_obstacle is not None else "range_limit"
            ),
            "front_clearance_m": front_clearance,
            "rear_clearance_m": rear_clearance,
            "visible_gap_m": visible_gap,
            "required_gap_m": required_gap,
            "lateral_clearance_m": lateral_clearance,
            "required_lateral_margin_m": lateral_margin,
        }
    return assessments


class MergeGapTracker:
    """Confirm clear geometry over repeated scans and expose transitions."""

    def __init__(self, confirmation_scans):
        self.confirmation_scans = int(confirmation_scans)
        if self.confirmation_scans < 1:
            raise ValueError("confirmation scans must be at least 1")
        self.counts = {side: 0 for side, _sign in LANES}
        self.confirmed = {side: False for side, _sign in LANES}

    def update(self, assessments):
        updated = {}
        became_available = []
        became_unavailable = []
        for side, _sign in LANES:
            assessment = dict(assessments[side])
            was_confirmed = self.confirmed[side]
            if assessment["available"]:
                self.counts[side] = min(
                    self.confirmation_scans,
                    self.counts[side] + 1,
                )
            else:
                self.counts[side] = 0
            is_confirmed = self.counts[side] >= self.confirmation_scans
            self.confirmed[side] = is_confirmed
            assessment["confirmation_count"] = self.counts[side]
            assessment["confirmation_required"] = self.confirmation_scans
            assessment["confirmed_available"] = is_confirmed
            updated[side] = assessment
            if is_confirmed and not was_confirmed:
                became_available.append(side)
            elif was_confirmed and not is_confirmed:
                became_unavailable.append(side)
        return updated, became_available, became_unavailable


def format_merge_gap_status(assessment):
    """Return a compact status string suitable for periodic ROS logs."""
    if assessment["confirmed_available"]:
        state = "AVAILABLE"
    elif assessment["available"]:
        state = "CHECKING({}/{})".format(
            assessment["confirmation_count"],
            assessment["confirmation_required"],
        )
    else:
        state = "BLOCKED"
    return (
        "{side}={state} reason={reason} gap={gap:.2f}/{required:.2f}m "
        "front={front:.2f}m({front_source}) rear={rear:.2f}m({rear_source}) "
        "lateral={lateral:.2f}m"
    ).format(
        side=assessment["side"].upper(),
        state=state,
        reason=assessment["reason"],
        gap=assessment["visible_gap_m"],
        required=assessment["required_gap_m"],
        front=assessment["front_clearance_m"],
        front_source=assessment["front_boundary_source"],
        rear=assessment["rear_clearance_m"],
        rear_source=assessment["rear_boundary_source"],
        lateral=assessment["lateral_clearance_m"],
    )
