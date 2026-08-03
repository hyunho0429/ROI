"""Static adjacent-lane gap assessment from ego-frame LiDAR clusters."""


LANE_SIDES = (("left", 1.0), ("right", -1.0))


def _reason(front_vehicle, rear_vehicle, side_vehicle, front_ok, rear_ok):
    if side_vehicle is not None:
        return "vehicle_alongside"
    if front_vehicle is None and rear_vehicle is None:
        return "front_and_rear_not_detected"
    if front_vehicle is None:
        return "front_vehicle_not_detected"
    if rear_vehicle is None:
        return "rear_vehicle_not_detected"
    if not front_ok and not rear_ok:
        return "front_and_rear_clearance_short"
    if not front_ok:
        return "front_clearance_short"
    if not rear_ok:
        return "rear_clearance_short"
    return "available"


def assess_adjacent_lane_gaps(
    clusters,
    ego_length_m,
    ego_width_m,
    lane_width_m,
    lateral_allowance_m,
    front_clearance_m,
    rear_clearance_m,
):
    """Assess whether the current ego longitudinal position fits each lane gap.

    A gap is available only when both a leading and a following vehicle are
    observed in the adjacent lane, no vehicle overlaps the ego longitudinal
    footprint, and the configured clearances are satisfied. Cluster x bounds
    represent the observed vehicle surfaces nearest to the gap.
    """
    ego_length = float(ego_length_m)
    ego_width = float(ego_width_m)
    lane_width = float(lane_width_m)
    lateral_allowance = float(lateral_allowance_m)
    required_front_clearance = float(front_clearance_m)
    required_rear_clearance = float(rear_clearance_m)
    if ego_length <= 0.0 or ego_width <= 0.0:
        raise ValueError("ego dimensions must be positive")
    if lane_width <= ego_width:
        raise ValueError("lane width must be greater than ego width")
    if lateral_allowance < 0.0:
        raise ValueError("lateral allowance cannot be negative")
    if required_front_clearance < 0.0 or required_rear_clearance < 0.0:
        raise ValueError("longitudinal clearances cannot be negative")

    half_length = 0.5 * ego_length
    ego_front_x = half_length
    ego_rear_x = -half_length
    lane_center_tolerance = 0.5 * (lane_width - ego_width) + lateral_allowance
    required_gap = (
        ego_length + required_front_clearance + required_rear_clearance
    )

    assessments = {}
    for side, lateral_sign in LANE_SIDES:
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

        front_vehicle = min(
            front_candidates,
            key=lambda cluster: cluster["min_x_m"],
            default=None,
        )
        rear_vehicle = max(
            rear_candidates,
            key=lambda cluster: cluster["max_x_m"],
            default=None,
        )
        side_vehicle = min(
            side_candidates,
            key=lambda cluster: abs(cluster["centroid_x_m"]),
            default=None,
        )

        front_clearance = (
            front_vehicle["min_x_m"] - ego_front_x
            if front_vehicle is not None
            else None
        )
        rear_clearance = (
            ego_rear_x - rear_vehicle["max_x_m"]
            if rear_vehicle is not None
            else None
        )
        free_gap = (
            front_vehicle["min_x_m"] - rear_vehicle["max_x_m"]
            if front_vehicle is not None and rear_vehicle is not None
            else None
        )
        front_ok = (
            front_clearance is not None
            and front_clearance >= required_front_clearance
        )
        rear_ok = (
            rear_clearance is not None
            and rear_clearance >= required_rear_clearance
        )
        available = side_vehicle is None and front_ok and rear_ok

        assessments[side] = {
            "side": side,
            "lane_center_y_m": lane_center_y,
            "lane_center_tolerance_m": lane_center_tolerance,
            "available": available,
            "reason": _reason(
                front_vehicle,
                rear_vehicle,
                side_vehicle,
                front_ok,
                rear_ok,
            ),
            "front_vehicle": front_vehicle,
            "rear_vehicle": rear_vehicle,
            "side_vehicle": side_vehicle,
            "front_clearance_m": front_clearance,
            "rear_clearance_m": rear_clearance,
            "free_gap_m": free_gap,
            "required_gap_m": required_gap,
        }
    return assessments


class GapAvailabilityTracker:
    """Require repeated available scans and report secured/lost transitions."""

    def __init__(self, confirmation_scans):
        self.confirmation_scans = int(confirmation_scans)
        if self.confirmation_scans < 1:
            raise ValueError("confirmation scans must be at least 1")
        self.counts = {side: 0 for side, _sign in LANE_SIDES}
        self.confirmed = {side: False for side, _sign in LANE_SIDES}

    def update(self, assessments):
        updated = {}
        secured = []
        lost = []
        for side, _sign in LANE_SIDES:
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
            assessment["confirmed_available"] = is_confirmed
            updated[side] = assessment
            if is_confirmed and not was_confirmed:
                secured.append(side)
            elif was_confirmed and not is_confirmed:
                lost.append(side)
        return updated, secured, lost
