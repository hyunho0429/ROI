"""Adjacent-lane gap checks using ego-frame LiDAR clusters and tracks."""

import math


LANES = (("left", 1.0), ("right", -1.0))


def select_map_obstacles_in_adjacent_lane(
    obstacles,
    ego_x_map,
    ego_y_map,
    ego_yaw,
    side,
    lane_width_m,
    vehicle_width_m,
    lane_lateral_allowance_m,
    detection_range_m,
):
    """Select map-frame objects overlapping an ego-relative adjacent lane.

    The returned dictionaries are the original enriched obstacle states. The
    selector only converts each map center into the current ego frame for lane
    membership; published positions, velocities, and yaw remain in ``map``.
    """
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    lane_width = float(lane_width_m)
    vehicle_width = float(vehicle_width_m)
    lane_allowance = float(lane_lateral_allowance_m)
    detection_range = float(detection_range_m)
    pose_values = (
        float(ego_x_map),
        float(ego_y_map),
        float(ego_yaw),
        lane_width,
        vehicle_width,
        lane_allowance,
        detection_range,
    )
    if not all(math.isfinite(value) for value in pose_values):
        raise ValueError("ego pose and lane parameters must be finite")
    if lane_width <= 0.0 or vehicle_width <= 0.0 or detection_range <= 0.0:
        raise ValueError("lane, vehicle width, and detection range must be positive")
    if lane_allowance < 0.0:
        raise ValueError("lane lateral allowance cannot be negative")

    lateral_sign = 1.0 if side == "left" else -1.0
    lane_center_y = lateral_sign * lane_width
    center_tolerance = (
        0.5 * max(0.0, lane_width - vehicle_width) + lane_allowance
    )
    cosine = math.cos(float(ego_yaw))
    sine = math.sin(float(ego_yaw))
    selected = []
    for obstacle in obstacles:
        center_x = float(obstacle["center_x_map"])
        center_y = float(obstacle["center_y_map"])
        width = max(0.0, float(obstacle.get("width", 0.0)))
        if not all(math.isfinite(value) for value in (center_x, center_y, width)):
            raise ValueError("map obstacle contains a non-finite value")
        delta_x = center_x - float(ego_x_map)
        delta_y = center_y - float(ego_y_map)
        longitudinal = cosine * delta_x + sine * delta_y
        lateral = -sine * delta_x + cosine * delta_y
        if abs(longitudinal) > detection_range:
            continue
        if abs(lateral - lane_center_y) > center_tolerance + 0.5 * width:
            continue
        selected.append((longitudinal, obstacle))

    selected.sort(key=lambda item: item[0])
    return [obstacle for _, obstacle in selected]


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


def _tracked_box(track):
    """Normalize a Kalman/Hungarian result into an ego-frame obstacle box."""
    center_x = float(track["center_x_m"])
    center_y = float(track["center_y_m"])
    size_x = max(0.0, float(track.get("size_x_m", 0.0)))
    size_y = max(0.0, float(track.get("size_y_m", 0.0)))
    velocity_x = float(track.get("velocity_x_mps", 0.0))
    values = (center_x, center_y, size_x, size_y, velocity_x)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("tracked obstacle contains a non-finite value")
    return {
        "track_id": int(track.get("track_id", -1)),
        "confirmed": bool(track.get("confirmed", False)),
        "center_x_m": center_x,
        "center_y_m": center_y,
        "min_x_m": center_x - 0.5 * size_x,
        "max_x_m": center_x + 0.5 * size_x,
        "min_y_m": center_y - 0.5 * size_y,
        "max_y_m": center_y + 0.5 * size_y,
        "velocity_x_mps": velocity_x,
    }


def _dynamic_reason(
    side_obstacle,
    lateral_clearance,
    lateral_margin,
    front_clearance,
    rear_clearance,
    front_required,
    rear_required,
    front_ttc,
    rear_ttc,
    minimum_ttc,
):
    if side_obstacle is not None:
        return "obstacle_alongside"
    if lateral_clearance < lateral_margin:
        return "lane_width_short"
    if front_ttc < minimum_ttc:
        return "front_ttc_short"
    if rear_ttc < minimum_ttc:
        return "rear_ttc_short"
    front_short = front_clearance < front_required
    rear_short = rear_clearance < rear_required
    if front_short and rear_short:
        return "front_and_rear_dynamic_margin_short"
    if front_short:
        return "front_dynamic_margin_short"
    if rear_short:
        return "rear_dynamic_margin_short"
    return "available"


def assess_tracked_merge_gaps(
    tracks,
    vehicle_length_m,
    vehicle_width_m,
    vehicle_height_m,
    lane_width_m,
    lane_lateral_allowance_m,
    longitudinal_margin_m,
    lateral_margin_m,
    detection_range_m,
    time_headway_s,
    minimum_ttc_s,
):
    """Assess left/right insertion space using tracked boxes and relative speed.

    Track velocity is measured in the ego LiDAR frame.  A negative x velocity
    for a front vehicle means the ego is closing on it; a positive x velocity
    for a rear vehicle means it is closing on the ego.  Dynamic clearance adds
    ``closing_speed * time_headway`` to the static longitudinal margin.

    This function only reports perception evidence.  It does not command a
    lane change or prove that a complete lane-change trajectory is collision
    free.
    """
    vehicle_length = float(vehicle_length_m)
    vehicle_width = float(vehicle_width_m)
    vehicle_height = float(vehicle_height_m)
    lane_width = float(lane_width_m)
    lane_allowance = float(lane_lateral_allowance_m)
    longitudinal_margin = float(longitudinal_margin_m)
    lateral_margin = float(lateral_margin_m)
    detection_range = float(detection_range_m)
    time_headway = float(time_headway_s)
    minimum_ttc = float(minimum_ttc_s)
    if min(vehicle_length, vehicle_width, vehicle_height) <= 0.0:
        raise ValueError("vehicle xyz dimensions must be positive")
    if lane_width <= 0.0 or detection_range <= 0.5 * vehicle_length:
        raise ValueError("lane width and detection range must cover the ego")
    if min(lane_allowance, longitudinal_margin, lateral_margin) < 0.0:
        raise ValueError("gap margins cannot be negative")
    if time_headway < 0.0 or minimum_ttc < 0.0:
        raise ValueError("time headway and minimum TTC cannot be negative")

    boxes = [_tracked_box(track) for track in tracks]
    half_length = 0.5 * vehicle_length
    ego_front_x = half_length
    ego_rear_x = -half_length
    lateral_clearance = 0.5 * (lane_width - vehicle_width)
    lane_center_tolerance = (
        0.5 * max(0.0, lane_width - vehicle_width) + lane_allowance
    )

    assessments = {}
    for side, lateral_sign in LANES:
        lane_center_y = lateral_sign * lane_width
        lane_min_y = lane_center_y - lane_center_tolerance
        lane_max_y = lane_center_y + lane_center_tolerance
        lane_boxes = [
            box
            for box in boxes
            if box["max_y_m"] >= lane_min_y and box["min_y_m"] <= lane_max_y
        ]
        front_candidates = [
            box for box in lane_boxes if box["min_x_m"] > ego_front_x
        ]
        rear_candidates = [
            box for box in lane_boxes if box["max_x_m"] < ego_rear_x
        ]
        side_candidates = [
            box
            for box in lane_boxes
            if box["min_x_m"] <= ego_front_x
            and box["max_x_m"] >= ego_rear_x
        ]

        front_obstacle = min(
            front_candidates,
            key=lambda obstacle: obstacle["min_x_m"],
            default=None,
        )
        rear_obstacle = max(
            rear_candidates,
            key=lambda obstacle: obstacle["max_x_m"],
            default=None,
        )
        side_obstacle = min(
            side_candidates,
            key=lambda obstacle: abs(obstacle["center_x_m"]),
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
        front_relative_vx = (
            front_obstacle["velocity_x_mps"]
            if front_obstacle is not None
            else 0.0
        )
        rear_relative_vx = (
            rear_obstacle["velocity_x_mps"]
            if rear_obstacle is not None
            else 0.0
        )
        front_closing_speed = max(0.0, -front_relative_vx)
        rear_closing_speed = max(0.0, rear_relative_vx)
        front_required = longitudinal_margin + front_closing_speed * time_headway
        rear_required = longitudinal_margin + rear_closing_speed * time_headway
        front_ttc = (
            front_clearance / front_closing_speed
            if front_closing_speed > 1e-3
            else math.inf
        )
        rear_ttc = (
            rear_clearance / rear_closing_speed
            if rear_closing_speed > 1e-3
            else math.inf
        )
        visible_gap = (
            0.0 if side_obstacle is not None else front_boundary - rear_boundary
        )
        reason = _dynamic_reason(
            side_obstacle,
            lateral_clearance,
            lateral_margin,
            front_clearance,
            rear_clearance,
            front_required,
            rear_required,
            front_ttc,
            rear_ttc,
            minimum_ttc,
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
                "track" if front_obstacle is not None else "range_limit"
            ),
            "rear_boundary_source": (
                "track" if rear_obstacle is not None else "range_limit"
            ),
            "front_clearance_m": front_clearance,
            "rear_clearance_m": rear_clearance,
            "visible_gap_m": visible_gap,
            "required_gap_m": vehicle_length + front_required + rear_required,
            "front_required_clearance_m": front_required,
            "rear_required_clearance_m": rear_required,
            "front_relative_velocity_x_mps": front_relative_vx,
            "rear_relative_velocity_x_mps": rear_relative_vx,
            "front_ttc_s": front_ttc,
            "rear_ttc_s": rear_ttc,
            "minimum_ttc_s": minimum_ttc,
            "time_headway_s": time_headway,
            "lateral_clearance_m": lateral_clearance,
            "required_lateral_margin_m": lateral_margin,
        }
    return assessments


def format_tracked_merge_gap_status(assessment):
    if assessment.get("confirmed_available", False):
        state = "AVAILABLE"
    elif assessment["available"]:
        state = "CHECKING({}/{})".format(
            assessment.get("confirmation_count", 0),
            assessment.get("confirmation_required", 0),
        )
    else:
        state = "BLOCKED"

    def ttc_text(value):
        return "inf" if not math.isfinite(value) else "{:.1f}".format(value)

    return (
        "{side}={state} reason={reason} gap={gap:.2f}/{required:.2f}m "
        "front={front:.2f}/{front_required:.2f}m ttc={front_ttc}s "
        "rear={rear:.2f}/{rear_required:.2f}m ttc={rear_ttc}s"
    ).format(
        side=assessment["side"].upper(),
        state=state,
        reason=assessment["reason"],
        gap=assessment["visible_gap_m"],
        required=assessment["required_gap_m"],
        front=assessment["front_clearance_m"],
        front_required=assessment["front_required_clearance_m"],
        front_ttc=ttc_text(assessment["front_ttc_s"]),
        rear=assessment["rear_clearance_m"],
        rear_required=assessment["rear_required_clearance_m"],
        rear_ttc=ttc_text(assessment["rear_ttc_s"]),
    )
