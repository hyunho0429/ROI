#!/usr/bin/env python3
"""Behavior-level obstacle avoidance decision logic.

Output states:
    KEEP
    MONITOR
    FOLLOW
    SLOW_DOWN
    AVOID_LEFT
    AVOID_RIGHT
    BRAKE

This module only decides what SHOULD be done. It does not actuate the vehicle.
"""

import math


class BehaviorDecision:
    def __init__(
        self,
        front_half_width_m=1.25,
        monitor_ttc_s=6.0,
        slowdown_ttc_s=4.5,
        avoid_ttc_s=3.0,
        brake_ttc_s=1.5,
        follow_distance_m=35.0,
        emergency_distance_m=8.0,
        side_front_clearance_m=20.0,
        side_rear_clearance_m=10.0,
        lane_width_m=3.5,
        side_half_width_m=1.35,
    ):
        self.front_half_width_m = float(front_half_width_m)
        self.monitor_ttc_s = float(monitor_ttc_s)
        self.slowdown_ttc_s = float(slowdown_ttc_s)
        self.avoid_ttc_s = float(avoid_ttc_s)
        self.brake_ttc_s = float(brake_ttc_s)
        self.follow_distance_m = float(follow_distance_m)
        self.emergency_distance_m = float(emergency_distance_m)
        self.side_front_clearance_m = float(side_front_clearance_m)
        self.side_rear_clearance_m = float(side_rear_clearance_m)
        self.lane_width_m = float(lane_width_m)
        self.side_half_width_m = float(side_half_width_m)

    def _front_obstacles(self, tracks):
        return [
            obs
            for obs in tracks
            if float(obs.get("max_x_m", obs.get("centroid_x_m", 0.0))) > 0.0
            and abs(float(obs.get("centroid_y_m", 0.0))) <= self.front_half_width_m
        ]

    def _side_safe(self, tracks, side):
        center_y = self.lane_width_m if side == "left" else -self.lane_width_m

        for obs in tracks:
            y = float(obs.get("centroid_y_m", 0.0))
            x_min = float(obs.get("min_x_m", obs.get("centroid_x_m", 0.0)))
            x_max = float(obs.get("max_x_m", obs.get("centroid_x_m", 0.0)))

            in_side_corridor = abs(y - center_y) <= self.side_half_width_m
            longitudinal_overlap = (
                x_max >= -self.side_rear_clearance_m
                and x_min <= self.side_front_clearance_m
            )

            if in_side_corridor and longitudinal_overlap:
                return False

        return True

    def decide(
        self,
        tracks,
        left_lane_change_allowed=True,
        right_lane_change_allowed=True,
    ):
        front = self._front_obstacles(tracks)

        left_space_safe = self._side_safe(tracks, "left")
        right_space_safe = self._side_safe(tracks, "right")

        left_safe = bool(left_lane_change_allowed and left_space_safe)
        right_safe = bool(right_lane_change_allowed and right_space_safe)

        if not front:
            return {
                "decision": "KEEP",
                "reason": "no_front_obstacle",
                "front_obstacle": None,
                "left_safe": left_safe,
                "right_safe": right_safe,
            }

        # Prioritize the object with the smallest TTC; if TTC is infinite,
        # prioritize the nearest object.
        def danger_key(obs):
            ttc = float(obs.get("ttc_s", float("inf")))
            distance = float(obs.get("longitudinal_distance_m", 1e9))
            return (ttc, distance)

        target = min(front, key=danger_key)
        distance = float(target.get("longitudinal_distance_m", 1e9))
        closing_speed = float(target.get("closing_speed_mps", 0.0))
        ttc = float(target.get("ttc_s", float("inf")))

        # Immediate danger: do not prefer a lane change unless it is allowed
        # and clearly available. Solid-line areas can disable both sides.
        if distance <= self.emergency_distance_m or ttc <= self.brake_ttc_s:
            if left_safe:
                decision = "AVOID_LEFT"
                reason = "critical_front_risk_left_only_or_preferred"
            elif right_safe:
                decision = "AVOID_RIGHT"
                reason = "critical_front_risk_right_available"
            else:
                decision = "BRAKE"
                reason = "critical_front_risk_no_legal_escape"
        elif ttc <= self.avoid_ttc_s:
            if left_safe and not right_safe:
                decision = "AVOID_LEFT"
                reason = "high_risk_left_available"
            elif right_safe and not left_safe:
                decision = "AVOID_RIGHT"
                reason = "high_risk_right_available"
            elif left_safe and right_safe:
                # Conservative default: keep lane if braking can still buy time.
                decision = "SLOW_DOWN"
                reason = "high_risk_both_sides_available_but_slowdown_preferred"
            else:
                decision = "BRAKE"
                reason = "high_risk_no_legal_escape"
        elif ttc <= self.slowdown_ttc_s:
            decision = "SLOW_DOWN"
            reason = "closing_object_preemptive_slowdown"
        elif ttc <= self.monitor_ttc_s:
            decision = "MONITOR"
            reason = "front_object_closing_but_not_yet_critical"
        elif closing_speed <= 0.8 and distance <= self.follow_distance_m:
            decision = "FOLLOW"
            reason = "front_object_distance_stable"
        else:
            decision = "KEEP"
            reason = "front_object_not_currently_threatening"

        return {
            "decision": decision,
            "reason": reason,
            "front_obstacle": {
                "track_id": target.get("track_id"),
                "distance_m": distance,
                "closing_speed_mps": closing_speed,
                "ttc_s": None if math.isinf(ttc) else ttc,
                "bearing_deg": target.get("bearing_deg"),
            },
            "left_safe": left_safe,
            "right_safe": right_safe,
            "left_lane_change_allowed": bool(left_lane_change_allowed),
            "right_lane_change_allowed": bool(right_lane_change_allowed),
        }
