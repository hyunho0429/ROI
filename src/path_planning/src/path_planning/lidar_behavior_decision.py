#!/usr/bin/env python3
"""Verified stop-first behavior decision logic.

Decision priority:
1. KEEP / FOLLOW when risk is low.
2. MONITOR as an obstacle begins closing.
3. SLOW_DOWN while stopping remains comfortably feasible.
4. BRAKE in the current lane if comfortable stopping is no longer feasible
   but emergency stopping is still feasible.
5. Only if even emergency stopping is predicted to be insufficient,
   evaluate legal/safe left/right avoidance.
6. If no legal/safe avoidance exists, BRAKE remains the fallback.

This module only outputs a behavior decision. It does not actuate the vehicle.
"""

import math


class BehaviorDecision:
    def __init__(
        self,
        front_half_width_m=1.25,
        monitor_ttc_s=6.0,
        slowdown_ttc_s=4.5,
        brake_ttc_s=2.5,
        follow_distance_m=35.0,
        emergency_distance_m=8.0,
        side_front_clearance_m=20.0,
        side_rear_clearance_m=10.0,
        lane_width_m=3.5,
        side_half_width_m=1.35,
        reaction_time_s=0.6,
        comfortable_decel_mps2=3.5,
        emergency_decel_mps2=7.0,
        stopping_margin_m=3.0,
    ):
        self.front_half_width_m = float(front_half_width_m)
        self.monitor_ttc_s = float(monitor_ttc_s)
        self.slowdown_ttc_s = float(slowdown_ttc_s)
        self.brake_ttc_s = float(brake_ttc_s)
        self.follow_distance_m = float(follow_distance_m)
        self.emergency_distance_m = float(emergency_distance_m)
        self.side_front_clearance_m = float(side_front_clearance_m)
        self.side_rear_clearance_m = float(side_rear_clearance_m)
        self.lane_width_m = float(lane_width_m)
        self.side_half_width_m = float(side_half_width_m)

        self.reaction_time_s = float(reaction_time_s)
        self.comfortable_decel_mps2 = float(comfortable_decel_mps2)
        self.emergency_decel_mps2 = float(emergency_decel_mps2)
        self.stopping_margin_m = float(stopping_margin_m)

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

    def _required_stop_distance(self, closing_speed_mps, decel_mps2):
        """Compute simplified stopping distance from relative closing speed."""
        v = max(0.0, float(closing_speed_mps))

        if v <= 0.0:
            return self.stopping_margin_m

        reaction_distance = v * self.reaction_time_s
        braking_distance = (v * v) / (2.0 * max(float(decel_mps2), 0.1))

        return reaction_distance + braking_distance + self.stopping_margin_m

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
                "stop_feasible": True,
                "comfortable_stop_feasible": True,
                "left_safe": left_safe,
                "right_safe": right_safe,
                "left_lane_change_allowed": bool(left_lane_change_allowed),
                "right_lane_change_allowed": bool(right_lane_change_allowed),
            }

        def danger_key(obs):
            ttc = float(obs.get("ttc_s", float("inf")))
            distance = float(obs.get("longitudinal_distance_m", 1e9))
            return (ttc, distance)

        target = min(front, key=danger_key)

        distance = float(target.get("longitudinal_distance_m", 1e9))
        closing_speed = float(target.get("closing_speed_mps", 0.0))
        ttc = float(target.get("ttc_s", float("inf")))

        comfortable_stop_distance = self._required_stop_distance(
            closing_speed,
            self.comfortable_decel_mps2,
        )
        emergency_stop_distance = self._required_stop_distance(
            closing_speed,
            self.emergency_decel_mps2,
        )

        comfortable_stop_feasible = distance >= comfortable_stop_distance
        emergency_stop_feasible = distance >= emergency_stop_distance

        # 1) Same-speed / nearly stable lead object.
        if closing_speed <= 0.8 and distance <= self.follow_distance_m:
            decision = "FOLLOW"
            reason = "front_object_distance_stable"

        # 2) Emergency stopping itself is insufficient.
        #    Only here do we inspect avoidance candidates.
        elif not emergency_stop_feasible:
            if left_safe and not right_safe:
                decision = "AVOID_LEFT"
                reason = "emergency_stop_infeasible_left_escape_available"

            elif right_safe and not left_safe:
                decision = "AVOID_RIGHT"
                reason = "emergency_stop_infeasible_right_escape_available"

            elif left_safe and right_safe:
                decision = "AVOID_LEFT"
                reason = "emergency_stop_infeasible_both_escapes_available_left_default"

            else:
                decision = "BRAKE"
                reason = "emergency_stop_infeasible_no_legal_safe_escape"

        # 3) Comfortable stopping is insufficient, but emergency stopping works.
        #    Stay in lane and brake.
        elif not comfortable_stop_feasible:
            decision = "BRAKE"
            reason = "comfortable_stop_infeasible_emergency_stop_feasible"

        # 4) Risk increasing, but enough stopping margin remains.
        elif ttc <= self.slowdown_ttc_s:
            decision = "SLOW_DOWN"
            reason = "preemptive_slowdown_stop_feasible"

        # 5) Closing obstacle detected, still low urgency.
        elif ttc <= self.monitor_ttc_s:
            decision = "MONITOR"
            reason = "front_object_closing_stop_margin_available"

        # 6) Very short range with a low/unstable closing-speed estimate.
        #    Prefer braking over unnecessary lane change.
        elif distance <= self.emergency_distance_m or ttc <= self.brake_ttc_s:
            decision = "BRAKE"
            reason = "short_range_risk_brake_preferred"

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
                "comfortable_stop_distance_m": comfortable_stop_distance,
                "emergency_stop_distance_m": emergency_stop_distance,
            },
            "stop_feasible": emergency_stop_feasible,
            "comfortable_stop_feasible": comfortable_stop_feasible,
            "left_safe": left_safe,
            "right_safe": right_safe,
            "left_lane_change_allowed": bool(left_lane_change_allowed),
            "right_lane_change_allowed": bool(right_lane_change_allowed),
        }
