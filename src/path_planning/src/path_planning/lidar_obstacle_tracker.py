#!/usr/bin/env python3
"""Simple LiDAR obstacle tracker for behavior-level decisions.

Tracks clustered obstacles in the ego-local frame using nearest-neighbor
association. It estimates relative velocity from frame-to-frame centroid
motion and provides closing speed and TTC for front obstacles.

This module DOES NOT send steering, throttle, or brake commands.
"""

import math
import time


class ObstacleTracker:
    def __init__(
        self,
        match_distance_m=3.0,
        max_track_age_s=0.8,
        velocity_alpha=0.45,
        min_dt_s=0.03,
        max_dt_s=0.5,
    ):
        self.match_distance_m = float(match_distance_m)
        self.max_track_age_s = float(max_track_age_s)
        self.velocity_alpha = float(velocity_alpha)
        self.min_dt_s = float(min_dt_s)
        self.max_dt_s = float(max_dt_s)
        self._tracks = {}
        self._next_id = 1

    def update(self, clusters, timestamp_s=None):
        if timestamp_s is None:
            timestamp_s = time.monotonic()

        # Remove stale tracks.
        stale = [
            track_id
            for track_id, track in self._tracks.items()
            if timestamp_s - track["last_seen_s"] > self.max_track_age_s
        ]
        for track_id in stale:
            del self._tracks[track_id]

        unmatched_track_ids = set(self._tracks.keys())
        results = []

        # Process nearest clusters first for stable front-object tracking.
        ordered_clusters = sorted(
            clusters,
            key=lambda c: float(c.get("nearest_distance_m", c.get("centroid_distance_m", 1e9))),
        )

        for cluster in ordered_clusters:
            cx = float(cluster["centroid_x_m"])
            cy = float(cluster["centroid_y_m"])

            best_track_id = None
            best_distance = None

            for track_id in unmatched_track_ids:
                track = self._tracks[track_id]
                dx = cx - track["x_m"]
                dy = cy - track["y_m"]
                distance = math.hypot(dx, dy)
                if distance <= self.match_distance_m:
                    if best_distance is None or distance < best_distance:
                        best_distance = distance
                        best_track_id = track_id

            if best_track_id is None:
                track_id = self._next_id
                self._next_id += 1
                track = {
                    "id": track_id,
                    "x_m": cx,
                    "y_m": cy,
                    "vx_rel_mps": 0.0,
                    "vy_rel_mps": 0.0,
                    "last_seen_s": timestamp_s,
                    "seen_count": 1,
                }
                self._tracks[track_id] = track
            else:
                track_id = best_track_id
                unmatched_track_ids.discard(track_id)
                track = self._tracks[track_id]

                dt = timestamp_s - track["last_seen_s"]
                if self.min_dt_s <= dt <= self.max_dt_s:
                    raw_vx = (cx - track["x_m"]) / dt
                    raw_vy = (cy - track["y_m"]) / dt
                    alpha = self.velocity_alpha
                    track["vx_rel_mps"] = (
                        alpha * raw_vx + (1.0 - alpha) * track["vx_rel_mps"]
                    )
                    track["vy_rel_mps"] = (
                        alpha * raw_vy + (1.0 - alpha) * track["vy_rel_mps"]
                    )

                track["x_m"] = cx
                track["y_m"] = cy
                track["last_seen_s"] = timestamp_s
                track["seen_count"] += 1

            result = dict(cluster)
            result["track_id"] = track["id"]
            result["relative_vx_mps"] = track["vx_rel_mps"]
            result["relative_vy_mps"] = track["vy_rel_mps"]

            # x decreases when an obstacle closes on Ego in the ego-local frame.
            closing_speed = max(0.0, -track["vx_rel_mps"])
            result["closing_speed_mps"] = closing_speed

            longitudinal_distance = max(
                0.0,
                float(cluster.get("min_x_m", cluster["centroid_x_m"])),
            )
            result["longitudinal_distance_m"] = longitudinal_distance

            if closing_speed > 0.2 and longitudinal_distance > 0.0:
                result["ttc_s"] = longitudinal_distance / closing_speed
            else:
                result["ttc_s"] = float("inf")

            results.append(result)

        return results
