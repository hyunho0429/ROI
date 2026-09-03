#!/usr/bin/env python3

import os
import math
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from lidar_perception.lidar_merge_gap import (
    MergeGapTracker,
    assess_merge_gaps,
    assess_tracked_merge_gaps,
    format_merge_gap_status,
    format_tracked_merge_gap_status,
    select_parallel_dynamic_obstacles_in_adjacent_lane,
    select_map_obstacles_in_adjacent_lane,
)


def _cluster(min_x, max_x, center_y):
    return {
        "min_x_m": float(min_x),
        "max_x_m": float(max_x),
        "centroid_x_m": 0.5 * (float(min_x) + float(max_x)),
        "centroid_y_m": float(center_y),
    }


def _assess(clusters, lane_width=3.5):
    return assess_merge_gaps(
        clusters=clusters,
        vehicle_length_m=4.635,
        vehicle_width_m=1.892,
        vehicle_height_m=2.434,
        lane_width_m=lane_width,
        lane_lateral_allowance_m=0.4,
        longitudinal_margin_m=1.0,
        lateral_margin_m=0.2,
        detection_range_m=40.0,
    )


def _track(track_id, center_x, center_y, velocity_x=0.0):
    return {
        "track_id": track_id,
        "confirmed": True,
        "center_x_m": float(center_x),
        "center_y_m": float(center_y),
        "size_x_m": 4.0,
        "size_y_m": 1.8,
        "velocity_x_mps": float(velocity_x),
    }


def _assess_tracks(tracks, minimum_ttc=3.0):
    return assess_tracked_merge_gaps(
        tracks=tracks,
        vehicle_length_m=4.635,
        vehicle_width_m=1.892,
        vehicle_height_m=2.434,
        lane_width_m=3.5,
        lane_lateral_allowance_m=0.4,
        longitudinal_margin_m=1.0,
        lateral_margin_m=0.2,
        detection_range_m=40.0,
        time_headway_s=1.5,
        minimum_ttc_s=minimum_ttc,
    )


def _map_obstacle(
    track_id,
    center_x,
    center_y,
    width=1.8,
    velocity_x=1.0,
    velocity_y=0.0,
    motion_state="MOVING",
):
    return {
        "id": track_id,
        "center_x_map": float(center_x),
        "center_y_map": float(center_y),
        "length": 4.0,
        "width": float(width),
        "velocity_x_map": float(velocity_x),
        "velocity_y_map": float(velocity_y),
        "speed_mps": math.hypot(velocity_x, velocity_y),
        "motion_state": motion_state,
        "yaw": 0.0,
        "yaw_deg": 0.0,
        "yaw_valid": True,
        "yaw_source": "VELOCITY",
    }


def _select_left(obstacles, ego_x=0.0, ego_y=0.0, ego_yaw=0.0):
    return select_map_obstacles_in_adjacent_lane(
        obstacles=obstacles,
        ego_x_map=ego_x,
        ego_y_map=ego_y,
        ego_yaw=ego_yaw,
        side="left",
        lane_width_m=3.5,
        vehicle_width_m=1.892,
        lane_lateral_allowance_m=0.4,
        detection_range_m=40.0,
    )


def _select_left_parallel(obstacles, ego_x=0.0, ego_y=0.0, ego_yaw=0.0):
    return select_parallel_dynamic_obstacles_in_adjacent_lane(
        obstacles=obstacles,
        ego_x_map=ego_x,
        ego_y_map=ego_y,
        ego_yaw=ego_yaw,
        side="left",
        lane_width_m=3.5,
        vehicle_width_m=1.892,
        lane_lateral_allowance_m=0.4,
        detection_range_m=40.0,
        minimum_speed_mps=1.0,
        maximum_heading_error_rad=math.radians(30.0),
        maximum_lateral_speed_mps=1.5,
    )


class LidarMergeGapTest(unittest.TestCase):
    def test_uses_xyz_vehicle_size_and_range_limits_for_open_lane(self):
        left = _assess([])["left"]

        self.assertTrue(left["available"])
        self.assertAlmostEqual(left["required_gap_m"], 6.635)
        self.assertAlmostEqual(left["visible_gap_m"], 80.0)
        self.assertAlmostEqual(left["front_clearance_m"], 37.6825)
        self.assertAlmostEqual(left["rear_clearance_m"], 37.6825)
        self.assertAlmostEqual(left["vehicle_height_m"], 2.434)
        self.assertEqual(left["front_boundary_source"], "range_limit")
        self.assertEqual(left["rear_boundary_source"], "range_limit")

    def test_accepts_obstacle_bounded_gap_that_fits_ego(self):
        left = _assess(
            [
                _cluster(8.0, 12.0, 3.5),
                _cluster(-12.0, -8.0, 3.5),
            ]
        )["left"]

        self.assertTrue(left["available"])
        self.assertAlmostEqual(left["visible_gap_m"], 16.0)
        self.assertAlmostEqual(left["front_clearance_m"], 5.6825)
        self.assertAlmostEqual(left["rear_clearance_m"], 5.6825)
        self.assertEqual(left["front_boundary_source"], "obstacle")
        self.assertEqual(left["rear_boundary_source"], "obstacle")

    def test_rejects_obstacle_alongside(self):
        right = _assess([_cluster(-1.0, 1.0, -3.5)])["right"]

        self.assertFalse(right["available"])
        self.assertEqual(right["reason"], "obstacle_alongside")
        self.assertEqual(right["visible_gap_m"], 0.0)

    def test_rejects_front_margin_short(self):
        left = _assess([_cluster(3.0, 7.0, 3.5)])["left"]

        self.assertFalse(left["available"])
        self.assertEqual(left["reason"], "front_margin_short")
        self.assertLess(left["front_clearance_m"], 1.0)

    def test_rejects_lane_that_is_too_narrow(self):
        left = _assess([], lane_width=2.0)["left"]

        self.assertFalse(left["available"])
        self.assertEqual(left["reason"], "lane_width_short")
        self.assertLess(left["lateral_clearance_m"], 0.2)

    def test_tracks_confirmation_and_formats_status(self):
        tracker = MergeGapTracker(confirmation_scans=3)
        clear = _assess([])

        first, became_available, became_unavailable = tracker.update(clear)
        self.assertEqual(became_available, [])
        self.assertEqual(became_unavailable, [])
        self.assertIn("LEFT=CHECKING(1/3)", format_merge_gap_status(first["left"]))

        tracker.update(clear)
        third, became_available, became_unavailable = tracker.update(clear)
        self.assertEqual(became_available, ["left", "right"])
        self.assertEqual(became_unavailable, [])
        self.assertIn("LEFT=AVAILABLE", format_merge_gap_status(third["left"]))

        blocked = _assess([_cluster(-1.0, 1.0, 3.5)])
        fourth, became_available, became_unavailable = tracker.update(blocked)
        self.assertEqual(became_available, [])
        self.assertEqual(became_unavailable, ["left"])
        self.assertIn("LEFT=BLOCKED", format_merge_gap_status(fourth["left"]))


class TrackedMergeGapTest(unittest.TestCase):
    def test_accepts_static_front_and_rear_tracked_vehicles(self):
        left = _assess_tracks(
            [_track(1, 15.0, 3.5), _track(2, -15.0, 3.5)]
        )["left"]

        self.assertTrue(left["available"])
        self.assertEqual(left["front_boundary_source"], "track")
        self.assertEqual(left["rear_boundary_source"], "track")
        self.assertAlmostEqual(left["visible_gap_m"], 26.0)

    def test_rejects_fast_closing_front_vehicle_by_ttc(self):
        left = _assess_tracks([_track(1, 10.0, 3.5, -4.0)])["left"]

        self.assertFalse(left["available"])
        self.assertEqual(left["reason"], "front_ttc_short")
        self.assertLess(left["front_ttc_s"], 3.0)

    def test_rejects_fast_closing_rear_vehicle_by_ttc(self):
        right = _assess_tracks([_track(3, -10.0, -3.5, 4.0)])["right"]

        self.assertFalse(right["available"])
        self.assertEqual(right["reason"], "rear_ttc_short")
        self.assertLess(right["rear_ttc_s"], 3.0)

    def test_dynamic_headway_increases_required_gap(self):
        static = _assess_tracks([_track(1, 10.0, 3.5, 0.0)], minimum_ttc=0.0)["left"]
        closing = _assess_tracks([_track(1, 10.0, 3.5, -4.0)], minimum_ttc=0.0)["left"]

        self.assertTrue(static["available"])
        self.assertFalse(closing["available"])
        self.assertEqual(closing["reason"], "front_dynamic_margin_short")
        self.assertGreater(
            closing["front_required_clearance_m"],
            static["front_required_clearance_m"],
        )

    def test_formats_confirmed_dynamic_status(self):
        tracker = MergeGapTracker(1)
        assessed, _, _ = tracker.update(_assess_tracks([]))
        text = format_tracked_merge_gap_status(assessed["left"])

        self.assertIn("LEFT=AVAILABLE", text)
        self.assertIn("ttc=infs", text)


class MapAdjacentLaneSelectionTest(unittest.TestCase):
    def test_selects_only_left_lane_objects_in_range(self):
        selected = _select_left(
            [
                _map_obstacle(1, 12.0, 3.5),
                _map_obstacle(2, -8.0, 3.4),
                _map_obstacle(3, 5.0, -3.5),
                _map_obstacle(4, 5.0, 0.0),
                _map_obstacle(5, 45.0, 3.5),
            ]
        )

        self.assertEqual([obstacle["id"] for obstacle in selected], [2, 1])

    def test_uses_ego_heading_to_find_map_frame_left_lane(self):
        selected = _select_left(
            [
                _map_obstacle(10, 6.5, 25.0),
                _map_obstacle(11, 13.5, 25.0),
            ],
            ego_x=10.0,
            ego_y=20.0,
            ego_yaw=0.5 * math.pi,
        )

        self.assertEqual([obstacle["id"] for obstacle in selected], [10])


class ParallelDynamicLaneSelectionTest(unittest.TestCase):
    def test_includes_left_lane_front_side_and_rear(self):
        selected = _select_left_parallel(
            [
                _map_obstacle(1, 12.0, 3.5, velocity_x=6.0),
                _map_obstacle(2, 0.0, 3.5, velocity_x=5.0),
                _map_obstacle(3, -12.0, 3.5, velocity_x=4.0),
            ]
        )

        self.assertEqual([obstacle["id"] for obstacle in selected], [3, 2, 1])

    def test_rejects_static_stopped_and_unknown_objects(self):
        selected = _select_left_parallel(
            [
                _map_obstacle(
                    1, 5.0, 3.5, velocity_x=0.0, motion_state="STATIC"
                ),
                _map_obstacle(
                    2, 6.0, 3.5, velocity_x=0.0, motion_state="STOPPED"
                ),
                _map_obstacle(
                    3, 7.0, 3.5, velocity_x=2.0, motion_state="UNKNOWN"
                ),
            ]
        )

        self.assertEqual(selected, [])

    def test_rejects_low_speed_tracker_jitter(self):
        selected = _select_left_parallel(
            [_map_obstacle(1, 5.0, 3.5, velocity_x=0.6)]
        )

        self.assertEqual(selected, [])

    def test_rejects_crossing_and_opposite_direction_objects(self):
        selected = _select_left_parallel(
            [
                _map_obstacle(1, 5.0, 3.5, velocity_x=0.0, velocity_y=4.0),
                _map_obstacle(2, 7.0, 3.5, velocity_x=-5.0),
            ]
        )

        self.assertEqual(selected, [])

    def test_rejects_ego_lane_and_right_lane_movers(self):
        selected = _select_left_parallel(
            [
                _map_obstacle(1, 5.0, 0.0, velocity_x=5.0),
                _map_obstacle(2, 5.0, -3.5, velocity_x=5.0),
            ]
        )

        self.assertEqual(selected, [])

    def test_uses_ego_heading_for_lane_and_parallel_motion(self):
        selected = _select_left_parallel(
            [
                _map_obstacle(
                    1, 6.5, 25.0, velocity_x=0.0, velocity_y=5.0
                ),
                _map_obstacle(
                    2, 13.5, 25.0, velocity_x=0.0, velocity_y=5.0
                ),
            ],
            ego_x=10.0,
            ego_y=20.0,
            ego_yaw=0.5 * math.pi,
        )

        self.assertEqual([obstacle["id"] for obstacle in selected], [1])


if __name__ == "__main__":
    unittest.main()
