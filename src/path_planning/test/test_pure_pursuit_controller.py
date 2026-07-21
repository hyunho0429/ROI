#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from path_planning.pure_pursuit_controller import PurePursuitController
from path_planning.stanley_controller import (
    PathPoint,
    SteeringCommandFilter,
    load_path_csv,
)


def controller(points, **kwargs):
    defaults = dict(
        wheelbase_m=2.7,
        lookahead_distance_m=5.0,
        lookahead_speed_gain_s=0.0,
        minimum_lookahead_m=1.0,
        maximum_lookahead_m=20.0,
        max_steering_deg=45.0,
        minimum_waypoint_spacing_m=0.0,
        waypoint_smoothing_window=1,
    )
    defaults.update(kwargs)
    return PurePursuitController(points, **defaults)


class PurePursuitControllerTest(unittest.TestCase):
    def test_straight_target_produces_zero_steering(self):
        pursuit = controller([PathPoint(0.0, 0.0), PathPoint(20.0, 0.0)])
        result = pursuit.compute(0.0, 0.0, 0.0, 0.0, 0.0)

        self.assertAlmostEqual(result.lookahead_distance_m, 5.0)
        self.assertAlmostEqual(result.target_position_m[0], 5.0)
        self.assertAlmostEqual(result.target_position_m[1], 0.0)
        self.assertAlmostEqual(result.alpha_rad, 0.0)
        self.assertAlmostEqual(result.curvature_inv_m, 0.0)
        self.assertAlmostEqual(result.steering_rad, 0.0)

    def test_reference_curvature_is_converted_to_bicycle_steering(self):
        pursuit = controller([PathPoint(0.0, 0.0), PathPoint(10.0, 10.0)])
        result = pursuit.compute(0.0, 0.0, 0.0, 0.0, 0.0)

        expected_curvature = 2.0 * math.sin(math.pi / 4.0) / 5.0
        self.assertAlmostEqual(result.alpha_rad, math.pi / 4.0)
        self.assertAlmostEqual(result.curvature_inv_m, expected_curvature)
        self.assertAlmostEqual(
            result.steering_rad, math.atan(2.7 * expected_curvature)
        )

    def test_right_hand_target_produces_negative_steering(self):
        pursuit = controller([PathPoint(0.0, 0.0), PathPoint(10.0, -10.0)])
        result = pursuit.compute(0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertLess(result.alpha_rad, 0.0)
        self.assertLess(result.steering_rad, 0.0)

    def test_positive_path_lateral_offset_moves_path_left(self):
        pursuit = controller(
            [PathPoint(0.0, 0.0), PathPoint(20.0, 0.0)],
            path_lateral_offset_m=1.0,
        )
        result = pursuit.compute(0.0, 0.0, 0.0, 0.0, 0.0)

        self.assertAlmostEqual(pursuit.points[0].y_m, 1.0)
        self.assertLess(result.cross_track_error_m, 0.0)
        self.assertGreater(result.steering_rad, 0.0)

    def test_speed_increases_lookahead_with_configured_limits(self):
        pursuit = controller(
            [PathPoint(0.0, 0.0), PathPoint(30.0, 0.0)],
            lookahead_distance_m=4.0,
            lookahead_speed_gain_s=0.5,
            minimum_lookahead_m=3.0,
            maximum_lookahead_m=12.0,
        )
        result = pursuit.compute(0.0, 0.0, 0.0, 0.0, 4.0)
        self.assertAlmostEqual(result.lookahead_distance_m, 6.0)

    def test_goal_stops_at_path_end(self):
        pursuit = controller([PathPoint(0.0, 0.0), PathPoint(10.0, 0.0)])
        result = pursuit.compute(10.0, 0.0, 0.0, 0.0, 0.0)
        self.assertTrue(result.goal_reached)
        self.assertEqual(result.steering_rad, 0.0)

    def test_competition_route_selects_a_forward_target(self):
        filename = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "data",
                "2026_molit_comp_global_path.txt",
            )
        )
        points = load_path_csv(filename)
        pursuit = PurePursuitController(points)
        yaw = math.atan2(
            points[1].y_m - points[0].y_m,
            points[1].x_m - points[0].x_m,
        )
        result = pursuit.compute(
            points[0].x_m,
            points[0].y_m,
            points[0].z_m,
            yaw,
            10.0 / 3.6,
        )

        self.assertFalse(result.goal_reached)
        self.assertGreater(result.target_index, result.segment_index)
        self.assertGreater(result.lookahead_distance_m, 4.0)
        self.assertTrue(math.isfinite(result.steering_rad))

    def test_competition_route_completes_bicycle_model_lap(self):
        filename = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "data",
                "2026_molit_comp_global_path.txt",
            )
        )
        points = load_path_csv(filename)
        pursuit = PurePursuitController(points)
        steering_filter = SteeringCommandFilter(
            alpha=0.25,
            max_rate_radps=0.4,
            max_abs_rad=pursuit.max_steering_rad,
        )
        x_m, y_m, z_m = points[0].x_m, points[0].y_m, points[0].z_m
        yaw_rad = math.atan2(
            points[1].y_m - y_m,
            points[1].x_m - x_m,
        )
        speed_mps = 10.0 / 3.6
        dt = 0.05
        maximum_cross_track_error = 0.0

        for step in range(18000):
            result = pursuit.compute(
                x_m, y_m, z_m, yaw_rad, speed_mps, wheelbase_m=2.7
            )
            if result.goal_reached:
                break
            steering = steering_filter.update(result.steering_rad, step * dt)
            x_m += speed_mps * math.cos(yaw_rad) * dt
            y_m += speed_mps * math.sin(yaw_rad) * dt
            yaw_rad += speed_mps / 2.7 * math.tan(steering) * dt
            yaw_rad = (yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
            maximum_cross_track_error = max(
                maximum_cross_track_error, abs(result.cross_track_error_m)
            )
        else:
            self.fail("Pure Pursuit did not finish the competition route")

        self.assertLess(maximum_cross_track_error, 0.5)
        self.assertLess(step * dt, 800.0)


if __name__ == "__main__":
    unittest.main()
