"""Geometry, hand-over and simple bicycle-model regressions."""
import math
import unittest

from purepursuit_mgeo.motion import diagonal_progress, SteeringRateLimiter
from purepursuit_mgeo.path import MgeoPurePursuit, PathPoint
import test_highway_safety as safety
from test_highway_safety import Stamp, path_at


def node_fixture():
    fixture = safety.HighwaySafetyTest()
    fixture.setUp()
    n = fixture.node
    n._boundary_local = lambda key: [(float(x), 1.75) for x in (0, 5, 10, 15, 20, 25)]
    return n


class DiagonalLaneChangeTest(unittest.TestCase):
    def test_profile_is_monotone_with_constant_middle_slope(self):
        values = [diagonal_progress(i/1000) for i in range(1001)]
        self.assertEqual(values[0], 0.0)
        self.assertEqual(values[-1], 1.0)
        self.assertTrue(all(b >= a for a, b in zip(values, values[1:])))
        for u in (0.25, 0.4, 0.5, 0.6, 0.75):
            slope = (diagonal_progress(u+.001)-diagonal_progress(u-.001))/.002
            self.assertAlmostEqual(slope, 1.25)
        self.assertLess(diagonal_progress(.001)/.001, .001)

    def test_path_has_shallow_heading_and_one_lane_displacement(self):
        n = node_fixture()
        for width in (2.7, 3.5, 4.2):
            with self.subTest(width=width):
                n._boundary_local = lambda key: [(float(x), width/2) for x in (0, 5, 10, 15, 20, 25)]
                points, length = n._generate_lane_change_local(width, 2.0)
                self.assertEqual(points[0], (0.0, 0.0))
                self.assertAlmostEqual(points[-1][1], width)
                headings = [math.atan2(b[1]-a[1], b[0]-a[0]) for a, b in zip(points, points[1:])]
                self.assertLessEqual(max(headings), math.radians(8.0)+1e-5)
                self.assertLess(abs(headings[-1]), 1e-6)
                self.assertTrue(n._path_curvature_ok(points, 6.0)[0])
                self.assertLessEqual(length, 40.0)

    def test_insufficient_length_rejects_steep_candidate(self):
        n = node_fixture()
        n.change_max_length_m = 20.0
        points, _ = n._generate_lane_change_local(3.5, 2.0)
        self.assertEqual(points, [])

    def test_ego_offset_does_not_create_sharp_entry(self):
        n = node_fixture()
        n._boundary_local = lambda key: [(float(x), 2.55) for x in (0, 5, 10, 15, 20, 25)]
        points, _ = n._generate_lane_change_local(3.5, 2.0)
        headings = [math.atan2(b[1]-a[1], b[0]-a[0]) for a, b in zip(points, points[1:])]
        self.assertLessEqual(max(headings), math.radians(8.0)+1e-5)
        self.assertAlmostEqual(points[-1][1], 4.3)

    def test_live_lidar_obstacle_is_routed_by_rrt_star(self):
        n = node_fixture()
        n.cruise_speed_mps = 4.0
        n.latest_obstacles.obstacles = [safety.obstacle(25.0,0.0)]

        points, _ = n._generate_lane_change_local(3.5,4.0)

        self.assertGreater(len(points),3)
        self.assertEqual(n.last_rrt_diag["planner"],"rrt_star")
        self.assertEqual(n.last_rrt_diag["source"],"live_lidar")
        self.assertIn(1,n.last_rrt_diag["obstacles"])
        self.assertGreater(n.last_rrt_diag["raw_points"],2)
        self.assertTrue(n._path_curvature_ok(points,4.0)[0])

    def test_distance_alone_cannot_complete_lane_change(self):
        n = node_fixture()
        n.state = n.LANE_CHANGE
        n.committed_path = path_at(3.5)
        n.change_travel_m = 42.0
        n.complete_since = Stamp(98.0)
        n._odom_pose.return_value = (42.0, 2.5, 0.0, 2.0)
        n._tick(None)
        self.assertEqual(n.state, n.LANE_CHANGE)
        self.assertFalse(n._publish.call_args.args[4]['aligned'])

    def test_wrong_heading_cannot_complete_lane_change(self):
        n = node_fixture()
        n.state = n.LANE_CHANGE
        n.committed_path = path_at(3.5)
        n.change_travel_m = 42.0
        n.complete_since = Stamp(98.0)
        n._odom_pose.return_value = (42.0, 3.5, math.radians(20.0), 2.0)
        n._tick(None)
        self.assertEqual(n.state, n.LANE_CHANGE)

    def test_aligned_pose_can_complete_lane_change(self):
        n = node_fixture()
        n.state = n.LANE_CHANGE
        n.committed_path = path_at(3.5)
        n.change_travel_m = 42.0
        n.complete_since = Stamp(98.0)
        n._odom_pose.return_value = (42.0, 3.5, 0.0, 2.0)
        n._tick(None)
        self.assertEqual(n.state, n.INNER_HOLD)

    def test_first_camera_correction_is_blended(self):
        n = node_fixture()
        n._centerline_local.return_value = [(0.0, 0.0)] + [(float(x), -0.6) for x in range(5, 26)]
        path, reason = n._filtered_inner_path(Stamp(), .05)
        self.assertEqual(reason, 'ok')
        # The first update may correct faster than the former 0.35 s filter,
        # but must still apply less than a quarter of the observed jump.
        self.assertLess(abs(path.poses[1].pose.position.y), .15)

    def test_inner_path_converges_at_controller_lookahead(self):
        n = node_fixture()
        n.last_inner_path = path_at()
        n._centerline_local.return_value = [(0.0, 0.0)] + [(float(x), -0.8) for x in range(5, 26)]
        for _ in range(20):
            path, _ = n._filtered_inner_path(Stamp(), .05)
            n.last_inner_path = path
        self.assertLess(path.poses[1].pose.position.y, -0.5)

    def test_control_center_is_midpoint_of_physical_boundaries(self):
        n = node_fixture()
        n._boundary_local = safety.NODE.HighwayLaneStrategyNode._boundary_local.__get__(n)
        n._centerline_local = safety.NODE.HighwayLaneStrategyNode._centerline_local.__get__(n)
        n.lane_info = {
            'lane_width_m': 3.5,
            # Deliberately biased derived data: control should use boundaries.
            'centerline_points': [[float(x), 1.6] for x in range(5, 26)],
            'left_boundary_points': [[float(x), 1.8] for x in range(5, 26)],
            'right_boundary_points': [[float(x), -1.7] for x in range(5, 26)],
        }
        center = n._centerline_local()
        self.assertAlmostEqual(center[1][1], 0.05)
        self.assertTrue(all(abs(y-0.05) < 1e-6 for _, y in center[1:]))

    def test_physical_boundary_midpoint_is_primary_hold_path(self):
        n = node_fixture()
        n._boundary_local = safety.NODE.HighwayLaneStrategyNode._boundary_local.__get__(n)
        n._centerline_local = safety.NODE.HighwayLaneStrategyNode._centerline_local.__get__(n)
        n.lane_info = {
            'lane_width_m': 3.5,
            # Even a plausible but biased reported center must not move the
            # vehicle away from the two physical boundaries' midpoint.
            'centerline_points': [[float(x), 0.25] for x in range(5, 26)],
            'left_boundary_points': [[float(x), 1.8] for x in range(5, 26)],
            'right_boundary_points': [[float(x), -1.7] for x in range(5, 26)],
            'straddling_lane': None,
        }
        center = n._centerline_local()
        self.assertAlmostEqual(center[1][1], 0.05)

    def test_post_change_center_requires_fresh_two_boundary_measurement(self):
        n = node_fixture()
        n._boundary_local = safety.NODE.HighwayLaneStrategyNode._boundary_local.__get__(n)
        n._centerline_local = safety.NODE.HighwayLaneStrategyNode._centerline_local.__get__(n)
        n._inner_center_sanity = safety.NODE.HighwayLaneStrategyNode._inner_center_sanity.__get__(n)
        n.lane_info = {
            'output_status': 'FRESH',
            'lane_width_m': 3.5,
            'left_lane': {'detected': True, 'coasted': False, 'from_guide': False},
            'right_lane': {'detected': True, 'coasted': False, 'from_guide': False},
            'left_boundary_points': [[float(x), 1.8] for x in range(5, 26)],
            'right_boundary_points': [[float(x), -1.7] for x in range(5, 26)],
            'centerline_points': [[float(x), 0.4] for x in range(5, 26)],
        }
        valid, reason, center_y = n._inner_center_sanity(require_two_boundaries=True)
        self.assertTrue(valid)
        self.assertEqual(reason, 'ok')
        self.assertAlmostEqual(center_y, 0.05)

        n.lane_info['output_status'] = 'HELD'
        self.assertEqual(
            n._inner_center_sanity(require_two_boundaries=True),
            (False, 'inner_center_not_fresh', None),
        )

        n.lane_info['output_status'] = 'FRESH'
        n.lane_info['right_lane']['detected'] = False
        self.assertEqual(
            n._inner_center_sanity(require_two_boundaries=True),
            (False, 'inner_right_boundary_missing', None),
        )

    def test_post_change_center_cannot_jump_back_toward_previous_lane(self):
        n = node_fixture()
        n._boundary_local = safety.NODE.HighwayLaneStrategyNode._boundary_local.__get__(n)
        n._centerline_local = safety.NODE.HighwayLaneStrategyNode._centerline_local.__get__(n)
        n._inner_center_sanity = safety.NODE.HighwayLaneStrategyNode._inner_center_sanity.__get__(n)
        n._path_map_to_local = safety.NODE.HighwayLaneStrategyNode._path_map_to_local.__get__(n)
        n._odom_pose.return_value = (10.0, 3.5, 0.0, 2.0)
        n.last_inner_path = path_at(3.5)
        n.lane_info = {
            'output_status': 'FRESH',
            'lane_width_m': 3.5,
            'left_lane': {'detected': True},
            'right_lane': {'detected': True},
            # A plausible-width pair centered 0.8 m to the right is still the
            # wrong hand-over target relative to the committed RRT lane.
            'left_boundary_points': [[float(x), 0.95] for x in range(5, 26)],
            'right_boundary_points': [[float(x), -2.55] for x in range(5, 26)],
            'centerline_points': [[float(x), -0.8] for x in range(5, 26)],
        }
        valid, reason, center_y = n._inner_center_sanity(require_two_boundaries=True)
        self.assertFalse(valid)
        self.assertEqual(reason, 'inner_center_wrong_lane')
        self.assertAlmostEqual(center_y, -0.8)

    def test_straddling_line_is_not_used_as_lane_center(self):
        n = node_fixture()
        n._centerline_local = safety.NODE.HighwayLaneStrategyNode._centerline_local.__get__(n)
        n.lane_info = {
            'straddling_lane': {'detected': True},
            'centerline_points': [[float(x), 0.0] for x in range(5, 26)],
        }
        self.assertEqual(n._centerline_local(), [(0.0, 0.0)])

    def test_camera_jump_is_blended_without_delayed_stop(self):
        n = node_fixture()
        n._global_signed_d.return_value = 3.5
        n._centerline_local.return_value = [(0.0, 0.0)] + [(float(x), -0.8) for x in range(5, 26)]
        n._tick(None)
        path, stop, _, _, status, *_ = n._publish.call_args.args
        self.assertFalse(stop)
        self.assertEqual(status['reason'], 'ok')
        self.assertLess(abs(path.poses[1].pose.position.y), .20)
        n.lane_invalid_since = Stamp(98.0)
        n._tick(None)
        self.assertFalse(n._publish.call_args.args[1])
        self.assertIsNone(n.lane_invalid_since)

    def test_allowed_handover_heading_does_not_reject_inner_path(self):
        n = node_fixture()
        n.last_inner_path = path_at(3.5)
        n._odom_pose.return_value = (42.0, 3.5, math.radians(8.0), 2.0)
        n._centerline_local.return_value = [(float(x), 0.0) for x in range(41)]
        path, reason = n._filtered_inner_path(Stamp(), .05)
        self.assertIsNotNone(path)
        self.assertEqual(reason, 'limited')

    def test_expired_committed_path_still_gets_continuous_inner_path(self):
        n = node_fixture()
        n.last_inner_path = path_at(3.5, end=5)
        n._odom_pose.return_value = (6.0, 3.5, 0.0, 2.0)
        n._centerline_local.return_value = [(float(x), 0.0) for x in range(41)]
        path, _ = n._filtered_inner_path(Stamp(), .05)
        self.assertGreater(len(path.poses), 20)

    def test_path_blending_accounts_for_vehicle_motion(self):
        n = node_fixture()
        n.last_inner_path = path_at(3.5)
        n._odom_pose.return_value = (10.0, 3.5, 0.0, 2.0)
        path, reason = n._filtered_inner_path(Stamp(), .05)
        self.assertEqual(reason, 'ok')
        self.assertTrue(all(abs(p.pose.position.y-3.5) < 1e-6 for p in path.poses))

    def test_bicycle_tracks_diagonal_at_low_and_cruise_speed(self):
        for speed in (2.0, 6.0):
            with self.subTest(speed=speed):
                n = node_fixture()
                points, length = n._generate_lane_change_local(3.5, speed)
                pp = MgeoPurePursuit([PathPoint(x, y, 0) for x, y in points], 3.0, 4.0, .35, 1.5)
                limiter = SteeringRateLimiter(.2)
                x = y = yaw = 0.0
                peak_yaw = 0.0
                for step in range(1000):
                    target, stop, *_ = pp.compute(x, y, yaw, speed)
                    self.assertFalse(stop)
                    angle = limiter.update(target, step*.05)
                    x += speed*math.cos(yaw)*.05
                    y += speed*math.sin(yaw)*.05
                    yaw += speed/3.0*math.tan(angle)*.05
                    peak_yaw = max(peak_yaw, abs(yaw))
                    if x >= n.change_start_m+length+7:
                        break
                else:
                    self.fail('did not finish diagonal transition')
                self.assertLess(peak_yaw, math.radians(10))
                self.assertLess(abs(y-3.5), .2)
                self.assertLess(abs(yaw), math.radians(3))


class SteeringLimitTest(unittest.TestCase):
    def test_rate_is_independent_of_control_frequency(self):
        for hz in (10, 20, 40):
            limiter = SteeringRateLimiter(.2, 1/hz)
            for i in range(hz):
                angle = limiter.update(.6, i/hz)
            self.assertAlmostEqual(angle, .2)

    def test_sign_reversal_is_limited(self):
        limiter = SteeringRateLimiter(.2)
        limiter.update(.3, 0, enabled=False)
        self.assertAlmostEqual(limiter.update(-.3, .05), .29)

    def test_stop_and_restart(self):
        limiter = SteeringRateLimiter(.2)
        limiter.update(.4, 0, enabled=False)
        self.assertEqual(limiter.reset(.05), 0)
        self.assertAlmostEqual(limiter.update(.4, .10), .01)

    def test_disabled_mode_tracks_current_command(self):
        limiter = SteeringRateLimiter(.2)
        self.assertEqual(limiter.update(.4, 0, enabled=False), .4)
        self.assertAlmostEqual(limiter.update(-.4, .05), .39)


if __name__ == '__main__':
    unittest.main()
