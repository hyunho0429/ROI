"""Repeat only after lane settling and a fresh, uninterrupted safe gap."""
import math
import unittest
from unittest.mock import Mock, patch

import test_highway_safety as safety


class RepeatedLaneChangeTest(unittest.TestCase):
    def setUp(self):
        fixture = safety.HighwaySafetyTest()
        fixture.setUp()
        self.node = fixture.node
        n = self.node
        n.min_lane_hold_before_next_change_m = 8.0
        n.min_lane_hold_before_next_change_s = 5.0
        n.inner_hold_started_at = safety.Stamp(90.0)
        n.next_change_centered_since = safety.Stamp(90.0)
        n._global_signed_d.return_value = 3.5
        n._choose_lane_change = Mock(return_value=(safety.path_at(3.5), 2.0, 32.0, 'ok', {}))

    def tick(self, now=100.0):
        n = self.node
        stamp = safety.Stamp(now)
        n.base_path_at = n.base_stop_at = n.odom_at = n.obstacles_at = stamp
        with patch.object(safety.NODE.rospy.Time, 'now', return_value=stamp):
            n._tick(None)

    def test_no_second_change_before_eight_metres(self):
        n = self.node
        n.inner_hold_travel_m = 7.9
        n.ready_since = safety.Stamp(95.0)
        self.tick()
        self.assertEqual(n.state, n.INNER_HOLD)
        self.assertIsNone(n.ready_since)
        n._choose_lane_change.assert_not_called()

    def test_no_second_change_before_five_seconds(self):
        n = self.node
        n.next_change_centered_since = safety.Stamp(98.0)
        n.ready_since = safety.Stamp(95.0)
        self.tick(100.0)
        self.assertEqual(n.state, n.INNER_HOLD)
        self.assertIsNone(n.ready_since)
        n._choose_lane_change.assert_not_called()

    def test_five_second_hold_starts_after_lane_center_is_stable(self):
        n = self.node
        n.next_change_centered_since = None

        self.tick(100.0)
        self.assertEqual(n.state, n.INNER_HOLD)
        self.assertEqual(n.next_change_centered_since.seconds, 100.0)
        n._choose_lane_change.assert_not_called()

        self.tick(104.9)
        self.assertEqual(n.state, n.INNER_HOLD)
        n._choose_lane_change.assert_not_called()

        self.tick(105.1)
        self.assertIsNotNone(n.ready_since)
        self.tick(105.7)
        self.assertEqual(n.state, n.LANE_CHANGE)

    def test_second_change_requires_new_half_second_confirmation(self):
        n = self.node
        self.tick(100.0)
        self.tick(100.4)
        self.assertEqual(n.state, n.INNER_HOLD)
        self.tick(100.6)
        self.assertEqual(n.state, n.LANE_CHANGE)
        self.assertEqual(n._publish.call_args.args[4]['reason'], 'next_left_lane_change')

    def test_third_change_is_allowed_after_a_new_confirmation(self):
        n = self.node
        n.lane_changes_done = 2
        self.tick(100.0)
        self.assertEqual(n.state, n.INNER_HOLD)
        self.tick(100.6)
        self.assertEqual(n.state, n.LANE_CHANGE)
        self.assertEqual(n._publish.call_args.args[4]['reason'], 'next_left_lane_change')

    def test_two_left_solids_lock_further_changes_and_hold_lane(self):
        n = self.node
        n.lane_info.update({
            'lane_valid': True,
            'output_status': 'FRESH',
            'left_lane': {
                'detected': True,
                'type': 'white_solid',
                'from_guide': False,
                'coasted': False,
                'age': 3,
                'coef': [0.0, 1.75],
                'x_range_m': [0.0, 30.0],
            },
            'right_lane': {
                'detected': True,
                'type': 'white_dashed',
                'from_guide': False,
                'coasted': False,
            },
            'left_outer_lane': {
                'detected': True,
                'type': 'white_solid',
                'from_guide': False,
                'coasted': False,
                'age': 3,
                'coef': [0.0, 5.25],
                'x_range_m': [0.0, 30.0],
            },
        })
        n._global_signed_d.return_value = 0.2
        n._choose_lane_change = Mock(return_value=(
            safety.path_at(3.5), 2.0, 32.0, 'ok', {}
        ))

        self.tick(100.0)
        self.assertFalse(n.lane_change_locked_by_left_solid)
        n._choose_lane_change.assert_not_called()
        n._generate_rejoin_path.assert_not_called()
        self.assertEqual(
            n._publish.call_args.args[4]['reason'],
            'final_lane_confirming',
        )

        fresh = safety.Stamp(100.6)
        n.base_path_at = n.base_stop_at = n.odom_at = n.obstacles_at = fresh
        self.tick(100.6)

        self.assertTrue(n.lane_change_locked_by_left_solid)
        self.assertEqual(n.state, n.INNER_HOLD)
        self.assertFalse(n._publish.call_args.args[1])
        self.assertTrue(n._publish.call_args.args[3])
        self.assertEqual(
            n._publish.call_args.args[4]['reason'],
            'final_lane_center_hold',
        )
        self.assertFalse(
            n._publish.call_args.args[4]['lane_change_enabled']
        )
        n._choose_lane_change.assert_not_called()
        n._generate_rejoin_path.assert_not_called()

    def test_single_left_solid_does_not_lock_intermediate_lane(self):
        n = self.node
        n.lane_info.update({
            'lane_valid': True,
            'output_status': 'FRESH',
            'left_lane': {
                'detected': True,
                'type': 'white_solid',
                'age': 3,
                'coef': [0.0, 1.75],
                'x_range_m': [0.0, 30.0],
            },
            'right_lane': {'detected': False, 'type': None},
        })

        self.assertFalse(n._final_lane_markings_present())

    def test_far_left_solid_does_not_block_change_across_adjacent_dashed(self):
        n = self.node
        n.lane_info.update({
            'lane_valid': True,
            'output_status': 'FRESH',
            'left_lane': {
                'detected': True,
                'type': 'white_solid',
                'age': 3,
                # A solid line beyond the adjacent divider must not mark the
                # current lane as the final solid-left/dashed-right lane.
                'coef': [0.0, 5.25],
                'x_range_m': [0.0, 30.0],
            },
            'right_lane': {'detected': True, 'type': 'white_dashed'},
        })

        self.assertFalse(n._final_lane_markings_present())

    def test_outer_solid_marks_next_lane_as_final_before_crossing(self):
        n = self.node
        n.lane_info.update({
            'lane_valid': True,
            'output_status': 'FRESH',
            'left_lane': {
                'detected': True, 'type': 'white_dashed', 'age': 3,
                'coef': [0.0, 1.75], 'x_range_m': [0.0, 30.0],
            },
            'left_outer_lane': {
                'detected': True, 'type': 'white_solid', 'age': 3,
                'coef': [0.0, 5.25], 'x_range_m': [0.0, 30.0],
            },
        })

        self.assertTrue(n._target_lane_has_solid_left_boundary())

        n.lane_info['left_outer_lane']['type'] = 'white_dashed'
        self.assertFalse(n._target_lane_has_solid_left_boundary())

    def test_final_target_commit_locks_out_a_third_change_on_completion(self):
        n = self.node
        n.committed_enters_final_lane = True

        n._enter_inner_hold(
            safety.Stamp(100.0), 0.0, 0.0, 'target_lane_capture', 8.0
        )

        self.assertTrue(n.lane_change_locked_by_left_solid)
        self.assertEqual(n.state, n.INNER_HOLD)

    def test_two_fresh_left_solids_lock_further_lane_changes(self):
        n = self.node
        n.lane_info.update({
            'lane_valid': True,
            'output_status': 'FRESH',
            'left_lane': {
                'detected': True, 'type': 'white_solid', 'age': 3,
                'coef': [0.0, 1.75], 'x_range_m': [0.0, 30.0],
            },
            'left_outer_lane': {
                'detected': True, 'type': 'white_solid', 'age': 3,
                'coef': [0.0, 5.25], 'x_range_m': [0.0, 30.0],
            },
            'right_lane': {'detected': False, 'type': None},
        })

        self.assertTrue(n._double_left_solid_present())
        self.assertTrue(n._final_lane_markings_present())

    def test_final_lane_continues_last_verified_center_after_camera_dropout(self):
        n = self.node
        n.lane_change_locked_by_left_solid = True
        n._lane_valid.return_value = (False, 'lane_invalid')
        n.lane_invalid_since = safety.Stamp(98.0)

        self.tick(100.0)

        self.assertFalse(n._publish.call_args.args[1])
        self.assertEqual(
            n._publish.call_args.args[4]['reason'],
            'lane_fallback_lane_invalid',
        )

    def test_control_center_or_heading_error_delays_second_change(self):
        n = self.node
        for center_y, heading in ((0.6, 0), (0, math.radians(15))):
            with self.subTest(center_y=center_y, heading=heading):
                n.state = n.INNER_HOLD
                n._inner_center_sanity.return_value = (True, 'ok', center_y)
                slope = math.tan(heading)
                n._centerline_local.return_value = [
                    (float(x), slope*float(x)) for x in range(51)
                ]
                n.ready_since = safety.Stamp(95.0)
                n.next_change_centered_since = safety.Stamp(90.0)
                n.next_change_center_lost_since = None
                n._choose_lane_change.reset_mock()
                self.tick()
                self.assertEqual(n.state, n.INNER_HOLD)
                self.assertIsNone(n.ready_since)
                self.assertIsNotNone(n.next_change_centered_since)
                n._choose_lane_change.assert_not_called()
                self.tick(100.5)
                self.assertIsNone(n.next_change_centered_since)

    def test_offset_filtered_path_delays_second_change(self):
        n = self.node
        n._filtered_inner_path = Mock(
            return_value=(safety.path_at(0.35), 'ok')
        )
        n.ready_since = safety.Stamp(95.0)
        n.next_change_centered_since = safety.Stamp(90.0)
        n.next_change_center_lost_since = None

        self.tick()

        self.assertEqual(n.state, n.INNER_HOLD)
        self.assertIsNone(n.ready_since)
        self.assertIsNotNone(n.next_change_centered_since)
        n._choose_lane_change.assert_not_called()
        self.assertAlmostEqual(
            n._publish.call_args.args[4]['control_path_y5_m'], 0.35
        )
        self.tick(100.5)
        self.assertIsNone(n.next_change_centered_since)

    def test_brief_camera_dropout_preserves_centered_lane_timer(self):
        n = self.node
        n.next_change_centered_since = safety.Stamp(95.0)
        n.lane_info['output_status'] = 'HELD'

        self.tick(100.0)
        self.assertEqual(n.next_change_centered_since.seconds, 95.0)
        n._choose_lane_change.assert_not_called()

        n.lane_info['output_status'] = 'FRESH'
        self.tick(100.2)
        self.assertEqual(n.next_change_centered_since.seconds, 95.0)
        self.assertIsNotNone(n.ready_since)

    def test_stale_reported_errors_do_not_reverse_next_change(self):
        n = self.node
        n.lane_info.update(
            lateral_error_m=0.8,
            heading_error_rad=math.radians(15),
        )
        # The actual midpoint path is centered and straight. The next command
        # must therefore remain the planned LEFT change.
        self.tick(100.0)
        self.tick(100.6)
        self.assertEqual(n.state, n.LANE_CHANGE)
        self.assertEqual(
            n._publish.call_args.args[4]['reason'],
            'next_left_lane_change',
        )

    def test_emergency_resets_gap_confirmation(self):
        n = self.node
        n.latest_obstacles.obstacles = [safety.obstacle(7.0)]
        n.ready_since = safety.Stamp(95.0)
        self.tick()
        self.assertTrue(n._publish.call_args.args[1])
        self.assertIsNone(n.ready_since)
        n._choose_lane_change.assert_not_called()

    def test_unsafe_gap_keeps_following_and_resets_confirmation(self):
        n = self.node
        n._choose_lane_change.return_value = (None, None, None, 'rear_gap', {})
        n.ready_since = safety.Stamp(95.0)
        self.tick()
        self.assertEqual(n.state, n.INNER_HOLD)
        self.assertFalse(n._publish.call_args.args[1])
        self.assertIsNone(n.ready_since)

    def test_camera_held_values_do_not_arm_second_change(self):
        n = self.node
        n.lane_info['output_status'] = 'HELD'
        self.tick()
        n._choose_lane_change.assert_not_called()

    def test_one_change_can_finish_when_road_converges(self):
        n = self.node
        n._global_signed_d.return_value = 1.0
        n._choose_lane_change.return_value = (None, None, None, 'left_not_dashed', {})
        self.tick()
        self.assertEqual(n.state, n.REJOIN)
        self.assertEqual(n.lane_changes_done, 1)

    def test_pending_second_change_does_not_rejoin_mid_confirmation(self):
        n = self.node
        n._global_signed_d.return_value = 1.0
        self.tick()
        self.assertEqual(n.state, n.INNER_HOLD)
        self.assertIsNotNone(n.ready_since)
        n._generate_rejoin_path.assert_not_called()


if __name__ == '__main__':
    unittest.main()
