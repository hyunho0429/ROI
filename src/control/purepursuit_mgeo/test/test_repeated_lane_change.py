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
                n._choose_lane_change.reset_mock()
                self.tick()
                self.assertEqual(n.state, n.INNER_HOLD)
                self.assertIsNone(n.ready_since)
                n._choose_lane_change.assert_not_called()

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
