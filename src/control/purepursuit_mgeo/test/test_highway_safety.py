#!/usr/bin/env python3
"""Safety regressions using the real node with ROS transport stubbed out.

Run: python -m unittest discover -s src/control/purepursuit_mgeo/test -v
Set PYTHONPATH to src/control/purepursuit_mgeo/src first.
No ROS master, simulator, or control publisher is used.
"""
import importlib.util
import math
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch


class Stamp:
    def __init__(self, seconds=100.0):
        self.seconds = seconds

    @staticmethod
    def now():
        return Stamp()

    def __sub__(self, other):
        return Stamp(self.seconds - other.seconds)

    def to_sec(self):
        return self.seconds


class PoseStamped:
    def __init__(self):
        self.header = NS(stamp=Stamp(), frame_id="map")
        self.pose = NS(position=NS(x=0.0, y=0.0, z=0.0),
                       orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))


class RosPath:
    def __init__(self):
        self.header = NS(stamp=Stamp(), frame_id="map")
        self.poses = []


def path_at(y=0.0, start=0, end=50):
    path = RosPath()
    for x in range(start, end + 1):
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = float(x), y
        path.poses.append(pose)
    return path


def obstacle(x, y=0.0, vx=0.0, vy=0.0):
    return NS(id=1, center_x_map=x, center_y_map=y,
              velocity_x_map=vx, velocity_y_map=vy, length=4.635,
              width=1.892, yaw=0.0)


def load_node():
    rospy = Mock()
    rospy.Time = Stamp
    rospy.get_param.side_effect = lambda name, default=None: default
    modules = {"rospy": rospy}
    for name, values in {
        "geometry_msgs.msg": {"PoseStamped": PoseStamped},
        "nav_msgs.msg": {"Odometry": NS, "Path": RosPath},
        "std_msgs.msg": {"Bool": NS, "Float64": NS, "String": NS},
        "lidar_perception.msg": {"LidarObstacleArray": NS},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        modules[name] = module
    source = Path(os.environ.get("HIGHWAY_NODE_SOURCE", str(
        Path(__file__).resolve().parents[1] / "scripts/highway_lane_strategy_node.py")))
    spec = importlib.util.spec_from_file_location("highway_safety_under_test", source)
    module = importlib.util.module_from_spec(spec)
    # dataclasses resolves annotations through sys.modules while importing.
    with patch.dict(sys.modules, {**modules, spec.name: module}):
        spec.loader.exec_module(module)
    return module


NODE = load_node()


class HighwaySafetyTest(unittest.TestCase):
    def setUp(self):
        with patch.object(NODE, "load_mgeo_path", return_value=[]):
            self.node = NODE.HighwayLaneStrategyNode()
        n = self.node
        n.cruise_speed_mps = 2.0
        # Existing safety cases describe one completed change; repeated-change
        # behavior has its own tests with the new upper bound of two.
        n.target_left_lane_changes = 1
        n._odom_pose = Mock(return_value=(0.0, 0.0, 0.0, 2.0))
        n.latest_odom = NS()
        n.latest_obstacles = NS(obstacles=[])
        n.latest_base_path = path_at()
        n.base_stop = False
        n.base_path_at = n.base_stop_at = n.odom_at = n.obstacles_at = Stamp()
        n.lane_info = {"lane_width_m": 3.5, "lateral_error_m": 0.0, "heading_error_rad": 0.0}
        n._lane_valid = Mock(return_value=(True, "ok"))
        n._inner_center_sanity = Mock(return_value=(True, "ok", 0.0))
        n._centerline_local = Mock(return_value=[(float(x), 0.0) for x in range(51)])
        n._global_signed_d = Mock(return_value=1.0)
        n._publish = Mock()
        n.state = n.INNER_HOLD
        n.lane_changes_done = 1
        n.inner_hold_travel_m = 10.0
        n.last_inner_path = path_at()
        n.committed_path = path_at()
        n.committed_rejoin_path = path_at()
        n._generate_rejoin_path = Mock(return_value=path_at())

    def tick(self):
        self.node._tick(None)
        return self.node._publish.call_args.args

    def test_closer_same_lane_obstacle_remains_emergency(self):
        for x in (7.0, 5.0):
            with self.subTest(x=x):
                self.node.latest_obstacles.obstacles = [obstacle(x)]
                _, emergency, diag = self.node._adaptive_speed(2.0)
                self.assertTrue(emergency)
                self.assertEqual(diag["lead"], 1)

    def test_parallel_adjacent_car_does_not_trigger_lead_stop(self):
        self.node.latest_obstacles.obstacles = [obstacle(5.0, 3.5)]
        speed, emergency, _ = self.node._adaptive_speed(2.0)
        self.assertFalse(emergency)
        self.assertEqual(speed, 2.0)
        self.assertTrue(self.node._dynamic_path_safe(path_at(), 2.0)[0])

    def test_rear_vehicle_does_not_stop_completed_merge(self):
        self.node.latest_obstacles.obstacles = [obstacle(-6.0, vx=12.0)]
        safe, reason = self.node._dynamic_path_safe(path_at(), 2.0)
        self.assertTrue(safe)
        self.assertEqual(reason, "ok")

    def test_dynamic_guard_uses_bounded_replanning_horizon(self):
        self.node.latest_obstacles.obstacles = [obstacle(30.0)]
        self.assertTrue(self.node._dynamic_path_safe(path_at(), 2.0)[0])

    def test_collision_in_first_two_metres_is_not_waived(self):
        self.node.latest_obstacles.obstacles = [obstacle(1.5, vx=-10.0)]
        self.assertFalse(self.node._dynamic_path_safe(path_at(), 2.0)[0])

    def test_committed_path_ignores_already_passed_obstacle(self):
        self.node._odom_pose.return_value = (20.0, 0.0, 0.0, 2.0)
        self.node.latest_obstacles.obstacles = [obstacle(8.0)]
        self.assertTrue(self.node._dynamic_path_safe(path_at(), 2.0)[0])

    def test_prediction_time_restarts_at_current_pose(self):
        self.node._odom_pose.return_value = (20.0, 0.0, 0.0, 2.0)
        # Crossing vehicle reaches x=28,y=0 four seconds from NOW.
        self.node.latest_obstacles.obstacles = [obstacle(28.0, 8.0, vy=-2.0)]
        self.assertFalse(self.node._dynamic_path_safe(path_at(), 2.0)[0])

    def test_lane_change_checks_collision_when_camera_is_stale(self):
        n = self.node
        n.state = n.LANE_CHANGE
        n._lane_valid.return_value = (False, "lane_stale")
        n.latest_obstacles.obstacles = [obstacle(12.0)]
        self.assertTrue(self.tick()[1])

    def test_lane_change_checks_future_target_lane_collision(self):
        n = self.node
        n.state = n.LANE_CHANGE
        n.committed_path = path_at(3.5)
        n.latest_obstacles.obstacles = [obstacle(16.0, 3.5)]
        n._adaptive_speed = Mock(return_value=(2.0, False, {}))
        self.assertTrue(self.tick()[1])

    def test_outer_lane_vehicle_does_not_stop_first_left_change(self):
        n = self.node
        n.state = n.LANE_CHANGE
        diagonal = RosPath()
        for x in range(41):
            pose = PoseStamped()
            pose.pose.position.x = float(x)
            pose.pose.position.y = 3.5*min(1.0, float(x)/20.0)
            diagonal.poses.append(pose)
        n.committed_path = diagonal
        n.latest_obstacles.obstacles = [obstacle(18.0, 7.0)]

        speed, emergency, diagnostics = n._adaptive_speed(2.0)

        self.assertFalse(emergency)
        self.assertEqual(speed, 2.0)
        self.assertIsNone(diagnostics["lead"])
        self.assertEqual(diagnostics["reference"], "map_path")
        self.assertEqual(n._dynamic_path_safe(diagonal,2.0),(True,"ok"))

    def test_single_boundary_can_hold_lane_but_cannot_authorize_change(self):
        n = self.node
        n._boundary_local = NODE.HighwayLaneStrategyNode._boundary_local.__get__(n)
        n._centerline_local = NODE.HighwayLaneStrategyNode._centerline_local.__get__(n)
        n._lane_valid = NODE.HighwayLaneStrategyNode._lane_valid.__get__(n)
        n.lane_info_at = Stamp()
        n.lane_info = {
            "lane_valid": True,
            "left_lane": {
                "detected": True, "confidence": 0.8,
                "from_guide": False, "coasted": False,
            },
            "right_lane": {"detected": False, "confidence": 0.0},
            "left_boundary_points": [[float(x), 1.75] for x in range(5, 26)],
            "right_boundary_points": [],
            "centerline_points": None,
            "lane_width_m": None,
            "heading_error_rad": None,
        }

        self.assertEqual(n._lane_valid(Stamp()), (True, "ok"))
        self.assertEqual(
            n._lane_valid(Stamp(), require_measured_width=True),
            (False, "lane_width"),
        )
        center = n._centerline_local()
        self.assertGreaterEqual(len(center), 3)
        self.assertTrue(all(abs(y) < 1e-6 for _, y in center[1:]))

    def test_guide_lane_cannot_authorize_left_change(self):
        self.node.require_left_dashed = True
        self.node.lane_info = {
            "left_lane": {"detected": True, "dashed": True, "from_guide": True}
        }
        self.assertEqual(self.node._left_dashed_ok(), (False, "left_from_guide"))

    def test_coasted_lane_cannot_authorize_left_change(self):
        self.node.require_left_dashed = True
        self.node.lane_info = {
            "left_lane": {"detected": True, "dashed": True, "coasted": True}
        }
        self.assertEqual(self.node._left_dashed_ok(), (False, "left_coasted"))

    def test_two_dashed_boundaries_are_valid_for_lane_center_hold(self):
        n = self.node
        n._boundary_local = NODE.HighwayLaneStrategyNode._boundary_local.__get__(n)
        n._centerline_local = NODE.HighwayLaneStrategyNode._centerline_local.__get__(n)
        n._lane_valid = NODE.HighwayLaneStrategyNode._lane_valid.__get__(n)
        n.lane_info_at = Stamp()
        n.lane_info = {
            "lane_valid": True,
            "confidence": 0.8,
            "lane_width_m": 3.5,
            "straddling_lane": None,
            "left_lane": {"detected": True,"type": "white_dashed","dashed": True},
            "right_lane": {"detected": True,"type": "white_dashed","dashed": True},
            "left_boundary_points": [[float(x),1.75] for x in range(5,26)],
            "right_boundary_points": [[float(x),-1.75] for x in range(5,26)],
            "centerline_points": [[float(x),0.0] for x in range(5,26)],
        }

        self.assertEqual(n._lane_valid(Stamp()),(True,"ok"))
        self.assertEqual(n._left_dashed_ok(),(True,"ok"))
        self.assertAlmostEqual(n._centerline_local()[1][1],0.0)

    def test_inner_handover_waits_for_stable_new_lane_without_slowing(self):
        n = self.node
        n.inner_handover_pending = True
        n.committed_speed_mps = n.cruise_speed_mps
        n._global_signed_d.return_value = 3.5

        path, stop, speed, _, status, *_ = self.tick()

        self.assertFalse(stop)
        self.assertEqual(speed, n.cruise_speed_mps)
        self.assertTrue(status["lane_handover_pending"])
        self.assertEqual(status["reason"], "lane_handover_confirming")
        self.assertGreater(len(path.poses), 20)

        n.inner_lane_candidate_since = Stamp(99.0)
        self.tick()
        self.assertFalse(n.inner_handover_pending)
        self.assertEqual(n._publish.call_args.args[4]["reason"], "ok")

    def test_dashed_track_id_changes_do_not_reset_center_handover(self):
        n = self.node
        n.inner_handover_pending = True
        n.committed_speed_mps = n.cruise_speed_mps
        n._global_signed_d.return_value = 3.5
        n.lane_info.update({
            "lane_state": "both",
            "left_lane": {"detected": True,"type": "white_dashed","track_id": 10},
            "right_lane": {"detected": True,"type": "white_dashed","track_id": 11},
        })

        with patch.object(NODE.rospy.Time,"now",return_value=Stamp(100.0)):
            n._tick(None)
        self.assertTrue(n.inner_handover_pending)
        self.assertEqual(n.inner_lane_candidate_since.seconds,100.0)

        n.lane_info["left_lane"]["track_id"] = 20
        n.lane_info["right_lane"]["track_id"] = 21
        with patch.object(NODE.rospy.Time,"now",return_value=Stamp(100.2)):
            n._tick(None)
        self.assertTrue(n.inner_handover_pending)
        self.assertEqual(n.inner_lane_candidate_since.seconds,100.0)

        with patch.object(NODE.rospy.Time,"now",return_value=Stamp(100.31)):
            n._tick(None)
        self.assertFalse(n.inner_handover_pending)

    def test_invalid_center_restarts_handover_confirmation(self):
        n = self.node
        n.inner_handover_pending = True
        n.inner_lane_candidate_since = Stamp(99.9)
        n._inner_center_sanity.return_value = (False,"inner_center_not_ego_lane",1.2)
        n._global_signed_d.return_value = 3.5

        self.tick()

        self.assertTrue(n.inner_handover_pending)
        self.assertIsNone(n.inner_lane_candidate_since)
        self.assertIn("inner_center_not_ego_lane",n._publish.call_args.args[4]["reason"])

    def test_straddling_lane_is_rejected_during_handover(self):
        n = self.node
        n._lane_valid = NODE.HighwayLaneStrategyNode._lane_valid.__get__(n)
        n.lane_info_at = Stamp()
        n.lane_info = {
            "lane_valid": True,
            "confidence": 0.9,
            "straddling_lane": {"detected": True},
        }
        self.assertEqual(n._lane_valid(Stamp()), (False, "lane_straddling"))

    def test_emergency_does_not_commit_rejoin(self):
        self.node.latest_obstacles.obstacles = [obstacle(7.0)]
        self.assertTrue(self.tick()[1])
        self.assertEqual(self.node.state, self.node.INNER_HOLD)

    def test_stale_obstacles_do_not_commit_rejoin(self):
        self.node.obstacles_at = Stamp(90.0)
        self.assertTrue(self.tick()[1])
        self.assertEqual(self.node.state, self.node.INNER_HOLD)

    def test_base_stop_or_stale_status_prevents_rejoin(self):
        for stale in (False, True):
            with self.subTest(stale=stale):
                self.node.base_stop = not stale
                self.node.base_stop_at = Stamp(90.0 if stale else 100.0)
                self.tick()
                self.assertEqual(self.node.state, self.node.INNER_HOLD)
                self.node._generate_rejoin_path.assert_not_called()

    def test_blocked_rejoin_does_not_use_direct_release(self):
        n = self.node
        n._global_signed_d.return_value = 0.2
        n.release_since = Stamp(98.0)
        # Current path is clear, but the proposed return intersects another car.
        n._generate_rejoin_path.return_value = path_at(3.5)
        n.latest_obstacles.obstacles = [obstacle(16.0, 3.5)]
        self.assertTrue(self.tick()[1])
        self.assertEqual(n.state, n.INNER_HOLD)
        self.assertFalse(n.completed_once)

    def test_clear_rejoin_still_commits(self):
        self.assertFalse(self.tick()[1])
        self.assertEqual(self.node.state, self.node.REJOIN)

    def test_inner_hold_continues_after_camera_grace_expires(self):
        n = self.node
        n._lane_valid.return_value = (False, "lane_info_missing_or_stale")
        n.lane_invalid_since = Stamp(90.0)
        n._global_signed_d.return_value = 3.5

        path, stop, speed, _, status, *_ = self.tick()

        self.assertFalse(stop)
        self.assertEqual(speed, n.inner_lane_grace_speed_mps)
        self.assertTrue(status["lane_fallback"])
        self.assertEqual(status["reason"], "lane_fallback_lane_info_missing_or_stale")
        self.assertGreater(len(path.poses), len(n.committed_path.poses))

    def test_inner_hold_lead_gate_uses_commanded_lane_during_camera_handover(self):
        n = self.node
        n._odom_pose.return_value = (0.0, 3.5, 0.0, 2.0)
        n.last_inner_path = path_at(3.5)
        n._centerline_local.return_value = [(float(x), -3.5) for x in range(51)]
        n.latest_obstacles.obstacles = [obstacle(10.0, 0.0)]

        lead, _, _ = n._current_lane_lead()

        self.assertIsNone(lead)

    def test_turning_yaw_cannot_rotate_adjacent_car_into_committed_path(self):
        n = self.node
        n.state = n.LANE_CHANGE
        n.committed_path = path_at(0.0)
        n._odom_pose.return_value = (5.0, 0.0, math.radians(20.0),2.0)
        n.latest_obstacles.obstacles = [obstacle(15.0,3.5)]

        lead, _, _ = n._current_lane_lead()

        self.assertIsNone(lead)

    def test_committed_path_lead_gap_is_independent_of_vehicle_yaw(self):
        n = self.node
        n.state = n.LANE_CHANGE
        n.committed_path = path_at(0.0)
        n.latest_obstacles.obstacles = [obstacle(15.0,0.0)]
        gaps = []
        for yaw in (0.0, math.radians(20.0)):
            n._odom_pose.return_value = (5.0,0.0,yaw,2.0)
            lead, gap, _ = n._current_lane_lead()
            self.assertIsNotNone(lead)
            gaps.append(gap)
        self.assertAlmostEqual(gaps[0],gaps[1],places=6)

    def test_direct_release_preserves_emergency(self):
        n = self.node
        n._lane_valid.return_value = (False, "lane_stale")
        n._global_signed_d.return_value = 0.2
        n.release_since = Stamp(98.0)
        n.latest_obstacles.obstacles = [obstacle(7.0)]
        self.assertTrue(self.tick()[1])
        self.assertFalse(n.completed_once)

    def test_clear_direct_release_survives_camera_dropout(self):
        n = self.node
        n._lane_valid.return_value = (False, "lane_stale")
        n.lane_invalid_since = Stamp(95.0)
        n._global_signed_d.return_value = 0.2
        n.release_since = Stamp(98.0)
        self.assertFalse(self.tick()[1])
        self.assertTrue(n.completed_once)

    def test_rejoin_completion_preserves_emergency(self):
        n = self.node
        n.state = n.REJOIN
        n.rejoin_travel_m = 20.0
        n.release_since = Stamp(98.0)
        n.latest_obstacles.obstacles = [obstacle(7.0)]
        self.assertTrue(self.tick()[1])
        self.assertEqual(n.state, n.REJOIN)
        self.assertFalse(n.completed_once)

    def test_rejoin_completion_preserves_stale_obstacles(self):
        n = self.node
        n.state = n.REJOIN
        n.rejoin_travel_m = 20.0
        n.release_since = Stamp(98.0)
        n.obstacles_at = Stamp(90.0)
        self.assertTrue(self.tick()[1])
        self.assertFalse(n.completed_once)

    def test_clear_rejoin_can_complete(self):
        n = self.node
        n.state = n.REJOIN
        n.rejoin_travel_m = 20.0
        n.release_since = Stamp(98.0)
        self.assertFalse(self.tick()[1])
        self.assertTrue(n.completed_once)
        self.assertEqual(n.state, n.DONE)


if __name__ == "__main__":
    unittest.main()
