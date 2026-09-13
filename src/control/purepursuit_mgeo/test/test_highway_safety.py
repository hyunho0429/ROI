#!/usr/bin/env python3
"""Safety regressions using the real node with ROS transport stubbed out.

Run: python -m unittest discover -s src/control/purepursuit_mgeo/test -v
Set PYTHONPATH to src/control/purepursuit_mgeo/src first.
No ROS master, simulator, or control publisher is used.
"""
import importlib.util
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
        self.assertTrue(self.tick()[1])

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
