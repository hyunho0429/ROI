#!/usr/bin/env python3
"""Wiring of scripts/highway_overtake_node.py with ROS stubbed out.

Run from src/control/purepursuit_mgeo:
    PYTHONPATH=src python -m unittest discover -s test -p "test_highway_overtake_node.py" -v
"""
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE / "src"))

from purepursuit_mgeo.highway_overtake import Route  # noqa: E402
from purepursuit_mgeo.path import load_mgeo_path  # noqa: E402

ROUTE_FILE = REPO / "data" / "routes" / "2026_molit_comp_global_path.txt"


class Msg:
    """Attribute bag standing in for any ROS message."""
    def __init__(self, **kwargs):
        self.header = NS(stamp=None, frame_id="")
        self.__dict__.update(kwargs)


class PoseStamped(Msg):
    def __init__(self):
        super().__init__(pose=NS(position=NS(x=0.0, y=0.0, z=0.0),
                                 orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))


class RosPath(Msg):
    def __init__(self):
        super().__init__(poses=[])


class Marker(Msg):
    CUBE, TEXT_VIEW_FACING, ADD = 1, 9, 0

    def __init__(self):
        super().__init__(ns="", id=0, type=0, action=0, text="",
                         pose=NS(position=NS(x=0.0, y=0.0, z=0.0),
                                 orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
                         scale=NS(x=0.0, y=0.0, z=0.0), color=NS(r=0.0, g=0.0, b=0.0, a=0.0))


class MarkerArray(Msg):
    def __init__(self):
        super().__init__(markers=[])


def load_node_module(params):
    rospy = Mock()
    rospy.get_param.side_effect = lambda name, default=None: params.get(name, default)
    rospy.has_param.side_effect = lambda name: name in params
    publishers = {}

    def publisher(name, *_args, **_kwargs):
        publishers[name] = Mock()
        return publishers[name]

    rospy.Publisher.side_effect = publisher
    modules = {"rospy": rospy}
    for name, values in {
        "geometry_msgs.msg": {"PoseStamped": PoseStamped},
        "nav_msgs.msg": {"Odometry": Msg, "Path": RosPath},
        "std_msgs.msg": {"Bool": Msg, "Float64": Msg, "String": Msg},
        "visualization_msgs.msg": {"Marker": Marker, "MarkerArray": MarkerArray},
        "lidar_perception.msg": {"LidarObstacleArray": Msg},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        modules[name] = module
    source = PACKAGE / "scripts" / "highway_overtake_node.py"
    spec = importlib.util.spec_from_file_location("highway_overtake_node_under_test", source)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {**modules, spec.name: module}):
        spec.loader.exec_module(module)
    return module, publishers


def odom(x, y, yaw, v):
    return NS(pose=NS(pose=NS(position=NS(x=x, y=y, z=0.0),
                              orientation=NS(x=0.0, y=0.0, z=math.sin(0.5*yaw), w=math.cos(0.5*yaw)))),
              twist=NS(twist=NS(linear=NS(x=v, y=0.0, z=0.0))))


def lane_json(stamp):
    import json

    def lane(c, kind):
        return {"detected": True, "type": kind, "dashed": kind == "white_dashed",
                "coef": [0.0, 0.0, c], "x_range_m": [4.0, 40.0], "age": 5,
                "from_guide": False, "coasted": False}
    return NS(data=json.dumps({"timestamp": stamp, "output_status": "FRESH", "lane_valid": True,
                               "left_lane": lane(1.75, "white_dashed"),
                               "right_lane": lane(-1.75, "white_solid"),
                               "straddling_lane": None}))


class HighwayOvertakeNodeTest(unittest.TestCase):
    def setUp(self):
        config = yaml.safe_load((PACKAGE / "config" / "highway_overtake.yaml").read_text(encoding="utf-8"))
        params = {"~" + key: value for key, value in config.items()}
        params.update({"~path_file": str(ROUTE_FILE), "~cruise_speed_mps": 15.0,
                       "~max_speed_mps": 22.0})
        self.module, self.pubs = load_node_module(params)
        self.node = self.module.HighwayOvertakeNode()
        points = [(p.x, p.y) for p in load_mgeo_path(str(ROUTE_FILE))]
        self.route = Route(points)
        self.points = points
        self.zone = (self.route.s_of_xy(*config["zone_start_xy"]),
                     self.route.s_of_xy(*config["zone_end_xy"]))

    def pose_at(self, s):
        i = next(k for k, sk in enumerate(self.route.s) if sk >= s)
        (ax, ay), (bx, by) = self.points[i-1], self.points[i]
        return bx, by, math.atan2(by-ay, bx-ax), i

    def feed(self, s, v=15.0):
        import time
        x, y, yaw, i = self.pose_at(s)
        base = RosPath()
        for px, py in self.points[max(0, i-6):i+160]:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = px, py
            base.poses.append(pose)
        self.node._odom_cb(odom(x, y, yaw, v))
        self.node._base_path_cb(base)
        self.node._base_stop_cb(NS(data=False))
        self.node._obstacles_cb(NS(obstacles=[]))
        self.node._lane_cb(lane_json(time.time()))
        self.node._tick(None)

    def last(self, topic):
        return self.pubs[topic].publish.call_args[0][0]

    def test_zone_resolution(self):
        self.assertAlmostEqual(self.zone[0], 1160.0, delta=2.0)
        self.assertAlmostEqual(self.zone[1], 1590.0, delta=2.0)

    def test_pass_through_before_the_zone(self):
        self.feed(self.zone[0] - 60.0)
        self.assertFalse(self.last("~zone_active").data)
        self.assertFalse(self.last("~active").data)
        self.assertFalse(self.last("~stop_required").data)
        self.assertEqual(len(self.last("~active_path").poses), 166)

    def test_lane_change_inside_the_zone(self):
        for k in range(4):
            self.feed(self.zone[0] + 5.0 + 0.75*k)
        self.assertTrue(self.last("~zone_active").data)
        self.assertTrue(self.last("~active").data)
        self.assertTrue(self.last("~fast_change_active").data)
        self.assertIn('"state":"CHANGE"', self.last("~state").data)
        path = self.last("~active_path")
        self.assertEqual(path.header.frame_id, "map")
        x, y, yaw, _ = self.pose_at(self.zone[0] + 7.25)
        far = path.poses[-1].pose.position
        left = -math.sin(yaw)*(far.x - x) + math.cos(yaw)*(far.y - y)
        self.assertGreater(left, 3.0)     # the path ends one lane to the left


if __name__ == "__main__":
    unittest.main()
