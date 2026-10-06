#!/usr/bin/env python3
"""Check the highway ROS gate's entry source without requiring a ROS runtime."""

import importlib.util
import json
import os
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)


class Bool:
    def __init__(self, data=False):
        self.data = data


def load_gate():
    rospy = types.ModuleType("rospy")
    rospy.logwarn_throttle = lambda *args: None
    rospy.logwarn = lambda *args: None
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = Bool
    std_msgs_msg.String = type("String", (), {})
    path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "scripts", "highway_environment_gate_node.py"
    ))
    spec = importlib.util.spec_from_file_location("highway_gate_under_test", path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {
        "rospy": rospy, "std_msgs": std_msgs, "std_msgs.msg": std_msgs_msg,
    }):
        spec.loader.exec_module(module)
    return module


class FakePublisher:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(message.data)


class HighwayGateTest(unittest.TestCase):
    def test_only_adjacent_composite_activates_without_vehicle(self):
        module = load_gate()
        gate = module.HighwayEnvironmentGateNode.__new__(
            module.HighwayEnvironmentGateNode
        )
        gate.lane_pattern_timeout_s = 0.6
        gate.lane_pattern_tracker = module.ConsecutiveLanePattern(3)
        gate.last_lane_pattern_at = None
        gate.state_latch = module.HighwayEnvironmentLatch(True)
        gate.output_lock = threading.Lock()
        gate.publisher = FakePublisher()
        gate.intersection_active = False
        gate.last_output = None

        def observe(info, index):
            info = dict(info)
            info.update(timestamp=time.time() + index * 0.001,
                        observation_time_source="camera_receive_wall")
            gate._lane_info_callback(types.SimpleNamespace(data=json.dumps(info)))
            gate._timer_callback(None)
            return gate.publisher.values[-1]

        other = {
            "lane_valid": True,
            "output_status": "FRESH",
            "lane_width_m": 3.5,
            "left_lane": {
                "detected": True, "type": "white_dashed", "age": 3,
                "coef": [0.0, 0.0, 1.75], "x_range_m": [5.0, 25.0],
            },
            "left_outer_lane": {
                "detected": True, "type": "white_solid", "age": 3,
                "coef": [0.0, 0.0, 5.25], "x_range_m": [5.0, 25.0],
            },
        }
        other["car_detected"] = True
        for index in range(3):
            self.assertFalse(observe(other, index))

        paired = dict(other)
        paired["left_outer_lane"] = dict(other["left_outer_lane"])
        paired["left_outer_lane"]["coef"] = [0.0, 0.0, 1.90]
        paired["car_detected"] = False
        self.assertFalse(observe(paired, 3))
        self.assertFalse(observe(paired, 4))
        self.assertTrue(observe(paired, 5))

        gate._intersection_callback(Bool(True))
        self.assertFalse(gate.publisher.values[-1])


if __name__ == "__main__":
    unittest.main()
