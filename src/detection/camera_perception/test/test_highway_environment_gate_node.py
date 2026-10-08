#!/usr/bin/env python3
"""Check immediate YOLO activation without requiring a ROS runtime."""

import importlib.util
import os
import sys
import threading
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
    rospy.logwarn = lambda *args: None
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = Bool
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
    def test_first_yolo_vehicle_callback_activates_without_lane_observation(self):
        module = load_gate()
        gate = module.HighwayEnvironmentGateNode.__new__(
            module.HighwayEnvironmentGateNode
        )
        gate.latch_once = True
        gate.state_latch = module.HighwayEnvironmentLatch(True)
        gate.output_lock = threading.Lock()
        gate.publisher = FakePublisher()
        gate.car_detected = False
        gate.last_car_msg_at = None
        gate.intersection_active = False
        gate.last_output = None

        gate._car_callback(Bool(True))
        self.assertEqual(gate.publisher.values, [True])

        gate._car_callback(Bool(False))
        self.assertTrue(gate.publisher.values[-1])
        gate._intersection_callback(Bool(True))
        self.assertFalse(gate.publisher.values[-1])
        gate._intersection_callback(Bool(False))
        self.assertTrue(gate.publisher.values[-1])


if __name__ == "__main__":
    unittest.main()
