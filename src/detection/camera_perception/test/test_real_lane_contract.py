#!/usr/bin/env python3
"""Compatibility checks for the real-lane JSON consumed by highway control."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np


def load_node():
    real_lane = ModuleType("real_lane")
    real_lane.CLASS_NAMES = ["background", "white_solid", "white_dashed"]
    real_lane.CLASS_WHITE_DASHED = 2
    real_lane.ORDER_X_M = 5.0

    camera = ModuleType("morai_camera")
    camera.DEFAULT_IP = "0.0.0.0"
    camera.DEFAULT_PORT = 1101
    camera.CameraStream = object

    imu = ModuleType("morai_imu")
    imu.ImuStream = object

    source = Path(__file__).resolve().parents[1] / "post_processing" / "real_lane_node.py"
    spec = importlib.util.spec_from_file_location("real_lane_node_under_test", source)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {
        "real_lane": real_lane,
        "morai_camera": camera,
        "morai_imu": imu,
        spec.name: module,
    }):
        spec.loader.exec_module(module)
    return module


NODE = load_node()


class Candidate:
    def __init__(self, lane_id, y, confidence, cls=2):
        self.lane_id = lane_id
        self.cls = cls
        self.track_id = lane_id
        self.age = 3
        self.coef = np.array([0.0, 0.0, y])
        self.x_range = (1.0, 6.0)
        self.x = np.array([1.0, 3.0, 6.0])
        self.confidence = confidence
        self.inlier_ratio = 0.9
        self.from_guide = False
        self.coasted = False

    def y_at(self, _x):
        return float(self.coef[-1])


class RealLaneContractTest(unittest.TestCase):
    def test_payload_preserves_highway_control_fields(self):
        result = NS(
            lanes=[Candidate(1, 1.75, 0.82), Candidate(-1, -1.75, 0.67)],
            stopline=None,
            curves=[],
            boundaries=[],
            ground={},
        )
        payload = NODE.build_payload(
            result,
            {"curves": False, "boundaries": False, "lane_pixels": False},
            {},
        )

        self.assertTrue(payload["lane_valid"])
        self.assertEqual(payload["center_source"], "both")
        self.assertEqual(payload["confidence"], 0.67)
        self.assertEqual(payload["lane_width_m"], 3.5)
        self.assertGreaterEqual(len(payload["centerline_points"]), 3)


if __name__ == "__main__":
    unittest.main()
