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
    def __init__(self, lane_id, y, confidence, cls=2, x_range=(1.0, 6.0)):
        self.lane_id = lane_id
        self.cls = cls
        self.track_id = lane_id
        self.age = 3
        self.coef = np.array([0.0, 0.0, y])
        self.x_range = x_range
        self.x = np.array([x_range[0], 0.5*(x_range[0]+x_range[1]), x_range[1]])
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

    def test_centerline_uses_common_boundary_x_coordinates(self):
        left = Candidate(1, 0.0, 0.9, x_range=(1.0, 6.0))
        right = Candidate(-1, -3.5, 0.9, x_range=(2.0, 6.0))
        left.coef = np.array([0.0, 1.0, 0.0])
        right.coef = np.array([0.0, 1.0, -3.5])
        payload = NODE.build_payload(
            NS(lanes=[left, right], stopline=None, curves=[], boundaries=[], ground={}),
            {"curves": False, "boundaries": False, "lane_pixels": False},
            {},
        )
        self.assertEqual(payload["centerline_points"][0], [2.0, 0.25])

    def test_straddling_lane_suppresses_control_centerline(self):
        result = NS(
            lanes=[
                Candidate(1, 3.5, 0.9),
                Candidate(0, 0.0, 0.9),
                Candidate(-1, -3.5, 0.9),
            ],
            stopline=None, curves=[], boundaries=[], ground={},
        )
        payload = NODE.build_payload(
            result,
            {"curves": False, "boundaries": False, "lane_pixels": False},
            {},
        )
        self.assertIsNone(payload["centerline_points"])
        self.assertIn("STRADDLING", payload["reasons"])

    def test_payload_exposes_second_left_boundary_for_solid_edge(self):
        result = NS(
            lanes=[
                Candidate(2, 5.25, 0.8, cls=1),
                Candidate(1, 1.75, 0.9, cls=1),
                Candidate(-1, -1.75, 0.9),
            ],
            stopline=None, curves=[], boundaries=[], ground={},
        )

        payload = NODE.build_payload(
            result,
            {"curves": False, "boundaries": False, "lane_pixels": False},
            {},
        )

        self.assertEqual(payload["left_lane"]["lane_id"], 1)
        self.assertEqual(payload["left_outer_lane"]["lane_id"], 2)
        self.assertEqual(payload["left_outer_lane"]["type"], "white_solid")
        self.assertEqual(len(payload["left_lanes"]), 2)


if __name__ == "__main__":
    unittest.main()
