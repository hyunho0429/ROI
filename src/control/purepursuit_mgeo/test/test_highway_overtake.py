#!/usr/bin/env python3
"""Checks for purepursuit_mgeo.highway_overtake (no ROS needed).

Run from src/control/purepursuit_mgeo:
    PYTHONPATH=src python -m unittest discover -s test -p "test_highway_overtake.py" -v
"""
import math
from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))
sys.path.insert(0, str(PACKAGE / "tools"))

from purepursuit_mgeo.highway_overtake import (  # noqa: E402
    LaneReference, LaneTrack, OvertakeConfig, change_geometry, evaluate_change,
    make_change_plan, parse_lane_info,
)


def lane(coef, kind="white_dashed", age=5, **extra):
    meta = {"detected": True, "type": kind, "dashed": kind == "white_dashed",
            "coef": list(coef), "x_range_m": [4.0, 40.0], "age": age,
            "from_guide": False, "coasted": False}
    meta.update(extra)
    return meta


def lane_info(stamp, left=(0.0, 0.0, 1.75), right=(0.0, 0.0, -1.75), **extra):
    info = {"timestamp": stamp, "output_status": "FRESH", "lane_valid": True,
            "left_lane": lane(left), "right_lane": lane(right, "white_solid"),
            "straddling_lane": None}
    info.update(extra)
    return info


class ParseLaneInfoTest(unittest.TestCase):
    cfg = OvertakeConfig()

    def test_real_lane_payload(self):
        obs, reason = parse_lane_info(lane_info(100.0), 100.05, self.cfg)
        self.assertEqual(reason, "ok")
        self.assertTrue(obs.left_dashed)
        self.assertAlmostEqual(obs.width, 3.5)
        self.assertFalse(obs.stamp_is_capture)

    def test_v2_capture_stamp(self):
        info = lane_info(99.9, observation_time_source="camera_receive_wall")
        obs, _ = parse_lane_info(info, 100.0, self.cfg)
        self.assertTrue(obs.stamp_is_capture)
        self.assertAlmostEqual(obs.stamp, 99.9)

    def test_rejections(self):
        cases = {
            "straddling": lane_info(100.0, straddling_lane=lane((0, 0, 0.1))),
            "status_held": lane_info(100.0, output_status="HELD"),
            "left_predicted": lane_info(100.0, left_lane=lane((0, 0, 1.75), coasted=True)),
            "left_young": lane_info(100.0, left_lane=lane((0, 0, 1.75), age=1)),
            "width_2.40": lane_info(100.0, left=(0, 0, 1.2), right=(0, 0, -1.2)),
            "stale": lane_info(99.0),
        }
        for expected, info in cases.items():
            obs, reason = parse_lane_info(info, 100.0, self.cfg)
            self.assertIsNone(obs, expected)
            self.assertEqual(reason, expected)


class LaneReferenceTest(unittest.TestCase):
    cfg = OvertakeConfig()

    def obs(self, right):
        info = lane_info(100.0, left=(0.0, 0.0, 1.75), right=right)
        return parse_lane_info(info, 100.0, self.cfg)[0]

    def test_round_trip_in_rotated_map(self):
        ref = LaneReference.from_observation(self.obs((0, 0, -1.75)), (10.0, 5.0, 1.0), self.cfg)
        x, y = ref.point(25.0, 3.5)
        s, d = ref.to_frenet(x, y)
        self.assertAlmostEqual(s, 25.0, places=6)
        self.assertAlmostEqual(d, 3.5, places=6)

    def test_converging_right_edge_does_not_tilt_the_frame(self):
        # Entry-lane taper: right edge converges at 3 deg, the divider is straight.
        ref = LaneReference.from_observation(
            self.obs((0.0, math.tan(math.radians(3.0)), -1.75)), (0.0, 0.0, 0.0), self.cfg)
        self.assertAlmostEqual(ref.psi, 0.0, places=9)


class ChangeGeometryTest(unittest.TestCase):
    def test_entry_angle_and_lateral_acceleration(self):
        cfg = OvertakeConfig()
        for v in (5.0, 10.0, 15.0, 22.0, 25.0):
            length, ramp = change_geometry(3.5, v, cfg)
            angle = math.degrees(math.atan(3.5/((1.0 - ramp)*length)))
            a_lat = v*v*1.5*3.5/(ramp*(1.0 - ramp)*length*length)
            self.assertLessEqual(angle, cfg.max_entry_angle_deg + 1e-6, v)
            self.assertLessEqual(a_lat, cfg.max_lateral_accel_mps2 + 1e-6, v)


class GapAcceptanceTest(unittest.TestCase):
    """Aggressive acceptance: merge as soon as the left car is passed."""
    cfg = OvertakeConfig()
    ref = LaneReference(0.0, 0.0, 0.0, 0.0, 200.0, 200.0)

    def decide(self, *tracks, v=20.0):
        plan = make_change_plan(self.ref, 0.0, 0.0, 3.5, v, self.cfg)
        return evaluate_change(plan, 0.0, v, list(tracks), self.cfg)

    @staticmethod
    def car(oid, s, lane, v):
        return LaneTrack(oid, s, 3.5*lane, v, 4.6, 1.9, lane)

    def test_empty_target_lane(self):
        self.assertTrue(self.decide().ok)

    def test_car_alongside_blocks(self):
        decision = self.decide(self.car(1, 1.5, 1, 20.0))
        self.assertFalse(decision.ok)
        self.assertEqual(decision.blocker, 1)

    def test_just_passed_slower_car_is_accepted(self):
        # Our rear bumper (s=-0.82) is ~1.5 m ahead of its front bumper.
        self.assertTrue(self.decide(self.car(1, -4.6, 1, 15.0)).ok)

    def test_faster_car_close_behind_blocks(self):
        decision = self.decide(self.car(1, -12.0, 1, 26.0))
        self.assertFalse(decision.ok)
        self.assertEqual(decision.reason, "target_rear")

    def test_own_lane_follower_is_ignored(self):
        self.assertTrue(self.decide(self.car(1, -8.0, 0, 25.0)).ok)


class SimulatorSmokeTest(unittest.TestCase):
    """Closed loop on the sample-scenario traffic: no collision, no solid line."""

    def test_scenarios(self):
        import highway_overtake_sim as sim
        for name, seed in (("morai", 0), ("morai", 1), ("empty", 0)):
            result = sim.run(sim.ALL_SCENARIOS[name], seed, record=False)
            self.assertEqual(sim.verdict(result), [], (name, seed))
            self.assertEqual(result["lane_changes"], 3, (name, seed))


if __name__ == "__main__":
    unittest.main()
