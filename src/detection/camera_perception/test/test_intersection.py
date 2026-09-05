#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from camera_perception.intersection import (
    IntersectionStateMachine,
    perpendicular_dynamic_obstacles,
)


def obstacle(vx, vy, state="MOVING", x=10.0, y=0.0):
    return {
        "center_x_map": x,
        "center_y_map": y,
        "velocity_x_map": vx,
        "velocity_y_map": vy,
        "motion_state": state,
    }


class PerpendicularDynamicObstacleTest(unittest.TestCase):
    def test_selects_motion_perpendicular_to_ego(self):
        selected = perpendicular_dynamic_obstacles(
            [obstacle(0.0, 3.0), obstacle(3.0, 0.0)], 0.0, 0.0, 0.0
        )
        self.assertEqual(len(selected), 1)
        self.assertEqual(selected[0]["velocity_y_map"], 3.0)

    def test_uses_map_frame_ego_yaw(self):
        selected = perpendicular_dynamic_obstacles(
            [obstacle(3.0, 0.0)], 0.0, 0.0, math.pi / 2.0
        )
        self.assertEqual(len(selected), 1)

    def test_rejects_static_slow_and_far_objects(self):
        selected = perpendicular_dynamic_obstacles(
            [
                obstacle(0.0, 3.0, state="STATIC"),
                obstacle(0.0, 0.2),
                obstacle(0.0, 3.0, x=100.0),
            ],
            0.0,
            0.0,
            0.0,
        )
        self.assertEqual(selected, [])


class IntersectionStateMachineTest(unittest.TestCase):
    def test_blocks_on_camera_and_perpendicular_lidar_then_allows_when_clear(self):
        state = IntersectionStateMachine(0.5, 2.0)
        self.assertEqual(state.update(False, True, 0.0).state, "IDLE")
        blocked = state.update(True, True, 0.1)
        self.assertTrue(blocked.detected)
        self.assertTrue(blocked.driving_unavailable)
        self.assertFalse(blocked.driving_allowed)
        self.assertEqual(state.update(False, False, 0.4).state, "BLOCKED")
        clear = state.update(False, False, 1.0)
        self.assertTrue(clear.detected)
        self.assertTrue(clear.driving_allowed)
        self.assertFalse(clear.driving_unavailable)

    def test_stale_camera_cannot_release_blocked_state(self):
        state = IntersectionStateMachine(0.5, 2.0)
        state.update(True, True, 0.0)
        self.assertEqual(
            state.update(False, False, 10.0, camera_fresh=False).state,
            "BLOCKED",
        )

    def test_camera_vehicle_reblocks_clear_state_without_new_lidar_gate(self):
        state = IntersectionStateMachine(0.0, 2.0)
        state.update(True, True, 0.0)
        self.assertEqual(state.update(False, False, 0.1).state, "CLEAR")
        decision = state.update(True, False, 0.2)
        self.assertEqual(decision.state, "BLOCKED")
        self.assertTrue(decision.driving_unavailable)


if __name__ == "__main__":
    unittest.main()
