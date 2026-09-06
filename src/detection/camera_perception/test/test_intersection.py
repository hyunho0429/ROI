#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from camera_perception.intersection import IntersectionStateMachine


class IntersectionStateMachineTest(unittest.TestCase):
    def test_requires_car_and_both_solid_lanes(self):
        for car, left_solid, right_solid in (
            (False, True, True),
            (True, False, True),
            (True, True, False),
        ):
            with self.subTest(
                car=car, left_solid=left_solid, right_solid=right_solid
            ):
                state = IntersectionStateMachine()
                decision = state.update(
                    car, left_solid, right_solid, now=0.0
                )
                self.assertEqual(decision.state, "IDLE")
                self.assertFalse(decision.driving_unavailable)

        state = IntersectionStateMachine()
        decision = state.update(True, True, True, now=0.0)
        self.assertEqual(decision.state, "BLOCKED")
        self.assertTrue(decision.driving_unavailable)

    def test_stale_input_cannot_recognize_intersection(self):
        state = IntersectionStateMachine()
        self.assertEqual(
            state.update(
                True, True, True, now=0.0, camera_fresh=False
            ).state,
            "IDLE",
        )
        self.assertEqual(
            state.update(
                True, True, True, now=0.1, lane_fresh=False
            ).state,
            "IDLE",
        )

    def test_allows_after_camera_is_clear(self):
        state = IntersectionStateMachine(0.5, 2.0)
        state.update(True, True, True, 0.0)
        self.assertEqual(state.update(False, True, True, 0.4).state, "BLOCKED")
        clear = state.update(False, True, True, 1.0)
        self.assertTrue(clear.detected)
        self.assertTrue(clear.driving_allowed)
        self.assertFalse(clear.driving_unavailable)

    def test_stale_camera_cannot_release_blocked_state(self):
        state = IntersectionStateMachine(0.5, 2.0)
        state.update(True, True, True, 0.0)
        self.assertEqual(
            state.update(
                False, False, False, 10.0, camera_fresh=False
            ).state,
            "BLOCKED",
        )

    def test_reblock_requires_car_and_both_solid_lanes(self):
        state = IntersectionStateMachine(0.0, 2.0)
        state.update(True, True, True, 0.0)
        self.assertEqual(state.update(False, True, True, 0.1).state, "CLEAR")
        self.assertEqual(state.update(True, True, False, 0.2).state, "CLEAR")
        decision = state.update(True, True, True, 0.3)
        self.assertEqual(decision.state, "BLOCKED")
        self.assertTrue(decision.driving_unavailable)


if __name__ == "__main__":
    unittest.main()
