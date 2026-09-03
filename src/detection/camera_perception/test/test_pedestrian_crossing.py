#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from camera_perception.pedestrian_crossing import PedestrianStopStateMachine


class PedestrianStopStateMachineTest(unittest.TestCase):
    def test_person_stops_immediately_and_camera_clear_resumes(self):
        state = PedestrianStopStateMachine(
            stop_confirmation_s=0.0,
            clear_confirmation_s=0.5,
        )

        decision = state.update(0.0, True, True, False)
        self.assertTrue(decision.stop_required)
        self.assertFalse(decision.resume_allowed)
        self.assertEqual(decision.transition, "STOP")

        decision = state.update(0.1, True, False, True)
        self.assertTrue(decision.stop_required)
        decision = state.update(0.61, True, False, True)
        self.assertFalse(decision.stop_required)
        self.assertTrue(decision.resume_allowed)
        self.assertEqual(decision.transition, "RESUME")

    def test_one_false_frame_does_not_release_stop(self):
        state = PedestrianStopStateMachine(0.0, 0.5)
        state.update(0.0, True, True, False)
        state.update(0.1, True, False, True)
        decision = state.update(0.2, True, True, False)
        self.assertTrue(decision.stop_required)

        decision = state.update(0.7, True, False, True)
        self.assertTrue(decision.stop_required)
        decision = state.update(1.21, True, False, True)
        self.assertFalse(decision.stop_required)

    def test_stale_camera_cannot_release_latched_stop(self):
        state = PedestrianStopStateMachine(0.0, 0.5)
        state.update(0.0, True, True, False)

        decision = state.update(1.0, False, False, True)
        self.assertTrue(decision.stop_required)
        decision = state.update(2.0, True, False, True)
        self.assertTrue(decision.stop_required)
        decision = state.update(2.51, True, False, True)
        self.assertFalse(decision.stop_required)


if __name__ == "__main__":
    unittest.main()
