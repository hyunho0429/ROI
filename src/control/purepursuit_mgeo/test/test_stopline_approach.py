import math
import unittest

from purepursuit_mgeo.stopline_approach import StoplineApproachController


class StoplineApproachControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = StoplineApproachController(
            target_distance_m=2.0,
            comfortable_decel_mps2=1.5,
            stale_timeout_s=0.5,
        )

    def test_ignores_stopline_without_signal_or_intersection_trigger(self):
        self.controller.observe_distance(1.0, 0.0)
        decision = self.controller.update(False, 6.0, 0.1)
        self.assertFalse(decision.armed)
        self.assertFalse(decision.full_stop)
        self.assertEqual(decision.target_speed_mps, 6.0)

    def test_waits_for_stopline_after_trigger(self):
        decision = self.controller.update(True, 6.0, 0.0)
        self.assertTrue(decision.armed)
        self.assertEqual(decision.reason, "WAITING_FOR_STOPLINE")
        self.assertFalse(decision.full_stop)

    def test_reduces_target_speed_using_remaining_distance(self):
        self.controller.observe_distance(5.0, 1.0)
        decision = self.controller.update(True, 6.0, 1.1)
        self.assertAlmostEqual(decision.target_speed_mps, 3.0)
        self.assertFalse(decision.full_stop)

    def test_full_brake_at_target_distance(self):
        self.controller.observe_distance(2.0, 1.0)
        decision = self.controller.update(True, 6.0, 1.1)
        self.assertTrue(decision.full_stop)
        self.assertEqual(decision.target_speed_mps, 0.0)

    def test_stale_seen_stopline_fails_safe(self):
        self.controller.observe_distance(6.0, 1.0)
        decision = self.controller.update(True, 6.0, 1.6)
        self.assertTrue(decision.full_stop)
        self.assertEqual(decision.reason, "STOPLINE_STALE")

    def test_nan_distance_is_ignored(self):
        self.assertFalse(self.controller.observe_distance(math.nan, 0.0))
        decision = self.controller.update(True, 6.0, 0.1)
        self.assertEqual(decision.reason, "WAITING_FOR_STOPLINE")


if __name__ == "__main__":
    unittest.main()
