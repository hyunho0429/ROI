import unittest

from purepursuit_mgeo.stopline_latch import StoplineBrakeLatch


class StoplineBrakeLatchTest(unittest.TestCase):
    def test_keeps_braking_after_camera_loses_line_until_vehicle_stops(self):
        latch = StoplineBrakeLatch(stop_hold_sec=1.0, rearm_clear_sec=0.5)

        self.assertTrue(latch.update(True, 6.0, 0.0))
        self.assertTrue(latch.update(False, 3.0, 0.5))
        self.assertTrue(latch.update(False, 0.1, 1.0))
        self.assertTrue(latch.update(False, 0.1, 1.9))
        self.assertFalse(latch.update(False, 0.1, 2.0))
        self.assertEqual(latch.state, StoplineBrakeLatch.WAIT_CLEAR)

    def test_does_not_retrigger_on_the_same_visible_stopline(self):
        latch = StoplineBrakeLatch(stop_hold_sec=0.0, rearm_clear_sec=0.5)

        self.assertTrue(latch.update(True, 0.0, 0.0))
        self.assertFalse(latch.update(True, 0.0, 0.1))
        self.assertFalse(latch.update(True, 2.0, 0.2))
        self.assertFalse(latch.update(False, 2.0, 0.3))
        self.assertFalse(latch.update(False, 2.0, 0.8))
        self.assertEqual(latch.state, StoplineBrakeLatch.IDLE)
        self.assertTrue(latch.update(True, 2.0, 0.9))


if __name__ == "__main__":
    unittest.main()
