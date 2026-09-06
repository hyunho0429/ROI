import unittest

from camera_perception.traffic_signal import (
    TrafficSignalStopLatch,
    traffic_signal_requires_stop,
)


class TrafficSignalTest(unittest.TestCase):
    def test_stops_for_red_only_and_all_yellow_variants(self):
        for class_name in (
            "Red",
            "Red_Yellow",
            "Yellow",
            "yellow_left",
            "Amber",
        ):
            self.assertTrue(
                traffic_signal_requires_stop([class_name]),
                msg=class_name,
            )

    def test_red_with_turn_signal_does_not_stop(self):
        self.assertFalse(traffic_signal_requires_stop(["Red_Left"]))
        self.assertFalse(traffic_signal_requires_stop(["Red_Right"]))
        self.assertFalse(traffic_signal_requires_stop(["Red", "Left"]))
        self.assertFalse(traffic_signal_requires_stop(["RED", "Green_Left"]))

    def test_does_not_stop_for_green_left_or_no_detection(self):
        self.assertFalse(traffic_signal_requires_stop([]))
        self.assertFalse(traffic_signal_requires_stop(["Green", "Green_Left", "Left"]))

    def test_requires_continuous_clear_frames_before_release(self):
        latch = TrafficSignalStopLatch(clear_confirmation_s=0.5)
        self.assertTrue(latch.update(["Red"], 0.0))
        self.assertTrue(latch.update([], 0.1))
        self.assertTrue(latch.update(["Green"], 0.5))
        self.assertFalse(latch.update(["Green"], 0.6))


if __name__ == "__main__":
    unittest.main()
