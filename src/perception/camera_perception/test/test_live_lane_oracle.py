import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[4]
SENSOR = ROOT / "Sensor"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SENSOR))

import LiveLaneOracle
from live_lane_oracle_protocol import CATEGORY_WHITE_DASHED, CATEGORY_WHITE_SOLID


class _Stamp:
    def __init__(self, sec, nsec):
        self.secs = sec
        self.nsecs = nsec

    def to_sec(self):
        return self.secs + self.nsecs * 1e-9


class LiveLaneOracleTest(unittest.TestCase):
    def test_left_dashed_requires_valid_white_broken_boundary(self):
        dashed = {
            "conf": 1,
            "broken": True,
            "category": CATEGORY_WHITE_DASHED,
        }
        self.assertTrue(LiveLaneOracle._left_fit_is_dashed(dashed))
        self.assertFalse(LiveLaneOracle._left_fit_is_dashed(None))
        self.assertFalse(
            LiveLaneOracle._left_fit_is_dashed(
                {**dashed, "category": CATEGORY_WHITE_SOLID}
            )
        )
        self.assertFalse(
            LiveLaneOracle._left_fit_is_dashed({**dashed, "broken": False})
        )

    def test_ros_status_is_converted_for_recorddrive_interpolation(self):
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=_Stamp(12, 500_000_000)),
            position=SimpleNamespace(x=10.0, y=20.0, z=30.0),
            velocity=SimpleNamespace(x=3.0, y=4.0, z=0.0),
            heading=270.0,
            wheel_angle=1.5,
            accel=0.2,
            brake=0.0,
        )
        snapshot, timestamp = LiveLaneOracle._ros_status_snapshot(message)
        self.assertAlmostEqual(timestamp, 12.5)
        self.assertAlmostEqual(snapshot["signed_vel"], 5.0)
        self.assertAlmostEqual(snapshot["yaw"], -90.0)
        self.assertEqual(snapshot["status_sec"], 12)
        self.assertEqual(snapshot["status_nsec"], 500_000_000)


if __name__ == "__main__":
    unittest.main()
