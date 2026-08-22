#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from camera_perception.pedestrian_crossing import (
    PedestrianStopStateMachine,
    pedestrian_lidar_candidates,
)


def _obstacle(x_m, y_m, length_m=0.5, width_m=0.5):
    return {
        "id": 1,
        "center_x_map": float(x_m),
        "center_y_map": float(y_m),
        "length": float(length_m),
        "width": float(width_m),
        "motion_state": "MOVING",
    }


class PedestrianLidarCandidateTest(unittest.TestCase):
    def test_selects_front_and_side_human_sized_cluster_in_map_frame(self):
        candidates = pedestrian_lidar_candidates(
            obstacles=[_obstacle(10.0, 21.0)],
            ego_x=10.0,
            ego_y=20.0,
            ego_yaw=0.5 * math.pi,
            detection_distance_m=1.5,
        )

        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0]["forward_m"], 1.0)
        self.assertAlmostEqual(candidates[0]["left_m"], 0.0, places=6)
        self.assertLess(candidates[0]["distance_m"], 1.0)

    def test_rejects_rear_far_and_vehicle_sized_clusters(self):
        candidates = pedestrian_lidar_candidates(
            obstacles=[
                _obstacle(-1.0, 0.0),
                _obstacle(4.0, 0.0),
                _obstacle(1.0, 0.0, length_m=4.5, width_m=1.9),
            ],
            ego_x=0.0,
            ego_y=0.0,
            ego_yaw=0.0,
            detection_distance_m=1.5,
            rear_allowance_m=0.5,
        )

        self.assertEqual(candidates, [])


class PedestrianStopStateMachineTest(unittest.TestCase):
    def test_stops_after_confirmation_and_resumes_only_when_both_clear(self):
        state = PedestrianStopStateMachine(
            stop_confirmation_s=0.2,
            clear_confirmation_s=1.0,
        )

        decision = state.update(0.0, True, True, False)
        self.assertFalse(decision.stop_required)
        decision = state.update(0.21, True, True, False)
        self.assertTrue(decision.stop_required)
        self.assertEqual(decision.transition, "STOP")

        # Camera clear alone is insufficient while a LiDAR cluster remains.
        decision = state.update(1.0, True, False, False)
        self.assertTrue(decision.stop_required)

        # Stale inputs cannot release a latched stop.
        decision = state.update(2.0, False, False, True)
        self.assertTrue(decision.stop_required)

        decision = state.update(3.0, True, False, True)
        self.assertTrue(decision.stop_required)
        decision = state.update(4.01, True, False, True)
        self.assertFalse(decision.stop_required)
        self.assertTrue(decision.resume_allowed)
        self.assertEqual(decision.transition, "RESUME")


if __name__ == "__main__":
    unittest.main()
