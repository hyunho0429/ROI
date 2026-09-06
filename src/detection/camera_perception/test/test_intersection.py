#!/usr/bin/env python3

import os
import math
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from camera_perception.intersection import (
    FrontCrossingVehicleLatch,
    IntersectionStateMachine,
    left_to_right_crossing_obstacles,
)


def obstacle(
    track_id=1,
    x=10.0,
    y=3.0,
    vx=0.0,
    vy=-2.0,
    state="MOVING",
    length=4.5,
    width=1.8,
):
    return {
        "id": track_id,
        "center_x_map": x,
        "center_y_map": y,
        "velocity_x_map": vx,
        "velocity_y_map": vy,
        "motion_state": state,
        "length": length,
        "width": width,
    }


class LeftToRightCrossingTest(unittest.TestCase):
    def test_selects_only_front_moving_rightward_objects(self):
        selected = left_to_right_crossing_obstacles(
            [
                obstacle(track_id=1),
                obstacle(track_id=2, vy=2.0),
                obstacle(track_id=3, state="STATIC"),
                obstacle(track_id=4, x=-2.0),
            ],
            ego_x_map=0.0,
            ego_y_map=0.0,
            ego_yaw=0.0,
        )
        self.assertEqual([item["id"] for item in selected], [1])
        self.assertLess(selected[0]["lateral_speed_mps"], 0.0)

    def test_uses_ego_yaw_for_rightward_direction(self):
        selected = left_to_right_crossing_obstacles(
            [obstacle(x=-3.0, y=10.0, vx=2.0, vy=0.0)],
            ego_x_map=0.0,
            ego_y_map=0.0,
            ego_yaw=math.pi / 2.0,
        )
        self.assertEqual(len(selected), 1)
        self.assertAlmostEqual(selected[0]["lateral_m"], 3.0)

    def test_front_crossing_observation_is_latched_immediately(self):
        tracker = FrontCrossingVehicleLatch()
        seen, ids = tracker.update([{"id": 7}])
        self.assertTrue(seen)
        self.assertEqual(ids, (7,))

        seen, ids = tracker.update([])
        self.assertTrue(seen)
        self.assertEqual(ids, (7,))

        seen, ids = tracker.update([], active=False)
        self.assertFalse(seen)
        self.assertEqual(ids, ())


class IntersectionStateMachineTest(unittest.TestCase):
    def test_requires_car_left_yellow_solid_and_right_solid(self):
        for car, left_yellow_solid, right_solid in (
            (False, True, True),
            (True, False, True),
            (True, True, False),
        ):
            with self.subTest(
                car=car,
                left_yellow_solid=left_yellow_solid,
                right_solid=right_solid,
            ):
                state = IntersectionStateMachine()
                decision = state.update(
                    car, left_yellow_solid, right_solid, now=0.0
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

    def test_front_crossing_observation_releases_immediately(self):
        state = IntersectionStateMachine(0.5, 2.0)
        clear = state.update(
            True,
            True,
            True,
            0.0,
            crossing_vehicle_seen_in_front=True,
        )
        self.assertEqual(clear.state, "CLEAR")
        self.assertTrue(clear.driving_allowed)
        self.assertFalse(clear.driving_unavailable)

        # The same still-visible vehicle must not immediately re-block.
        still_clear = state.update(
            True,
            True,
            True,
            0.2,
            crossing_vehicle_seen_in_front=True,
        )
        self.assertEqual(still_clear.state, "CLEAR")

    def test_missing_crossing_observation_reblocks_clear_state(self):
        state = IntersectionStateMachine(0.0, 2.0)
        state.update(True, True, True, 0.0)
        self.assertEqual(
            state.update(
                True,
                True,
                True,
                0.1,
                crossing_vehicle_seen_in_front=True,
            ).state,
            "CLEAR",
        )
        decision = state.update(
            True,
            True,
            True,
            0.2,
            crossing_vehicle_seen_in_front=False,
        )
        self.assertEqual(decision.state, "BLOCKED")
        self.assertTrue(decision.driving_unavailable)


if __name__ == "__main__":
    unittest.main()
