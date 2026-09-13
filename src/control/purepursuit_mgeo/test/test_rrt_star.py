"""Unit tests for the live in-memory RRT* lane-change planner."""

import math
import unittest

from purepursuit_mgeo.rrt_star import RRTStarPlanner, RectObstacle


class RRTStarTest(unittest.TestCase):
    def planner(self, obstacles=()):
        return RRTStarPlanner(
            (3.0,0.0),
            (35.0,3.5),
            obstacles,
            x_bounds=(3.0,35.0),
            y_bounds=(-0.35,3.85),
            state_is_valid=lambda _x,y: -0.35 <= y <= 3.85,
            max_iterations=50,
            goal_sample_rate=0.15,
            max_edge_heading_rad=math.radians(12.0),
            random_seed=1017,
        )

    def test_clear_corridor_uses_direct_live_path(self):
        planner = self.planner()
        path = planner.plan()
        self.assertEqual(path,[(3.0,0.0),(35.0,3.5)])
        self.assertTrue(planner.path_is_safe(path))

    def test_vehicle_box_produces_forward_rrt_detour(self):
        # The inflated box represents a lead vehicle close enough to matter,
        # while both the adjacent-lane front and rear areas are empty.
        planner = self.planner([RectObstacle(20.0,0.0,5.1,2.24)])
        path = planner.plan()
        self.assertGreater(len(path),2)
        self.assertTrue(all(b[0] > a[0] for a,b in zip(path,path[1:])))
        self.assertTrue(planner.path_is_safe(path))


if __name__ == "__main__":
    unittest.main()
