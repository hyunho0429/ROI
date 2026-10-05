#!/usr/bin/env python3
"""Geometry checks for drawing planner paths on the lane camera window."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lane"))

from path_overlay import PathOverlay, PoseHistory, map_to_ego  # noqa: E402


class FakeCamera:
    """Pin-hole looking down +x: u = 100 - 100*y/x, v = 50 + 100/x."""

    def project(self, pts):
        x = pts[:, 0]
        uv = np.column_stack([100.0 - 100.0 * pts[:, 1] / x, 50.0 + 100.0 / x])
        return uv, x > 0.1


class MapToEgoTest(unittest.TestCase):
    def test_point_ahead_of_rotated_ego(self):
        # Ego at (10, 5) facing +y (north): a map point 4 m north is 4 m ahead.
        ego = map_to_ego([(10.0, 9.0), (8.0, 5.0)], (10.0, 5.0, math.pi / 2))
        np.testing.assert_allclose(ego, [[4.0, 0.0], [0.0, 2.0]], atol=1e-9)


class PoseHistoryTest(unittest.TestCase):
    def test_nearest_within_tolerance(self):
        history = PoseHistory()
        history.add(1.00, (0.0, 0.0, 0.0))
        history.add(1.10, (1.0, 0.0, 0.0))
        self.assertEqual(history.at(1.08), (1.0, 0.0, 0.0))
        self.assertEqual(history.at(None), (1.0, 0.0, 0.0))
        self.assertIsNone(history.at(2.0))


class DrawPathTest(unittest.TestCase):
    def draw(self, pts):
        vis = np.zeros((120, 200, 3), np.uint8)
        shown = PathOverlay._draw_path(
            vis, FakeCamera(), -0.35, np.asarray(pts, float),
            (0.0, 0.0, 0.0), (255, 0, 255), 2, True)
        return vis, shown

    def test_points_behind_camera_are_dropped(self):
        _, shown = self.draw([(-5.0, 0.0), (1.0, 0.0), (10.0, 0.0), (20.0, 0.0)])
        self.assertEqual(shown, 2)

    def test_end_marker_drawn_only_for_visible_last_point(self):
        vis, _ = self.draw([(5.0, 0.0), (10.0, 0.0)])
        u, v = 100, int(50 + 100 / 10.0)
        self.assertTrue((vis[v, u] == (0, 0, 255)).all())
        vis, _ = self.draw([(5.0, 0.0), (10.0, 0.0), (200.0, 0.0)])
        self.assertFalse((vis[v, u] == (0, 0, 255)).all())

    def test_empty_path(self):
        _, shown = self.draw(np.zeros((0, 2)))
        self.assertEqual(shown, 0)


if __name__ == "__main__":
    unittest.main()
