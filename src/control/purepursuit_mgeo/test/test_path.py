#!/usr/bin/env python3
import os
import tempfile
import unittest

from purepursuit_mgeo.path import MgeoPurePursuit, load_mgeo_path


class PathTest(unittest.TestCase):
    def test_load_and_compute(self):
        with tempfile.NamedTemporaryFile(mode="w", delete=False) as stream:
            stream.write("0 0 0\n10 0 0\n20 0 0\n")
            path_file = stream.name
        try:
            points = load_mgeo_path(path_file)
            controller = MgeoPurePursuit(points, 2.7, 4.0, 0.0, 1.0)
            steering, stop, target, _, _ = controller.compute(0.0, 0.0, 0.0, 1.0)
            self.assertFalse(stop)
            self.assertAlmostEqual(steering, 0.0, places=6)
            self.assertGreater(target.x, 0.0)
        finally:
            os.unlink(path_file)


if __name__ == "__main__":
    unittest.main()
