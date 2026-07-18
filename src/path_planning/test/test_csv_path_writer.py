#!/usr/bin/env python3

import csv
import os
import sys
import tempfile
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from path_planning.csv_path_writer import CsvPathWriter, resample_segment, spatial_distance


class CsvPathWriterTest(unittest.TestCase):
    def test_spatial_distance_includes_z(self):
        self.assertAlmostEqual(spatial_distance((0.0, 0.0, 0.0), (2.0, 3.0, 6.0)), 7.0)

    def test_resample_segment_uses_uniform_3d_spacing_and_carry(self):
        fractions, distance_to_next = resample_segment(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 1.2),
            distance_to_next=0.5,
            sample_distance=0.5,
        )
        self.assertEqual(len(fractions), 2)
        self.assertAlmostEqual(fractions[0], 0.5 / 1.2)
        self.assertAlmostEqual(fractions[1], 1.0 / 1.2)
        self.assertAlmostEqual(distance_to_next, 0.3)

        fractions, distance_to_next = resample_segment(
            (0.0, 0.0, 1.2),
            (0.0, 0.0, 1.5),
            distance_to_next=distance_to_next,
            sample_distance=0.5,
        )
        self.assertEqual(fractions, [1.0])
        self.assertAlmostEqual(distance_to_next, 0.5)

    def test_writer_creates_readable_csv_and_header(self):
        with tempfile.TemporaryDirectory() as directory:
            output_file = os.path.join(directory, "path.csv")
            with CsvPathWriter(output_file) as writer:
                writer.write(
                    {
                        "sequence": 1,
                        "global_enu_x_m": "10.000000000",
                        "global_enu_y_m": "20.000000000",
                        "global_enu_z_m": "1.000000000",
                    }
                )

            with open(output_file, newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))

            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["sequence"], "1")
            self.assertEqual(rows[0]["global_enu_x_m"], "10.000000000")

    def test_append_rejects_an_old_or_incompatible_header(self):
        with tempfile.TemporaryDirectory() as directory:
            output_file = os.path.join(directory, "old_path.csv")
            with open(output_file, "w", newline="", encoding="utf-8") as stream:
                stream.write("x,y,z\n")

            with self.assertRaises(ValueError):
                CsvPathWriter(output_file, append=True)


if __name__ == "__main__":
    unittest.main()
