#!/usr/bin/env python3

import csv
import os
import sys
import tempfile
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from morai_gps_csv_recorder import GpsCsvRecorder
from path_planning.coordinates import MapProjection
from path_planning.morai_udp_gps import GpsMeasurement
from path_planning.stanley_controller import load_gps_path_projection


class MoraiGpsCsvRecorderTest(unittest.TestCase):
    def test_writes_latest_fix_at_fixed_time_intervals(self):
        measurement = GpsMeasurement(
            37.0,
            126.0,
            altitude_m=30.0,
            speed_mps=5.0,
            course_deg=90.0,
        )
        with tempfile.TemporaryDirectory() as directory:
            filename = os.path.join(directory, "path.csv")
            projection = MapProjection("EPSG:32652", 300000.0, 4100000.0, 20.0)
            recorder = GpsCsvRecorder(
                filename,
                sample_period=1.0,
                sample_distance=0.5,
                projection=projection,
            )
            try:
                self.assertTrue(
                    recorder.add_fix(measurement, (1.0, 2.0, 3.0), 100.0, 10.0)
                )
                self.assertFalse(
                    recorder.add_fix(measurement, (2.0, 3.0, 4.0), 100.5, 10.5)
                )
                self.assertTrue(
                    recorder.add_fix(measurement, (3.0, 4.0, 5.0), 101.0, 11.0)
                )
            finally:
                recorder.close()

            with open(filename, newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["global_enu_x_m"], "1.000000000")
            self.assertEqual(rows[1]["global_enu_z_m"], "5.000000000")
            self.assertEqual(rows[0]["velocity_x_mps"], "5.000000")
            self.assertEqual(rows[0]["velocity_y_mps"], "0.000000")
            self.assertEqual(rows[0]["latitude_deg"], "37.0000000000")
            self.assertEqual(rows[0]["longitude_deg"], "126.0000000000")
            self.assertEqual(rows[0]["altitude_m"], "30.000000")
            self.assertEqual(rows[0]["projection_crs"], "EPSG:32652")
            self.assertEqual(rows[0]["east_offset_m"], "300000.000000")
            self.assertEqual(rows[0]["north_offset_m"], "4100000.000000")
            self.assertEqual(rows[0]["up_offset_m"], "20.000000")
            restored = load_gps_path_projection(filename)
            self.assertEqual(restored, projection)

    def test_rejects_stationary_duplicate_even_when_time_advances(self):
        measurement = GpsMeasurement(37.0, 126.0, altitude_m=30.0)
        with tempfile.TemporaryDirectory() as directory:
            recorder = GpsCsvRecorder(
                os.path.join(directory, "path.csv"),
                sample_period=0.0,
                sample_distance=0.5,
            )
            try:
                self.assertTrue(
                    recorder.add_fix(measurement, (1.0, 2.0, 3.0), 100.0, 10.0)
                )
                self.assertFalse(
                    recorder.add_fix(measurement, (1.1, 2.1, 10.0), 101.0, 11.0)
                )
            finally:
                recorder.close()


if __name__ == "__main__":
    unittest.main()
