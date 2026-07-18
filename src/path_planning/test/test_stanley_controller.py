#!/usr/bin/env python3

import math
import os
import sys
import tempfile
import unittest
from unittest.mock import patch


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from path_planning.localization import PlanarGpsImuEkf
from path_planning.longitudinal_controller import PedalSpeedController
from path_planning.coordinates import GpsToRecordedLocalEnu, MapProjection
from path_planning.stanley_controller import (
    PathPoint,
    StanleyController,
    SteeringCommandFilter,
    load_path_csv,
    load_recorded_path_origin,
)
from path_planning.stanley_udp_runtime import argument_parser


class StanleyControllerTest(unittest.TestCase):
    def test_loads_origin_anchored_sensor_csv_in_recorded_local_frame(self):
        header = (
            "x,y,z,target_speed,lat,lon,alt,origin_lat,origin_lon,origin_alt,"
            "imu_qx,imu_qy,imu_qz,imu_qw\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            filename = os.path.join(directory, "provided_path.csv")
            with open(filename, "w", encoding="utf-8") as stream:
                stream.write(header)
                stream.write("0,0,0,1,,,,,,,-0.86,0,0,-0.51\n")
                stream.write(
                    "0.380822,0.726466,0,1,37.24098833,126.77436,0,"
                    "37.24098167,126.774355,0,0.86,0,0,0.51\n"
                )
            points = load_path_csv(filename)
            origin = load_recorded_path_origin(filename)

        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0].x_m, 0.0)
        self.assertAlmostEqual(points[0].y_m, 0.0)
        self.assertAlmostEqual(points[1].x_m, 0.380822)
        self.assertAlmostEqual(points[1].y_m, 0.726466)
        self.assertEqual(points[0].target_speed_mps, 1.0)
        self.assertEqual(points[1].target_speed_mps, 1.0)
        self.assertAlmostEqual(origin.latitude_deg, 37.24098167)
        self.assertAlmostEqual(origin.longitude_deg, 126.774355)
        self.assertAlmostEqual(origin.altitude_m, 0.0)

        live_x, live_y, live_z = GpsToRecordedLocalEnu(origin).convert(
            37.24098833, 126.77436, 0.0
        )
        self.assertAlmostEqual(live_x, points[1].x_m, delta=0.1)
        self.assertAlmostEqual(live_y, points[1].y_m, delta=0.1)
        self.assertAlmostEqual(live_z, points[1].z_m)

    def test_interpolates_path_target_speed(self):
        controller = StanleyController(
            [
                PathPoint(0.0, 0.0, target_speed_mps=1.0),
                PathPoint(10.0, 0.0, target_speed_mps=3.0),
            ],
            control_point_offset_m=0.0,
        )
        result = controller.compute(5.0, 0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(result.target_speed_mps, 2.0)

    def test_loads_morai_gps_and_imu_combined_csv_as_enu_path(self):
        origin_x, origin_y = 302595.0, 4124145.0
        latitude_1, longitude_1 = 37.1, 126.1
        latitude_2, longitude_2 = 37.2, 126.2
        header = (
            "latitude,longitude,altitude,eastOffset,northOffset,"
            "imu_sec,imu_nsec,orientation_x,orientation_y,orientation_z,orientation_w,"
            "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
            "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            filename = os.path.join(directory, "sensor_path.csv")
            with open(filename, "w", encoding="utf-8") as stream:
                stream.write(header)
                stream.write(
                    "{},{},28.0,{},{},1,0,0,0,0,1,0,0,0,0,0,9.81\n".format(
                        latitude_1, longitude_1, origin_x, origin_y
                    )
                )
                stream.write(
                    "{},{},29.0,{},{},2,0,0,0,0,1,0,0,0,0,0,9.81\n".format(
                        latitude_2, longitude_2, origin_x, origin_y
                    )
                )
            with patch("path_planning.stanley_controller.GpsToMapEnu") as converter_type:
                converter_type.return_value.convert.side_effect = (
                    lambda latitude, longitude, altitude: (
                        longitude,
                        latitude,
                        altitude,
                    )
                )
                points = load_path_csv(
                    filename,
                    gps_projection=MapProjection(
                        "EPSG:32652", origin_x, origin_y, 0.0
                    ),
                )
                used_projection = converter_type.call_args.args[0]

        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0].x_m, longitude_1)
        self.assertAlmostEqual(points[0].y_m, latitude_1)
        self.assertAlmostEqual(points[0].z_m, 28.0, places=3)
        self.assertAlmostEqual(points[1].x_m, longitude_2)
        self.assertAlmostEqual(points[1].y_m, latitude_2)
        self.assertEqual(used_projection.origin_x_m, origin_x)
        self.assertEqual(used_projection.origin_y_m, origin_y)

    def test_loads_documented_headerless_gps_sensor_text(self):
        origin_x, origin_y = 302595.0, 4124145.0
        latitude_1, longitude_1 = 37.1, 126.1
        latitude_2, longitude_2 = 37.2, 126.2
        with tempfile.TemporaryDirectory() as directory:
            filename = os.path.join(directory, "gps_path.txt")
            with open(filename, "w", encoding="utf-8") as stream:
                stream.write(
                    "{} {} 28.0 {} {}\n".format(
                        latitude_1, longitude_1, origin_x, origin_y
                    )
                )
                stream.write(
                    "{} {} 29.0 {} {}\n".format(
                        latitude_2, longitude_2, origin_x, origin_y
                    )
                )
            with patch("path_planning.stanley_controller.GpsToMapEnu") as converter_type:
                converter_type.return_value.convert.side_effect = (
                    lambda latitude, longitude, altitude: (
                        longitude,
                        latitude,
                        altitude,
                    )
                )
                points = load_path_csv(filename)

        self.assertAlmostEqual(points[0].x_m, longitude_1)
        self.assertAlmostEqual(points[0].y_m, latitude_1)
        self.assertAlmostEqual(points[1].z_m, 29.0, places=3)

    def test_vehicle_left_of_eastbound_path_steers_right(self):
        controller = StanleyController(
            [PathPoint(0.0, 0.0), PathPoint(20.0, 0.0)], gain=1.0
        )
        result = controller.compute(5.0, 2.0, 0.0, 0.0, 5.0)
        self.assertGreater(result.cross_track_error_m, 0.0)
        self.assertLess(result.steering_rad, 0.0)

    def test_heading_error_steers_toward_path_heading(self):
        controller = StanleyController([PathPoint(0.0, 0.0), PathPoint(20.0, 0.0)])
        result = controller.compute(5.0, 0.0, 0.0, math.radians(10.0), 5.0)
        self.assertLess(result.heading_error_rad, 0.0)
        self.assertLess(result.steering_rad, 0.0)

    def test_stanley_uses_nearest_segment_heading_without_lookahead(self):
        controller = StanleyController(
            [
                PathPoint(0.0, 0.0),
                PathPoint(10.0, 0.0),
                PathPoint(10.0, 10.0),
            ],
            control_point_offset_m=0.0,
            waypoint_smoothing_window=1,
        )
        result = controller.compute(8.0, 0.0, 0.0, 0.0, 2.0)
        self.assertAlmostEqual(result.path_yaw_rad, 0.0)
        self.assertAlmostEqual(result.heading_error_rad, 0.0)

    def test_waypoint_spacing_and_smoothing_are_applied(self):
        controller = StanleyController(
            [
                PathPoint(0.0, 0.0),
                PathPoint(0.1, 0.2),
                PathPoint(1.0, 1.0),
                PathPoint(2.0, 0.0),
            ],
            minimum_waypoint_spacing_m=0.5,
            waypoint_smoothing_window=3,
        )
        self.assertEqual(controller.original_point_count, 4)
        self.assertEqual(len(controller.points), 3)
        self.assertAlmostEqual(controller.points[1].y_m, 1.0 / 3.0)

    def test_target_index_does_not_move_backward_by_default(self):
        controller = StanleyController(
            [PathPoint(float(x), 0.0) for x in range(0, 101, 10)],
            control_point_offset_m=0.0,
            waypoint_smoothing_window=1,
        )
        forward = controller.compute(75.0, 0.0, 0.0, 0.0, 2.0)
        backward = controller.compute(5.0, 0.0, 0.0, 0.0, 2.0)
        self.assertGreaterEqual(backward.segment_index, forward.segment_index)

    def test_steering_filter_limits_rate_after_initial_sample(self):
        steering_filter = SteeringCommandFilter(
            alpha=1.0, max_rate_radps=0.4, max_abs_rad=1.0
        )
        self.assertEqual(steering_filter.update(0.0, 1.0), 0.0)
        self.assertAlmostEqual(steering_filter.update(1.0, 1.1), 0.04)

    def test_ins_runtime_defaults_to_fixed_ten_kmh_strict_stanley(self):
        arguments = argument_parser("ins").parse_args([])
        self.assertEqual(arguments.target_speed_kmh, 10.0)
        self.assertEqual(arguments.control_point_offset, 3.0)
        self.assertEqual(arguments.morai_steer_sign, 1.0)
        self.assertEqual(arguments.speed_kp, 0.35)
        self.assertFalse(hasattr(arguments, "path_yaw_lookahead"))
        self.assertEqual(arguments.target_search_window, 50)

    def test_planar_filter_requires_both_gps_and_imu(self):
        ekf = PlanarGpsImuEkf()
        ekf.add_gps(1.0, 10.0, 20.0, speed_mps=2.0, course_deg=90.0)
        self.assertIsNone(ekf.state_at(1.01))
        ekf.add_imu(1.02, 0.0, 0.0)
        state = ekf.state_at(1.03)
        self.assertIsNotNone(state)
        self.assertAlmostEqual(state.yaw_rad, 0.0, places=4)
        self.assertGreater(state.speed_mps, 0.0)

    def test_planar_filter_rejects_large_gps_jump(self):
        ekf = PlanarGpsImuEkf(gps_outlier_threshold_m=10.0)
        self.assertTrue(ekf.add_gps(1.0, 0.0, 0.0))
        self.assertFalse(ekf.add_gps(1.1, 1000.0, 1000.0))

    def test_speed_controller_never_commands_both_pedals(self):
        controller = PedalSpeedController(kp=0.2, ki=0.0)
        accel, brake = controller.compute(5.0, 2.0, 1.0)
        self.assertGreater(accel, 0.0)
        self.assertEqual(brake, 0.0)
        accel, brake = controller.compute(2.0, 5.0, 1.1)
        self.assertEqual(accel, 0.0)
        self.assertGreater(brake, 0.0)


if __name__ == "__main__":
    unittest.main()
