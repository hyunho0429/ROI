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
    load_gps_path_projection,
    load_path_csv,
    load_recorded_path_origin,
)
from path_planning.pure_pursuit_udp_runtime import (
    apply_opposing_steering_offset,
    argument_parser,
    main,
)


class StanleyControllerTest(unittest.TestCase):
    def test_loads_competition_headerless_xyz_path(self):
        filename = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "data",
                "2026_molit_comp_global_path.txt",
            )
        )
        points = load_path_csv(filename)
        projection = load_gps_path_projection(
            filename, MapProjection("EPSG:32652", 302595.0, 4124145.0, 0.0)
        )

        self.assertEqual(len(points), 4392)
        self.assertIsNone(projection)
        self.assertAlmostEqual(points[0].x_m, -131.68979755061446)
        self.assertAlmostEqual(points[0].y_m, -428.3310229377821)
        self.assertAlmostEqual(points[0].z_m, 28.543960281954277)
        self.assertAlmostEqual(points[-1].x_m, points[0].x_m)
        self.assertAlmostEqual(points[-1].y_m, points[0].y_m)

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

    def test_five_column_gps_csv_uses_one_offset_for_path_and_live_gps(self):
        latitude_1, longitude_1 = 37.24098167, 126.77435500
        latitude_2, longitude_2 = 37.24099167, 126.77436500
        origin_x, origin_y = 302595.0, 4124145.0
        fallback = MapProjection("EPSG:32652", 1.0, 2.0, 20.0)
        with tempfile.TemporaryDirectory() as directory:
            filename = os.path.join(directory, "gps_five_columns.csv")
            with open(filename, "w", encoding="utf-8") as stream:
                stream.write("위도,경도,고도,동쪽 좌표,북쪽 좌표\n")
                stream.write(
                    "{},{},29.0,{},{}\n".format(
                        latitude_1, longitude_1, origin_x, origin_y
                    )
                )
                stream.write(
                    "{},{},29.5,{},{}\n".format(
                        latitude_2, longitude_2, origin_x, origin_y
                    )
                )
            projection = load_gps_path_projection(filename, fallback)
            with patch("path_planning.stanley_controller.GpsToMapEnu") as path_converter:
                path_converter.return_value.convert.side_effect = (
                    lambda latitude, longitude, altitude: (
                        longitude,
                        latitude,
                        altitude - projection.origin_z_m,
                    )
                )
                points = load_path_csv(filename, gps_projection=projection)
                path_used_projection = path_converter.call_args.args[0]
            # The runtime constructs its live converter with this exact
            # projection; comparing the projection is sufficient here and
            # keeps unit tests independent of the optional pyproj package.
            live_position = (longitude_1, latitude_1, 9.0)

        self.assertEqual(projection.crs, "EPSG:32652")
        self.assertEqual(projection.origin_x_m, origin_x)
        self.assertEqual(projection.origin_y_m, origin_y)
        self.assertEqual(projection.origin_z_m, 20.0)
        self.assertEqual(path_used_projection, projection)
        self.assertAlmostEqual(points[0].x_m, live_position[0], places=6)
        self.assertAlmostEqual(points[0].y_m, live_position[1], places=6)
        self.assertAlmostEqual(points[0].z_m, 9.0, places=6)

    def test_rejects_changing_gps_map_offsets(self):
        with tempfile.TemporaryDirectory() as directory:
            filename = os.path.join(directory, "invalid_gps_path.csv")
            with open(filename, "w", encoding="utf-8") as stream:
                stream.write("latitude,longitude,altitude,eastOffset,northOffset\n")
                stream.write("37.24,126.77,29,302595,4124145\n")
                stream.write("37.25,126.78,30,302596,4124145\n")
            with self.assertRaisesRegex(ValueError, "must be constant"):
                load_gps_path_projection(filename)

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

    def test_ins_runtime_defaults_to_competition_pure_pursuit(self):
        arguments = argument_parser("ins").parse_args([])
        self.assertEqual(arguments.target_speed_kmh, 30.0)
        self.assertEqual(arguments.gps_port, 3001)
        self.assertEqual(arguments.imu_port, 4001)
        self.assertEqual(arguments.competition_status_host_port, 9080)
        self.assertEqual(arguments.competition_status_port, 9081)
        self.assertEqual(arguments.collision_host_port, 9091)
        self.assertEqual(arguments.collision_port, 9092)
        self.assertEqual(arguments.control_ip, "192.168.56.1")
        self.assertEqual(arguments.control_port, 9093)
        self.assertEqual(arguments.control_source_port, 9094)
        self.assertEqual(arguments.control_protocol, "25s4")
        self.assertEqual(
            os.path.basename(arguments.path), "2026_molit_comp_global_path.txt"
        )
        self.assertEqual(arguments.control_point_offset, 0.0)
        self.assertEqual(arguments.steering_offset_deg, 3.0)
        self.assertEqual(arguments.wheelbase, 3.0)
        self.assertEqual(arguments.lookahead_distance, 2.0)
        self.assertEqual(arguments.lookahead_speed_gain, 0.5)
        self.assertEqual(arguments.minimum_lookahead, 2.0)
        self.assertEqual(arguments.maximum_lookahead, 12.0)
        self.assertEqual(arguments.steering_filter_alpha, 0.15)
        self.assertEqual(arguments.max_steering_rate_radps, 0.25)
        self.assertEqual(arguments.alignment_seconds, 2.0)
        self.assertEqual(arguments.alignment_min_samples, 20)
        self.assertEqual(arguments.morai_steer_sign, 1.0)
        self.assertEqual(arguments.control_rate_hz, 30.0)
        self.assertEqual(arguments.speed_kp, 0.075)
        self.assertEqual(arguments.speed_ki, 0.0001)
        self.assertEqual(arguments.speed_kd, 0.025)
        self.assertFalse(hasattr(arguments, "stanley_gain"))
        self.assertEqual(arguments.target_search_window, 50)

    def test_opposing_steering_offset_reduces_magnitude(self):
        max_abs_rad = math.radians(20.0)
        self.assertAlmostEqual(
            apply_opposing_steering_offset(
                math.radians(10.0), 3.0, max_abs_rad
            ),
            math.radians(7.0),
        )
        self.assertAlmostEqual(
            apply_opposing_steering_offset(
                math.radians(-10.0), 3.0, max_abs_rad
            ),
            math.radians(-7.0),
        )
        self.assertAlmostEqual(
            apply_opposing_steering_offset(
                math.radians(2.0), 3.0, max_abs_rad
            ),
            0.0,
        )

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
        controller = PedalSpeedController(kp=0.2, ki=0.0, kd=0.0)
        accel, brake = controller.compute(5.0, 2.0, 1.0)
        self.assertGreater(accel, 0.0)
        self.assertEqual(brake, 0.0)
        accel, brake = controller.compute(2.0, 5.0, 1.1)
        self.assertEqual(accel, 0.0)
        self.assertGreater(brake, 0.0)

    def test_speed_controller_uses_main_branch_pid_equation(self):
        controller = PedalSpeedController(
            kp=0.075,
            ki=0.0001,
            kd=0.025,
            nominal_dt=1.0 / 30.0,
        )
        accel, brake = controller.compute(3.0, 2.0, 1.0)
        expected = 0.075 + 0.0001 / 30.0 + 0.025 * 30.0
        self.assertAlmostEqual(accel, expected)
        self.assertEqual(brake, 0.0)

        accel, brake = controller.compute(3.0, 2.0, 1.0 + 1.0 / 30.0)
        self.assertAlmostEqual(accel, 0.075 + 0.0001 * 2.0 / 30.0)
        self.assertEqual(brake, 0.0)

    def test_roslaunch_remapping_arguments_are_ignored(self):
        with patch("path_planning.pure_pursuit_udp_runtime.run") as run_mock:
            main(
                "ins",
                [
                    "--target-speed-kmh",
                    "12.5",
                    "__name:=morai_pure_pursuit_ins_udp",
                    "__log:=/tmp/controller.log",
                ],
            )
        arguments = run_mock.call_args.args[1]
        self.assertEqual(arguments.target_speed_kmh, 12.5)


if __name__ == "__main__":
    unittest.main()
