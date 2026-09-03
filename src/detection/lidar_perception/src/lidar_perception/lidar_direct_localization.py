"""Standalone GPS/IMU localization used by the UDP LiDAR deskew node."""

from lidar_perception.coordinates import GeodeticOrigin, GpsToRecordedLocalEnu
from lidar_perception.localization import PlanarGpsImuEkf
from lidar_perception.morai_udp_gps import parse_nmea_datagram
from lidar_perception.morai_udp_imu import parse_imu_packet, quaternion_to_yaw
from lidar_perception.morai_udp_localization_pose import LocalizationPose


class DirectGpsImuPoseEstimator:
    """Build a locally referenced planar pose directly from MORAI UDP sensors."""

    def __init__(self):
        self._converter = None
        self._altitude_origin_m = None
        self._localizer = PlanarGpsImuEkf()
        self._latest_yaw_rate_radps = 0.0

    def _pose_at(self, timestamp):
        state = self._localizer.state_at(timestamp)
        if state is None:
            return None
        return LocalizationPose(
            timestamp_monotonic_s=state.timestamp,
            x_m=state.x_m,
            y_m=state.y_m,
            z_m=state.z_m,
            yaw_rad=state.yaw_rad,
            speed_mps=state.speed_mps,
            yaw_rate_radps=self._latest_yaw_rate_radps,
        )

    def add_gps_packet(self, packet, received_at):
        measurement = parse_nmea_datagram(packet)
        if not measurement.fix_valid:
            return None
        if self._converter is None:
            self._converter = GpsToRecordedLocalEnu(
                GeodeticOrigin(
                    measurement.latitude_deg,
                    measurement.longitude_deg,
                )
            )
        x_m, y_m, _unused_z_m = self._converter.convert(
            measurement.latitude_deg,
            measurement.longitude_deg,
            None,
        )
        z_m = None
        if measurement.altitude_m is not None:
            if self._altitude_origin_m is None:
                self._altitude_origin_m = measurement.altitude_m
            z_m = measurement.altitude_m - self._altitude_origin_m
        accepted = self._localizer.add_gps(
            received_at,
            x_m,
            y_m,
            z_m,
            measurement.speed_mps,
            measurement.course_deg,
        )
        if not accepted:
            return None
        return self._pose_at(received_at)

    def add_imu_packet(self, packet, received_at):
        measurement = parse_imu_packet(packet)
        self._latest_yaw_rate_radps = measurement.angular_velocity_radps[2]
        self._localizer.add_imu(
            received_at,
            quaternion_to_yaw(measurement.orientation_xyzw),
            self._latest_yaw_rate_radps,
        )
        return self._pose_at(received_at)
