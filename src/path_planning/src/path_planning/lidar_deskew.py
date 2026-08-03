"""Planar motion compensation for ego-frame MORAI LiDAR packet batches."""

import math

from path_planning.morai_udp_localization_pose import LocalizationPose


POINT_X = 0
POINT_Y = 1
POINT_Z = 2
POINT_DISTANCE = 3
POINT_INTENSITY = 4
POINT_RING = 5


def wrap_angle(angle_rad):
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def interpolate_pose(first, second, timestamp):
    duration = second.timestamp_monotonic_s - first.timestamp_monotonic_s
    if duration <= 0.0:
        return second
    alpha = max(
        0.0,
        min(1.0, (timestamp - first.timestamp_monotonic_s) / duration),
    )
    yaw_delta = wrap_angle(second.yaw_rad - first.yaw_rad)
    return LocalizationPose(
        timestamp_monotonic_s=timestamp,
        x_m=first.x_m + alpha * (second.x_m - first.x_m),
        y_m=first.y_m + alpha * (second.y_m - first.y_m),
        z_m=first.z_m + alpha * (second.z_m - first.z_m),
        yaw_rad=wrap_angle(first.yaw_rad + alpha * yaw_delta),
        speed_mps=first.speed_mps + alpha * (second.speed_mps - first.speed_mps),
        yaw_rate_radps=(
            first.yaw_rate_radps
            + alpha * (second.yaw_rate_radps - first.yaw_rate_radps)
        ),
    )


def extrapolate_pose(pose, timestamp):
    dt = timestamp - pose.timestamp_monotonic_s
    mid_yaw = pose.yaw_rad + 0.5 * pose.yaw_rate_radps * dt
    return LocalizationPose(
        timestamp_monotonic_s=timestamp,
        x_m=pose.x_m + pose.speed_mps * math.cos(mid_yaw) * dt,
        y_m=pose.y_m + pose.speed_mps * math.sin(mid_yaw) * dt,
        z_m=pose.z_m,
        yaw_rad=wrap_angle(pose.yaw_rad + pose.yaw_rate_radps * dt),
        speed_mps=pose.speed_mps,
        yaw_rate_radps=pose.yaw_rate_radps,
    )


def pose_at(pose_samples, timestamp, extrapolation_limit_s):
    if not pose_samples:
        return None

    samples = list(pose_samples)
    first = samples[0]
    if timestamp <= first.timestamp_monotonic_s:
        if first.timestamp_monotonic_s - timestamp > extrapolation_limit_s:
            return None
        return extrapolate_pose(first, timestamp)

    for previous, current in zip(samples, samples[1:]):
        if timestamp <= current.timestamp_monotonic_s:
            return interpolate_pose(previous, current, timestamp)

    latest = samples[-1]
    if timestamp - latest.timestamp_monotonic_s > extrapolation_limit_s:
        return None
    return extrapolate_pose(latest, timestamp)


def deskew_scan(packet_batches, reference_pose):
    """Transform packet-local points into the ego frame at ``reference_pose``."""
    raw_points = [point for points, _pose in packet_batches for point in points]
    if reference_pose is None or any(pose is None for _points, pose in packet_batches):
        return raw_points, False

    reference_cos = math.cos(reference_pose.yaw_rad)
    reference_sin = math.sin(reference_pose.yaw_rad)
    deskewed = []
    for points, pose in packet_batches:
        pose_cos = math.cos(pose.yaw_rad)
        pose_sin = math.sin(pose.yaw_rad)
        for point in points:
            world_x = pose.x_m + pose_cos * point[POINT_X] - pose_sin * point[POINT_Y]
            world_y = pose.y_m + pose_sin * point[POINT_X] + pose_cos * point[POINT_Y]
            delta_x = world_x - reference_pose.x_m
            delta_y = world_y - reference_pose.y_m
            x_forward = reference_cos * delta_x + reference_sin * delta_y
            y_left = -reference_sin * delta_x + reference_cos * delta_y
            z_up = pose.z_m + point[POINT_Z] - reference_pose.z_m
            distance_m = math.sqrt(
                x_forward * x_forward + y_left * y_left + z_up * z_up
            )
            bearing_deg = math.degrees(math.atan2(y_left, x_forward))
            deskewed.append(
                (
                    float(x_forward),
                    float(y_left),
                    float(z_up),
                    float(distance_m),
                    point[POINT_INTENSITY],
                    point[POINT_RING],
                    float(bearing_deg),
                )
            )
    return deskewed, True
