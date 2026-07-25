"""Small quaternion helpers shared by MORAI localization filters."""

import math

import numpy as np


def normalize_quaternion(quaternion_xyzw):
    quaternion = np.asarray(quaternion_xyzw, dtype=float).reshape(4)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("quaternion must have a finite non-zero norm")
    return quaternion / norm


def quaternion_conjugate(quaternion_xyzw):
    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    return np.array([-x, -y, -z, w], dtype=float)


def quaternion_multiply(left_xyzw, right_xyzw):
    x1, y1, z1, w1 = np.asarray(left_xyzw, dtype=float).reshape(4)
    x2, y2, z2, w2 = np.asarray(right_xyzw, dtype=float).reshape(4)
    return normalize_quaternion(
        (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )
    )


def quaternion_from_rotation_vector(rotation_vector):
    vector = np.asarray(rotation_vector, dtype=float).reshape(3)
    angle = np.linalg.norm(vector)
    if angle < 1e-12:
        return normalize_quaternion((0.5 * vector[0], 0.5 * vector[1], 0.5 * vector[2], 1.0))
    half = 0.5 * angle
    xyz = vector * (math.sin(half) / angle)
    return np.array((xyz[0], xyz[1], xyz[2], math.cos(half)), dtype=float)


def quaternion_to_rotation_vector(quaternion_xyzw):
    quaternion = normalize_quaternion(quaternion_xyzw)
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    xyz = quaternion[:3]
    magnitude = np.linalg.norm(xyz)
    if magnitude < 1e-12:
        return 2.0 * xyz
    angle = 2.0 * math.atan2(magnitude, quaternion[3])
    return xyz * (angle / magnitude)


def quaternion_to_matrix(quaternion_xyzw):
    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quaternion_yaw(quaternion_xyzw):
    matrix = quaternion_to_matrix(quaternion_xyzw)
    return math.atan2(matrix[1, 0], matrix[0, 0])


def quaternion_error(measured_xyzw, nominal_xyzw):
    """Return the small body-frame rotation taking nominal to measured."""
    relative = quaternion_multiply(quaternion_conjugate(nominal_xyzw), measured_xyzw)
    return quaternion_to_rotation_vector(relative)


def apply_body_rotation(quaternion_xyzw, rotation_vector):
    return quaternion_multiply(
        quaternion_xyzw, quaternion_from_rotation_vector(rotation_vector)
    )


def skew(vector):
    x, y, z = np.asarray(vector, dtype=float).reshape(3)
    return np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)), dtype=float)
