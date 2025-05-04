import math
from typing import Collection, Literal, Tuple

import numba as nb  # type: ignore[import-untyped]
import numpy as np

_F64_MATRIX = nb.types.Array(nb.types.float64, 2, "C")
_F64_VECTOR = nb.types.Array(nb.types.float64, 1, "C")
# Default ATOL, same as numpy.
DEFAULT_ATOL = 1e-8


def sign(val: float) -> Literal[-1, 0, 1]:
    """The sign of the value. Similar but faster to np or math sign."""
    if val > 0.0:
        return 1
    elif val < 0.0:
        return -1
    else:
        return 0


def linear_ramp(val: float, width: float = 1) -> float:
    """A ramped value by the width between 0 and 1."""
    return clip(val / width, 0.0, 1.0)


def clip(val: float, lower: float, upper: float) -> float:
    """Clips the value between the lower and upper."""
    if val < lower:
        return lower
    elif val > upper:
        return upper
    else:
        return val


def mean(values: Collection[float]) -> float:
    """The mean of the values."""
    return sum(values) / len(values)


def wrap_degrees(angle: float) -> float:
    """Wraps the angle to the range [-180, 180]."""
    return ((angle + 180) % 360) - 180


@nb.njit(nb.float64(nb.float64))
def _wrap_radian(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi  # Wraps to [-pi, pi]


@nb.njit(nb.float64(_F64_VECTOR))
def norm_3d_vector(vector: np.ndarray) -> float:
    """Equivalent to np.linalg.norm."""
    return math.sqrt(
        vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]
    )


@nb.njit(_F64_MATRIX(nb.float64, nb.float64, nb.float64))
def extrinsic_xyz_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """A much faster version of a sst.Rotation.from_euler("xyz", angles).as_matrix()
    The input angles of [roll, pitch, yaw] match to a rotation matrix defined as
    R = R_z(yaw) R_y(pitch) R_x(roll)

    The notation follows the reference:
    https://ntrs.nasa.gov/citations/19770019231 page 22
    The paper gives ZYX (intrinsic) which is equal XYZ (extrinsic).
    """
    rot = np.zeros((3, 3))

    c1 = np.cos(yaw)
    c2 = np.cos(pitch)
    c3 = np.cos(roll)

    s1 = np.sin(yaw)
    s2 = np.sin(pitch)
    s3 = np.sin(roll)

    # C order
    rot[0, 0] = c1 * c2
    rot[0, 1] = c1 * s2 * s3 - s1 * c3
    rot[0, 2] = c1 * s2 * c3 + s1 * s3

    rot[1, 0] = s1 * c2
    rot[1, 1] = s1 * s2 * s3 + c1 * c3
    rot[1, 2] = s1 * s2 * c3 - c1 * s3

    rot[2, 0] = -s2
    rot[2, 1] = c2 * s3
    rot[2, 2] = c2 * c3

    return rot


@nb.njit(_F64_MATRIX(nb.float64, nb.float64, nb.float64))
def intrinsic_xyz_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """A much faster version of an XYZ intrinsic rotation such as
    sst.Rotation.from_euler("zyx", angles).as_matrix(). The input angles of
    [roll, pitch, yaw] match to a rotation matrix defined as
    R = R_x(roll) R_y(pitch) R_z(yaw)

    The notation follows the reference:
    https://ntrs.nasa.gov/citations/19770019231 page 13 which gives XYZ (intrinsic).
    """
    rot = np.zeros((3, 3))

    c1 = np.cos(roll)
    c2 = np.cos(pitch)
    c3 = np.cos(yaw)

    s1 = np.sin(roll)
    s2 = np.sin(pitch)
    s3 = np.sin(yaw)

    # C order
    rot[0, 0] = c2 * c3
    rot[0, 1] = -c2 * s3
    rot[0, 2] = s2

    rot[1, 0] = s1 * s2 * c3 + c1 * s3
    rot[1, 1] = -s1 * s2 * s3 + c1 * c3
    rot[1, 2] = -s1 * c2

    rot[2, 0] = -c1 * s2 * c3 + s1 * s3
    rot[2, 1] = c1 * s2 * s3 + s1 * c3
    rot[2, 2] = c1 * c2

    return rot


@nb.njit(_F64_MATRIX(nb.float64))
def x_axis_transform(angle: float) -> np.ndarray:
    """A 4x4 homogeneous transform matrix representation of a rigid body configuration in
    SE3 (Special Euclidean group). Rotates about the x-axis by the angle specified. Can
    be multiplied by other transforms to represent a frame configuration.

    Note:
    https://faculty.sites.iastate.edu/jia/files/inline-files/homogeneous-transform.pdf
    https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/
    """
    transform = np.eye(4)
    cos_val = np.cos(angle)
    sin_val = np.sin(angle)
    transform[1, 1] = cos_val
    transform[1, 2] = -sin_val

    transform[2, 1] = sin_val
    transform[2, 2] = cos_val

    return transform


@nb.njit(_F64_MATRIX(nb.float64))
def y_axis_transform(angle: float) -> np.ndarray:
    """A 4x4 homogeneous transform matrix representation of a rigid body configuration in
    SE3 (Special Euclidean group). Rotates about the y-axis by the angle specified. Can
    be multiplied by other transforms to represent a frame configuration.

    Note:
    https://faculty.sites.iastate.edu/jia/files/inline-files/homogeneous-transform.pdf
    https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/
    """
    transform = np.eye(4)
    cos_val = np.cos(angle)
    sin_val = np.sin(angle)
    transform[0, 0] = cos_val
    transform[2, 0] = -sin_val

    transform[0, 2] = sin_val
    transform[2, 2] = cos_val

    return transform


@nb.njit(_F64_MATRIX(nb.float64))
def z_axis_transform(angle: float) -> np.ndarray:
    """A 4x4 homogeneous transform matrix representation of a rigid body configuration in
    SE3 (Special Euclidean group). Rotates about the z-axis by the angle specified. Can
    be multiplied by other transforms to represent a frame configuration.

    Note:
    https://faculty.sites.iastate.edu/jia/files/inline-files/homogeneous-transform.pdf
    https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/
    """
    transform = np.eye(4)
    cos_val = np.cos(angle)
    sin_val = np.sin(angle)
    transform[0, 0] = cos_val
    transform[1, 0] = sin_val

    transform[0, 1] = -sin_val
    transform[1, 1] = cos_val
    return transform


def transpose(matrix: np.ndarray) -> np.ndarray:
    """Transpose of a matrix."""
    return np.ascontiguousarray(matrix.T)


@nb.njit(_F64_MATRIX(_F64_VECTOR))
def translation_transform(coord: np.ndarray) -> np.ndarray:
    """A 4x4 homogeneous transform matrix representation of a rigid body configuration in
    SE3 (Special Euclidean group). Translates by the vector specified. Can be
    multiplied by other transforms to represent a frame configuration.

    Note:
    https://faculty.sites.iastate.edu/jia/files/inline-files/homogeneous-transform.pdf
    https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/
    """
    rot = np.eye(4)
    rot[0, 3] = coord[0]
    rot[1, 3] = coord[1]
    rot[2, 3] = coord[2]
    return rot


@nb.njit(_F64_VECTOR(_F64_MATRIX))
def VecSO3(w_hat: np.ndarray) -> np.ndarray:
    """The mapping from the Lie Algegra skew symmetric matrix so(3) to R3."""
    w = np.zeros(3)
    w[0] = w_hat[2, 1]
    w[1] = w_hat[0, 2]
    w[2] = w_hat[1, 0]
    return w


# Functions that need to be compiled before functions that depend it
@nb.njit
def is_close(a: float, b: float, atol: float = 1.0e-15) -> bool:
    return np.abs(a - b) < atol


@nb.njit(_F64_VECTOR(_F64_MATRIX))
def log_SO3(rot: np.ndarray) -> np.ndarray:
    """The mapping SO(3) -> so(3) -> R3.

    The vector in R3 is the rotation vector of the rotation matrix R.
    """
    r_trace = np.trace(rot)
    if is_close(r_trace, 3.0):
        return np.zeros(3)

    phy = np.arccos((r_trace - 1.0) / 2.0)
    if abs(phy) > np.pi:
        raise ValueError("Angle larger than pi")

    if is_close(phy, 0.0):
        w = np.zeros(3)
    elif is_close(phy, np.pi):
        A = (rot - np.eye(3)) / 2.0

        A_11 = A[0, 0]
        A_22 = A[1, 1]
        A_33 = A[2, 2]

        w1 = np.sqrt(-((A_22 + A_33 - A_11) / 2.0))
        w2 = np.sqrt(-((A_11 + A_33 - A_22) / 2.0))
        w3 = np.sqrt(-((A_11 + A_22 - A_33) / 2.0))

        A_12 = A[0, 1]
        A_13 = A[0, 2]
        A_23 = A[1, 2]
        if not is_close(w1, 0):
            if A_12 < 0:
                w2 = -w2
            if A_13 < 0:
                w3 = -w3

        elif not is_close(w2, 0):
            if A_23 < 0:
                w3 = -w3

        w = np.array([w1, w2, w3]) * phy
    else:
        w_hat = (rot - np.transpose(rot)) / (2.0 * np.sin(phy)) * phy
        w = VecSO3(w_hat)

    if np.any(np.isnan(w)):
        raise Exception

    return w


@nb.njit(_F64_VECTOR(_F64_MATRIX, _F64_MATRIX))
def residual_SO3(r_1: np.ndarray, r_2: np.ndarray) -> np.ndarray:
    """Gives the rotation vector that rotates R_2 to R_1.

    Derived from R_1 = R_2 exp_SO3(w), where exp_SO3 is inverse mapping of log_SO3.
    """
    return log_SO3(np.dot(np.transpose(r_2), r_1))


@nb.njit(_F64_VECTOR(_F64_MATRIX, _F64_MATRIX))
def residual_SE3(pose_se3: np.ndarray, target_se3: np.ndarray) -> np.ndarray:
    """Gives the difference between objects in SE3 = (SO3, R3)."""
    r = np.zeros(6)

    # Slices of arrays are not contiguous arrays
    p_t, R_t = target_se3[:3, 3], np.ascontiguousarray(target_se3[:3, :3])
    p_fk, R_fk = pose_se3[:3, 3], np.ascontiguousarray(pose_se3[:3, :3])

    r[:3] = p_fk - p_t
    r[3:] = residual_SO3(R_fk, R_t)

    return r


@nb.njit(_F64_VECTOR(_F64_MATRIX))
def as_euler_xyz(rot: np.ndarray) -> np.ndarray:
    # Should never be in gimbal lock so no need to define.
    phi = _wrap_radian(math.atan2(rot[2, 1], rot[2, 2]))
    theta = _wrap_radian(math.asin(-rot[2, 0]))
    psi = _wrap_radian(math.atan2(rot[1, 0], rot[0, 0]))
    return np.array([phi, theta, psi])


@nb.njit(_F64_VECTOR(nb.float64, nb.float64))
def celestial_coordinates(altitude: float, azimuth: float) -> np.ndarray:
    alt_cos = math.cos(altitude)

    x = alt_cos * math.cos(azimuth)
    y = alt_cos * math.sin(azimuth)
    z = math.sin(altitude)

    return np.array([x, y, z])


@nb.njit(_F64_VECTOR(nb.float64, nb.float64, nb.float64))
def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """A conversion from euler RPY in degrees to a quaternion in the format
    (w, x, y, z).
    """
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])


@nb.njit(_F64_VECTOR(nb.float64, nb.float64, nb.float64, nb.float64))
def quaternion_to_euler(w: float, x: float, y: float, z: float) -> np.ndarray:
    """A conversion from quaternion to euler RPY (roll, pitch, yaw) in degrees.
    Normalizes the quaternion for you before performing the conversion.
    """
    norm = (w**2 + x**2 + y**2 + z**2) ** 0.5
    w /= norm
    x /= norm
    y /= norm
    z /= norm

    # Roll (x-axis rotation)
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    # Pitch (y-axis rotation)
    sinp = math.sqrt(1 + 2 * (w * y - x * z))
    cosp = math.sqrt(1 - 2 * (w * y - x * z))
    pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

    # Yaw (z-axis rotation)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return np.array([math.degrees(roll), math.degrees(pitch), math.degrees(yaw)])


@nb.njit(nb.boolean(nb.types.UniTuple(nb.float64, 2), nb.float64[:, :]))
def polygon_contains(point: Tuple[float, float], polygon: np.ndarray) -> bool:
    """A raycasting algorithm to determine if the point is contained within the polygon
    represented as a numpy array with polygon points.
    """
    px, py = point
    intersection_count = 0
    n = polygon.shape[0]

    for i in range(n):
        x1, y1 = polygon[i]
        # Ensure the last edge connects to the first point
        x2, y2 = polygon[(i + 1) % n]

        # Check if the point is between y1 and y2
        if (py > min(y1, y2)) and (py <= max(y1, y2)) and (px <= max(x1, x2)):
            # Compute the intersection point's x-coordinate. Avoids division by zero.
            x_intersect = x1 + (py - y1) * (x2 - x1) / (y2 - y1) if y1 != y2 else x1

            # Count intersection if the point is to the left of it
            if px < x_intersect:
                intersection_count += 1

    return intersection_count % 2 == 1


@nb.njit(
    _F64_MATRIX(nb.float64, nb.float64, nb.float64, nb.float64, nb.float64, nb.float64)
)
def create_rpy_transform(
    x: float, y: float, z: float, roll: float, pitch: float, yaw: float
) -> np.ndarray:
    """Create a 4x4 homogeneous transformation matrix from xyz translation and roll,
    pitch, yaw rotations (degrees). First applies the rotations (roll around X, pitch
    around Y, yaw around Z) and then applies the translation.
    """
    # Create rotation matrix using extrinsic XYZ convention (same as roll, pitch, yaw)
    rot = extrinsic_xyz_rotation_matrix(
        math.radians(roll), math.radians(pitch), math.radians(yaw)
    )

    # Create homogeneous transformation matrix
    transform = np.eye(4)
    transform[:3, :3] = rot
    transform[0, 3] = x
    transform[1, 3] = y
    transform[2, 3] = z

    return transform
