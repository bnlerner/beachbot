import math
from typing import Literal

import numba as nb  # type: ignore[import-untyped]
import numpy as np

_F64_MATRIX = nb.types.Array(nb.types.float64, 2, "C")
_F64_VECTOR = nb.types.Array(nb.types.float64, 1, "C")


def sign(val: float) -> Literal[-1, 0, 1]:
    if val > 0.0:
        return 1
    elif val < 0.0:
        return -1
    else:
        return 0


# @nb.njit(nb.float64(nb.float64))
def wrap_degrees(angle: float) -> float:
    return ((angle + 180) % 360) - 180


# @nb.njit(nb.float64(nb.float64))
def wrap_radian(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi  # Wraps to [-pi, pi]


# @nb.njit(_F64_VECTOR(_F64_VECTOR, _F64_VECTOR))
# def cross_3d_vector(vector_1: np.ndarray, vector_2: np.ndarray) -> np.ndarray:
#     """Equivalent to np.cross with two 3d vectors.

#     Note that this function could be made faster (about 50% speedup) if we pass in the
#     container for the result as an argument instead of creating and returning the
#     result. Ultimately decided that the returning the result is the code style that most
#     devs are used to and the decided that the perf hit was worth it.
#     """
#     result = np.zeros(3)

#     a1, a2, a3 = vector_1[0], vector_1[1], vector_1[2]
#     b1, b2, b3 = vector_2[0], vector_2[1], vector_2[2]

#     result[0] = a2 * b3 - a3 * b2
#     result[1] = a3 * b1 - a1 * b3
#     result[2] = a1 * b2 - a2 * b1

#     return result


# @nb.njit(nb.float64(_F64_VECTOR))
# def norm_3d_vector(vector: np.ndarray) -> float:
#     """Equivalent to np.linalg.norm."""
#     return math.sqrt(
#         vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]
#     )


# @nb.njit((nb.int16[:, :], nb.int64[:], nb.int64[:], nb.int16))
# def all_equal(
#     array: np.ndarray, rows: np.ndarray, cols: np.ndarray, value: int
# ) -> bool:
#     """True if all elements in the array, at the given rows and columns, equal the given
#     value. This is slightly faster than numpy array indexing and all-equal check.
#     """
#     for row, col in zip(rows, cols, strict=True):
#         if not array[row, col] == value:
#             return False
#     return True


# @nb.njit(_F64_MATRIX(nb.float64, nb.float64, nb.float64))
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


# @nb.njit(_F64_MATRIX(nb.float64, nb.float64, nb.float64))
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


# @nb.njit(_F64_MATRIX(nb.float64))
# def x_axis_transform(angle: float) -> np.ndarray:
#     """A 4x4 homogeneous transform matrix representation of a rigid body configuration in
#     SE3 (Special Euclidean group). Rotates about the x-axis by the angle specified. Can
#     be multiplied by other transforms to represent a frame configuration.

#     Note:
#     https://faculty.sites.iastate.edu/jia/files/inline-files/homogeneous-transform.pdf
#     https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/
#     """
#     transform = np.eye(4)
#     cos_val = np.cos(angle)
#     sin_val = np.sin(angle)
#     transform[1, 1] = cos_val
#     transform[1, 2] = -sin_val

#     transform[2, 1] = sin_val
#     transform[2, 2] = cos_val

#     return transform


# @nb.njit(_F64_MATRIX(nb.float64))
# def y_axis_transform(angle: float) -> np.ndarray:
#     """A 4x4 homogeneous transform matrix representation of a rigid body configuration in
#     SE3 (Special Euclidean group). Rotates about the y-axis by the angle specified. Can
#     be multiplied by other transforms to represent a frame configuration.

#     Note:
#     https://faculty.sites.iastate.edu/jia/files/inline-files/homogeneous-transform.pdf
#     https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/
#     """
#     transform = np.eye(4)
#     cos_val = np.cos(angle)
#     sin_val = np.sin(angle)
#     transform[0, 0] = cos_val
#     transform[2, 0] = -sin_val

#     transform[0, 2] = sin_val
#     transform[2, 2] = cos_val

#     return transform


# @nb.njit(_F64_MATRIX(nb.float64))
# def z_axis_transform(angle: float) -> np.ndarray:
#     """A 4x4 homogeneous transform matrix representation of a rigid body configuration in
#     SE3 (Special Euclidean group). Rotates about the z-axis by the angle specified. Can
#     be multiplied by other transforms to represent a frame configuration.

#     Note:
#     https://faculty.sites.iastate.edu/jia/files/inline-files/homogeneous-transform.pdf
#     https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/
#     """
#     transform = np.eye(4)
#     cos_val = np.cos(angle)
#     sin_val = np.sin(angle)
#     transform[0, 0] = cos_val
#     transform[1, 0] = sin_val

#     transform[0, 1] = -sin_val
#     transform[1, 1] = cos_val
#     return transform


# @nb.jit
def dot(array_1: np.ndarray, array_2: np.ndarray) -> np.ndarray:
    """Numba version of dot product. Somewhat faster than the numpy version.

    The explicit signature is not given since we want to take the dot products of
    matrices and vectors, matrices and matrices, vectors and matrices etc.
    Numba will compile the specific version when needed.
    """
    return np.dot(array_1, array_2)


# @nb.njit(_F64_MATRIX(_F64_MATRIX))
def transpose(matrix: np.ndarray) -> np.ndarray:
    """Numba version of transpose with numba. Somewhat faster than the numpy version.

    Store the array in C order so that it can be used in numba functions.
    """
    return np.ascontiguousarray(matrix.T)


# @nb.njit(_F64_MATRIX(_F64_VECTOR))
# def translation_transform(coord: np.ndarray) -> np.ndarray:
#     """A 4x4 homogeneous transform matrix representation of a rigid body configuration in
#     SE3 (Special Euclidean group). Translates by the vector specified. Can be
#     multiplied by other transforms to represent a frame configuration.

#     Note:
#     https://faculty.sites.iastate.edu/jia/files/inline-files/homogeneous-transform.pdf
#     https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/
#     """
#     rot = np.eye(4)
#     rot[0, 3] = coord[0]
#     rot[1, 3] = coord[1]
#     rot[2, 3] = coord[2]
#     return rot


# @nb.njit(_F64_VECTOR(_F64_MATRIX))
# def VecSO3(w_hat: np.ndarray) -> np.ndarray:
#     """The mapping from the Lie Algegra skew symmetric matrix so(3) to R3."""
#     w = np.zeros(3)
#     w[0] = w_hat[2, 1]
#     w[1] = w_hat[0, 2]
#     w[2] = w_hat[1, 0]
#     return w


# @nb.njit(_F64_VECTOR(_F64_MATRIX))
# def log_SO3(rot: np.ndarray) -> np.ndarray:
#     """The mapping SO(3) -> so(3) -> R3.

#     The vector in R3 is the rotation vector of the rotation matrix R.
#     """
#     r_trace = np.trace(rot)
#     if _is_close(r_trace, 3.0):
#         return np.zeros(3)

#     phy = np.arccos((r_trace - 1.0) / 2.0)
#     if abs(phy) > np.pi:
#         raise ValueError("Angle larger than pi")

#     if _is_close(phy, 0.0):
#         w = np.zeros(3)
#     elif _is_close(phy, np.pi):
#         A = (rot - np.eye(3)) / 2.0

#         A_11 = A[0, 0]
#         A_22 = A[1, 1]
#         A_33 = A[2, 2]

#         w1 = np.sqrt(-((A_22 + A_33 - A_11) / 2.0))
#         w2 = np.sqrt(-((A_11 + A_33 - A_22) / 2.0))
#         w3 = np.sqrt(-((A_11 + A_22 - A_33) / 2.0))

#         A_12 = A[0, 1]
#         A_13 = A[0, 2]
#         A_23 = A[1, 2]
#         if not _is_close(w1, 0):
#             if A_12 < 0:
#                 w2 = -w2
#             if A_13 < 0:
#                 w3 = -w3

#         elif not _is_close(w2, 0):
#             if A_23 < 0:
#                 w3 = -w3

#         w = np.array([w1, w2, w3]) * phy
#     else:
#         w_hat = (rot - np.transpose(rot)) / (2.0 * np.sin(phy)) * phy
#         w = VecSO3(w_hat)

#     if np.any(np.isnan(w)):
#         raise Exception

#     return w


# @nb.njit(_F64_VECTOR(_F64_MATRIX, _F64_MATRIX))
# def residual_SO3(r_1: np.ndarray, r_2: np.ndarray) -> np.ndarray:
#     """Gives the rotation vector that rotates R_2 to R_1.

#     Derived from R_1 = R_2 exp_SO3(w), where exp_SO3 is inverse mapping of log_SO3.
#     """
#     return log_SO3(np.dot(np.transpose(r_2), r_1))


# @nb.njit(_F64_VECTOR(_F64_MATRIX, _F64_MATRIX))
# def residual_SE3(pose_SE3: np.ndarray, target_SE3: np.ndarray) -> np.ndarray:
#     """Gives the difference between objects in SE3 = (SO3, R3)."""
#     r = np.zeros(6)

#     # Slices of arrays are not contiguous arrays
#     p_t, R_t = target_SE3[:3, 3], np.ascontiguousarray(target_SE3[:3, :3])
#     p_fk, R_fk = pose_SE3[:3, 3], np.ascontiguousarray(pose_SE3[:3, :3])

#     r[:3] = p_fk - p_t
#     r[3:] = residual_SO3(R_fk, R_t)

#     return r


# Functions that need to be compiled before functions that depend it
# @nb.njit(nb.boolean(nb.float64, nb.float64))
# def _is_close(a: float, b: float) -> bool:
#     return np.abs(a - b) < 1.0e-15


# @nb.njit(_F64_VECTOR(_F64_MATRIX))
def as_euler_xyz(rot: np.ndarray) -> np.ndarray:
    # Should never be in gimbal lock so no need to define.
    phi = wrap_radian(math.atan2(rot[2, 1], rot[2, 2]))
    theta = wrap_radian(math.asin(-rot[2, 0]))
    psi = wrap_radian(math.atan2(rot[1, 0], rot[0, 0]))
    return np.array([phi, theta, psi])
