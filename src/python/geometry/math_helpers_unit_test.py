import math

import numpy as np
import scipy.spatial.transform as sst  # type: ignore[import-untyped]

from geometry import math_helpers


def test_extrinsic_xyz_rotation_matrix() -> None:
    angle_values = [15.0, -15.0, 15.0]
    sst_rot = sst.Rotation.from_euler("xyz", angle_values, degrees=True).as_matrix()
    rot_mat = math_helpers.extrinsic_xyz_rotation_matrix(
        *[math.radians(val) for val in angle_values]
    )

    assert np.allclose(sst_rot, rot_mat)
    euler_angles = [math.degrees(angle) for angle in math_helpers.as_euler_xyz(rot_mat)]
    for a0, a1 in zip(euler_angles, angle_values, strict=True):
        assert np.isclose(a0, a1)


def test_intrinsic_xyz_rotation_matrix() -> None:
    angle_values = [15.0, -15.0, 15.0]
    sst_rot = sst.Rotation.from_euler("zyx", angle_values, degrees=True).as_matrix()
    rot_mat = math_helpers.intrinsic_xyz_rotation_matrix(
        *[math.radians(val) for val in angle_values]
    )

    assert np.allclose(sst_rot, rot_mat)
