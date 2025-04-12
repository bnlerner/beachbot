import math

import numpy as np
import pytest
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


def test_wrap_degrees() -> None:
    assert math_helpers.wrap_degrees(0) == 0
    assert math_helpers.wrap_degrees(45) == 45
    assert math_helpers.wrap_degrees(360) == 0
    assert math_helpers.wrap_degrees(361) == 1
    assert math_helpers.wrap_degrees(-1) == -1
    assert math_helpers.wrap_degrees(-361) == -1


def test_sign() -> None:
    assert math_helpers.sign(10) == 1
    assert math_helpers.sign(-10) == -1
    assert math_helpers.sign(0) == 0


@pytest.mark.parametrize(
    "w, x, y, z",
    [
        # Identity quaternion
        (1.0, 0.0, 0.0, 0.0),
        # 90 degree rotation around X
        (0.7, 0.7071068, 0.0, 0.0),
        # ~90 degree rotation around Y. Due to floating point precision, this is not exactly 90.
        (0.7071068, 0.01, 0.7071068, 0.0),
        # 90 degree rotation around Z
        (0.7071068, 0.0, 0.0, 0.7071068),
    ],
)
def test_quaternion_to_rotation_matrix(w: float, x: float, y: float, z: float) -> None:
    sst_rot = sst.Rotation.from_quat([x, y, z, w]).as_euler("xyz", degrees=True)
    rot_vec = math_helpers.quaternion_to_euler(w, x, y, z)
    assert np.allclose(sst_rot, rot_vec, atol=1e-10)


@pytest.mark.parametrize(
    "roll, pitch, yaw",
    [
        # Identity rotation
        (0.0, 0.0, 0.0),
        # 90 degree rotation around X
        (90.0, 0.0, 0.0),
        # 90 degree rotation around Y
        (0.0, 90.0, 0.0),
        # 90 degree rotation around Z
        (0.0, 0.0, 90.0),
    ],
)
def test_rotation_matrix_to_quaternion(roll: float, pitch: float, yaw: float) -> None:
    """Test conversion from rotation matrix to quaternion."""
    w, x, y, z = math_helpers.euler_to_quaternion(roll, pitch, yaw)
    sst_rot = sst.Rotation.from_euler("xyz", (roll, pitch, yaw), degrees=True)
    sst_x, sst_y, sst_z, sst_w = sst_rot.as_quat()
    assert np.allclose([w, x, y, z], [sst_w, sst_x, sst_y, sst_z])
