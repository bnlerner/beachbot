import numpy as np
import pytest
import scipy.spatial.transform as sst  # type: ignore[import-untyped]

import geometry


def test_position() -> None:
    p0 = geometry.Position(geometry.UTM, 1, 0)
    p1 = geometry.Position(geometry.UTM, 1, 1, -1)
    pzero = geometry.Position.zero(geometry.UTM)

    assert p0.distance(p1) == (1 + 1) ** 0.5
    assert np.allclose(p0.as_array(), np.array([1, 0, 0]))
    assert p1.magnitude == (1 + 1 + 1) ** 0.5
    assert not p0.is_close(p1)
    assert p0.is_close(p0)
    assert p1.to_2d() == geometry.Position(geometry.UTM, 1, 1, 0)
    assert pzero == geometry.Position(geometry.UTM, 0, 0, 0)
    assert p0 + p1 == geometry.Position(geometry.UTM, 2, 1, -1)
    assert p0 - p1 == geometry.Position(geometry.UTM, 0, -1, 1)
    assert p1 * 3 == geometry.Position(geometry.UTM, 3, 3, -3)


def test_orientation() -> None:
    ori0 = geometry.Orientation(geometry.UTM, 10, 15, -30)
    ori1 = geometry.Orientation(geometry.UTM, 0, 0, 0)

    assert ori1.x_axis() == geometry.Direction.unit_x(geometry.UTM)
    assert ori1.y_axis() == geometry.Direction.unit_y(geometry.UTM)
    assert ori1.z_axis() == geometry.Direction.unit_z(geometry.UTM)
    assert ori0.is_close(ori0)
    assert not ori0.is_close(ori1)
    assert ori0.to_2d() == geometry.Orientation(geometry.UTM, 0, 0, -30)
    assert ori0.is_close(geometry.Orientation.from_matrix(ori0.frame, ori0.as_matrix()))


def test_direction() -> None:
    dir0 = geometry.Direction.from_celestial(geometry.UTM, 0, 45)
    assert np.isclose(dir0.x, dir0.y)
    assert dir0.x == 0.5**0.5
    assert dir0.z == 0


def test_angular_velocity() -> None:
    spin = geometry.AngularVelocity(geometry.UTM, 2, 0, 1)
    assert spin.speed() == (2**2 + 1**2) ** 0.5


@pytest.mark.parametrize("roll, pitch, yaw", [(0, 0, 0), (0, 0, 90), (15, -15, 15)])
def test_extrinsic_rotation_matrix(roll: float, pitch: float, yaw: float) -> None:
    ori = geometry.Orientation(geometry.UTM, roll, pitch, yaw)
    rot = ori.as_rotation()
    rot_mat = rot.as_matrix()
    sst_rot = sst.Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True)
    assert np.allclose(rot_mat, sst_rot.as_matrix())

    rot_from_mat = geometry.Rotation.from_matrix(geometry.UTM, rot_mat)
    assert rot_from_mat.is_close(rot)


@pytest.mark.parametrize("roll, pitch, yaw", [(0, 0, 0), (20, 0, 20), (15, -15, 15)])
def test_intrinsic_rotation_matrix(roll: float, pitch: float, yaw: float) -> None:
    ori = geometry.Orientation.from_intrinsic_rpy(geometry.UTM, roll, pitch, yaw)
    rot = ori.as_rotation()
    rot_mat = rot.as_matrix()
    sst_rot = sst.Rotation.from_euler("zyx", [roll, pitch, yaw], degrees=True)

    assert np.allclose(rot_mat, sst_rot.as_matrix())

    rot_from_mat = geometry.Rotation.from_matrix(geometry.UTM, rot_mat)
    assert rot_from_mat.is_close(rot)


@pytest.mark.parametrize("angle", [0, 10, -45, 90, 130])
def test_rotating_objects(angle: float) -> None:
    p0 = geometry.Position(geometry.UTM, 1, 0)
    rot = geometry.Rotation(geometry.UTM, 0, 0, angle)
    p2 = p0.rotated(rot, intrinsic=False)

    assert np.isclose(p0.angle_to(p2), abs(angle))
    direction = geometry.Direction.from_celestial(geometry.UTM, 0, angle)
    ref_pos = direction.as_position()
    assert p2.is_close(ref_pos)


def test_to_local() -> None:
    p0 = geometry.Pose.zero(geometry.UTM)
    p1 = geometry.Pose(
        geometry.Position(geometry.UTM, 4, -2, 10),
        geometry.Orientation(geometry.UTM, 0, -90, 0),
    )

    assert p0.to_local(p1).is_close(p1)
    p2 = p1.to_local(p0)
    assert p2.orientation.pitch == 90
    assert np.isclose(p2.position.z, p1.position.x)
    assert p2.position.x == -p1.position.z


def test_from_local() -> None:
    pose = geometry.Pose(
        geometry.Position(geometry.UTM, 1, 1, 1),
        geometry.Orientation(geometry.UTM, 0, -45, 90),
    )
    pos = pose.from_local(1, 0, 0)
    assert pos == geometry.Position(geometry.UTM, 1, 1 + 0.5**0.5, 1 + 0.5**0.5)


def test_wrong_frame() -> None:
    p_utm = geometry.Position(geometry.UTM, 1, 0, -1)
    p_body = geometry.Position(geometry.BODY, 1, 0, -1)
    with pytest.raises(ValueError):
        _ = p_utm + p_body


@pytest.mark.parametrize("roll, pitch, yaw", [(0, 0, 0), (20, 0, 20), (15, -15, 15)])
def test_euler_to_quaternion(roll: float, pitch: float, yaw: float) -> None:
    sst_rot = sst.Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True)
    q_x, q_y, q_z, q_w = sst_rot.as_quat()
    w, x, y, z = geometry.euler_to_quaternion(roll, pitch, yaw)
    test_roll, test_pitch, test_yaw = geometry.quaternion_to_euler(w, x, y, z)

    assert np.isclose(q_x, x)
    assert np.isclose(q_y, y)
    assert np.isclose(q_z, z)
    assert np.isclose(q_w, w)
    assert np.isclose(test_roll, roll)
    assert np.isclose(test_pitch, pitch)
    assert np.isclose(test_yaw, yaw)
