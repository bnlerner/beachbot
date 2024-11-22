import numpy as np
import pytest
import scipy.spatial.transform as sst  # type: ignore[import-untyped]

import geometry


def test_position() -> None:
    p0 = geometry.Position(geometry.UTM, 1, 0)
    p1 = geometry.Position(geometry.UTM, 1, 1, -1)
    pzero = geometry.Position.zero(geometry.UTM)

    assert p0.distance(p1) == (1 + 1) ** 0.5
    assert np.allclose(p0.as_vector(), np.array([1, 0, 0]))
    assert p1.magnitude == (1 + 1 + 1) ** 0.5
    assert p1.to_2d() == geometry.Position(geometry.UTM, 1, 1, 0)
    assert pzero == geometry.Position(geometry.UTM, 0, 0, 0)
    assert p0 + p1 == geometry.Position(geometry.UTM, 2, 1, -1)
    assert p0 - p1 == geometry.Position(geometry.UTM, 0, -1, 1)
    assert p1 * 3 == geometry.Position(geometry.UTM, 3, 3, -3)


@pytest.mark.parametrize("roll, pitch, yaw", [(0, 0, 0), (0, 0, 90), (15, -15, 15)])
def test_rotation_matrix(roll: float, pitch: float, yaw: float) -> None:
    ori = geometry.Orientation(geometry.UTM, roll, pitch, yaw)
    rot_mat = ori.as_rotation().as_matrix()
    sst_rot = sst.Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True)
    assert np.allclose(rot_mat, sst_rot.as_matrix())


def test_rotating_objects() -> None:
    p0 = geometry.Position(geometry.UTM, 1, 0)
    rot = geometry.Rotation(geometry.UTM, 0, 0, 90)
    p2 = p0.rotated(rot)
    # print(p0, p2)
    assert p0.angle_to(p2) == 90
    assert p2 == geometry.Position(geometry.UTM, 0, 1, -1)


# @pytest.mark.parametrize("")
def test_wrong_frame() -> None:
    p_utm = geometry.Position(geometry.UTM, 1, 0, -1)
    p_body = geometry.Position(geometry.BODY, 1, 0, -1)
    with pytest.raises(ValueError):
        _ = p_utm + p_body
