import geometry
import pytest
from ipc import session

from planning import nav_vel_target_generator

_ZERO_POSE = geometry.Pose.zero(geometry.BODY)
_ZERO_POS = geometry.Position.zero(geometry.BODY)
_ZERO_ORI = geometry.Orientation.zero(geometry.BODY)


@pytest.fixture
def target_generator() -> nav_vel_target_generator.NavVelTargetGenerator:
    config = session.get_robot_config()
    return nav_vel_target_generator.NavVelTargetGenerator(config)


@pytest.mark.parametrize(
    "target_in_body, reference_speed",
    [
        (geometry.Pose(geometry.Position(geometry.BODY, 0.1, 0, 0), _ZERO_ORI), 1.0),
        (geometry.Pose(geometry.Position(geometry.BODY, -0.1, 0, 0), _ZERO_ORI), -1.0),
    ],
)
def test_straight_path(
    target_generator: nav_vel_target_generator.NavVelTargetGenerator,
    target_in_body: geometry.Pose,
    reference_speed: float,
) -> None:
    target_twist = target_generator.gen_twist(None, reference_speed, target_in_body)
    if reference_speed > 0:
        assert target_twist.velocity.x > reference_speed
    else:
        assert target_twist.velocity.x < reference_speed
    assert target_twist.velocity.y == 0.0
    assert target_twist.velocity.z == 0.0
    assert target_twist.spin.speed() == 0.0


@pytest.mark.parametrize("signed_turn_radius, reference_speed", [(10, 1.0)])
def test_twist_reference(
    target_generator: nav_vel_target_generator.NavVelTargetGenerator,
    signed_turn_radius: float,
    reference_speed: float,
) -> None:
    target_twist = target_generator.gen_twist(
        signed_turn_radius, reference_speed, _ZERO_POSE
    )
    print(target_twist)


@pytest.mark.parametrize(
    "x, y, yaw",
    [
        (0.1, 0.0, 0.0),
        (-0.1, 0.0, 0.0),
        (0.0, 0.1, 0.0),
        (0.0, -0.1, 0.0),
        (0.2, -0.1, 0.0),
        (0.0, 0.0, 10.0),
        (0.0, 0.0, -14.0),
    ],
)
def test_twist_correction(
    target_generator: nav_vel_target_generator.NavVelTargetGenerator,
    x: float,
    y: float,
    yaw: float,
) -> None:
    target_in_body = geometry.Pose(
        geometry.Position(geometry.BODY, x, y, 0),
        geometry.Orientation(geometry.BODY, 0, 0, yaw),
    )
    target_twist = target_generator.gen_twist(1.0, 0.0, target_in_body)
    if (x_sign := geometry.sign(target_in_body.position.x)) == 1:
        assert target_twist.velocity.x > 0
    elif x_sign == -1:
        assert target_twist.velocity.x < 0
    else:
        assert target_twist.velocity.x == 0

    if (y_sign := geometry.sign(target_in_body.position.y)) == 1:
        assert target_twist.spin.z > 0
    elif y_sign == -1:
        assert target_twist.spin.z < 0
    elif (yaw_sign := geometry.sign(target_in_body.orientation.yaw)) == 1:
        if target_twist.spin.z < 0:
            print(target_in_body)
        assert target_twist.spin.z > 0
    elif yaw_sign == -1:
        assert target_twist.spin.z < 0
    else:
        assert target_twist.spin.z == 0
