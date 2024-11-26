from unittest import mock

import geometry
import pytest
from config import robot_config
from ipc import session

from controls import nav_velocity_controller

_PERIOD = 1 / 50

_STOPPED = geometry.Twist.zero(geometry.BODY)
_ZERO_VELOCITY = geometry.Velocity.zero(geometry.BODY)
_ZERO_SPIN = geometry.AngularVelocity.zero(geometry.BODY)
_FORWARD = geometry.Twist(geometry.Velocity(geometry.BODY, 1, 0, 0), _ZERO_SPIN)
_REVERSE = geometry.Twist(geometry.Velocity(geometry.BODY, -1, 0, 0), _ZERO_SPIN)
_TURN_RIGHT = geometry.Twist(
    _ZERO_VELOCITY, geometry.AngularVelocity(geometry.BODY, 0, 0, -10)
)
_TURN_LEFT = geometry.Twist(
    _ZERO_VELOCITY, geometry.AngularVelocity(geometry.BODY, 0, 0, 10)
)


@pytest.fixture
def controller() -> nav_velocity_controller.NavVelocityController:
    config = session.get_robot_config()
    return nav_velocity_controller.NavVelocityController(config)


def test_zero_twist(controller: nav_velocity_controller.NavVelocityController) -> None:
    controller.update(_STOPPED, _STOPPED)
    motors = session.get_robot_motors()
    for motor in motors:
        vel = controller.velocity(motor)
        assert vel == 0.0


@pytest.mark.parametrize(
    "target, measured",
    [
        (_FORWARD, _STOPPED),
        (_REVERSE, _STOPPED),
        (_TURN_LEFT, _STOPPED),
        (_TURN_RIGHT, _STOPPED),
        (_STOPPED, _FORWARD),
    ],
)
def test_non_zero_twist(
    controller: nav_velocity_controller.NavVelocityController,
    target: geometry.Twist,
    measured: geometry.Twist,
) -> None:
    controller.update(target, measured)
    motors = session.get_robot_motors()
    for motor in motors:
        vel = controller.velocity(motor)
        assert vel != 0.0


def test_increases_linearly(
    controller: nav_velocity_controller.NavVelocityController,
) -> None:
    motor = robot_config.Motor(
        node_id=0, location=robot_config.DrivetrainLocation.FRONT_RIGHT
    )
    prev_vel = 0.0
    for ix in range(500):
        t = ix * _PERIOD
        with mock.patch("time.perf_counter", mock.MagicMock(return_value=t)):
            controller.update(_FORWARD, _STOPPED)

        vel = controller.velocity(motor)
        assert vel > prev_vel
        prev_vel = vel
