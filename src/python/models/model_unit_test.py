import math

import pytest
from config import robot_config
from ipc import session

from models import body_model, constants, twist_estimator
import geometry

@pytest.fixture
def model() -> body_model.BodyModel:
    config = session.get_robot_config()
    return body_model.BodyModel(config)


@pytest.fixture
def estimator() -> twist_estimator.TwistEstimator:
    config = session.get_robot_config()
    return twist_estimator.TwistEstimator(config)


@pytest.mark.parametrize(
    "linear_speed, angular_speed",
    # linear speed in m/s, angular speed in deg/s
    [(0.5, 0.0), (-0.5, 0.0), (0.0, 10.0), (0.0, -15.0), (0.5, 10.0)],
)
def test_tracking(
    model: body_model.BodyModel,
    estimator: twist_estimator.TwistEstimator,
    linear_speed: float,
    angular_speed: float,
) -> None:
    model.update(linear_speed, angular_speed)
    motors = session.get_robot_motors()
    for motor in motors:
        wheel_speed = model.velocity(motor)
        expected_speed = _calc_wheel_velocity(motor, linear_speed, angular_speed)
        assert expected_speed == wheel_speed
        estimator.update(motor.location, wheel_speed)

    est_twist = estimator.twist()
    expected_velocity = geometry.Velocity(geometry.BODY, linear_speed, 0, 0)
    assert est_twist.velocity.is_close(expected_velocity)
    expected_spin = geometry.AngularVelocity(geometry.BODY, 0, 0, angular_speed)
    assert est_twist.spin.is_close(expected_spin)


def _calc_wheel_velocity(
    motor: robot_config.Motor, linear_speed: float, angular_speed: float
) -> float:
    config = session.get_robot_config()
    wheel_circumference = config.rear_left_wheel.circumference
    half_track_width = config.track_width / 2

    expected_vel_sign = 1 if motor.side == "right" else -1

    angular_tread_speed = (
        math.radians(angular_speed)
        * half_track_width
        / constants.WHEEL_RESISTANCE_FACTOR
    )
    linear_tread_speed = expected_vel_sign * linear_speed
    tread_speed = angular_tread_speed + linear_tread_speed

    return tread_speed / wheel_circumference
