import time

import pytest
from config import robot_config
from ipc import session

from models import body_model, twist_estimator


@pytest.fixture
def model() -> body_model.BodyModel:
    config = session.get_robot_config()
    return body_model.BodyModel(config)


@pytest.fixture
def estimator() -> twist_estimator.TwistEstimator:
    config = session.get_robot_config()
    return twist_estimator.TwistEstimator(config)


def _robot_motor() -> robot_config.Motor:
    return session.get_robot_motors()[0]


def test_body_model_speed(model: body_model.BodyModel) -> None:
    TARGET_HZ = 15_000

    motor = _robot_motor()
    start_time = time.perf_counter()
    for _ in range(TARGET_HZ):
        model.set_target(3.0, 10.0)
        _ = model.feedforward_torque()
        _ = model.wheel_speed(motor)

    run_time = time.perf_counter() - start_time
    assert run_time < 1.0


def test_estimator_speed(estimator: twist_estimator.TwistEstimator) -> None:
    TARGET_HZ = 10_000

    motor = _robot_motor()
    start_time = time.perf_counter()
    for _ in range(TARGET_HZ):
        estimator.update(motor, 1.0)
        _ = estimator.twist()

    run_time = time.perf_counter() - start_time
    assert run_time < 1.0
