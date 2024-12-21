import time

import pytest
from config import robot_config
from ipc import session

from controls import motor_velocity_controller


@pytest.fixture
def controller() -> motor_velocity_controller.MotorVelocityController:
    motors = session.get_robot_motors()
    config = session.get_robot_config()
    return motor_velocity_controller.MotorVelocityController(motors, config)


def _robot_motor() -> robot_config.Motor:
    return session.get_robot_motors()[0]


def test_controller_speed(
    controller: motor_velocity_controller.MotorVelocityController,
) -> None:
    TARGET_HZ = 50_000

    motor = _robot_motor()
    start_time = time.perf_counter()
    for _ in range(TARGET_HZ):
        controller.set_target(3.0, 10.0)
        _ = controller.calc_feedforward_torque()
        _ = controller.calc_wheel_speed(motor)

    run_time = time.perf_counter() - start_time
    assert run_time < 1.0
