import pytest
from ipc import session

from controls import motor_velocity_controller


@pytest.fixture
def controller() -> motor_velocity_controller.MotorVelocityController:
    return motor_velocity_controller.MotorVelocityController(
        session.get_robot_motors(), session.get_robot_config()
    )


def test_controller(
    controller: motor_velocity_controller.MotorVelocityController,
) -> None:
    motor = session.get_robot_motors()[0]
    controller.set_target(0.0, 0.0)
    assert controller.calc_wheel_speed(motor) == 0.0
    assert controller.calc_feedforward_torque() == 0.0

    controller.set_target(0.0, 10.0)
    init_wheel_speed = controller.calc_wheel_speed(motor)
    init_ff_torque = controller.calc_feedforward_torque()

    assert init_wheel_speed != 0.0
    assert init_ff_torque != 0.0
    # Ramps the speed
    assert controller.calc_wheel_speed(motor) > init_wheel_speed
    assert controller.calc_feedforward_torque() > init_ff_torque

    for _ in range(100):
        _ = controller.calc_wheel_speed(motor)
        _ = controller.calc_feedforward_torque()

    controller.set_target(0.0, 0.0)
    for _ in range(105):
        controller.calc_wheel_speed(motor)
        controller.calc_feedforward_torque()

    assert controller.calc_wheel_speed(motor) == 0.0
    assert controller.calc_feedforward_torque() == 0.0
