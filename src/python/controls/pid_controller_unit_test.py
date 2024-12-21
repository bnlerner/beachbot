import time
from unittest import mock

import pytest

from controls import pid_controller

# 50 hz
_TIME_STEP = 1 / 50

_P_GAIN = 2.0
_I_GAIN = 100.0
_D_GAIN = 0.05


@pytest.fixture
def controller() -> pid_controller.PIDController:
    return pid_controller.PIDController(_P_GAIN, _I_GAIN, _D_GAIN)


def test_zero_error(controller: pid_controller.PIDController) -> None:
    assert controller.control_signal(0.0) == 0.0
    time.sleep(_TIME_STEP)
    assert controller.control_signal(0.0) == 0.0


def test_error_calc(controller: pid_controller.PIDController) -> None:
    # Prime the controller
    with mock.patch("time.perf_counter", mock.MagicMock(return_value=0.0)):
        _ = controller.control_signal(0.0)

    error = 1.0
    with mock.patch("time.perf_counter", mock.MagicMock(return_value=_TIME_STEP)):
        control_signal = controller.control_signal(error)

    p_part = _P_GAIN * error
    i_part = _I_GAIN * error * _TIME_STEP
    d_part = _D_GAIN * error / _TIME_STEP

    assert control_signal == p_part + i_part + d_part


def test_integral_increases(controller: pid_controller.PIDController) -> None:
    _ = controller.control_signal(0.0)
    time.sleep(_TIME_STEP)
    # Skip because contains derivative error
    _ = controller.control_signal(1.0)
    time.sleep(_TIME_STEP)
    err0 = controller.control_signal(1.0)
    time.sleep(_TIME_STEP)
    err1 = controller.control_signal(1.0)

    assert err1 > err0


def test_reset(controller: pid_controller.PIDController) -> None:
    _ = controller.control_signal(0.0)
    time.sleep(_TIME_STEP)
    assert controller.control_signal(1.0) != 0.0
    controller.reset()
    time.sleep(_TIME_STEP)
    assert controller.control_signal(0.0) == 0.0
