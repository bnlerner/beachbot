import time
from typing import Optional


class PIDController:
    """A PID Controller. Handles controlling a simple system via PID control gains. Time
    interval is set by how often control signal is calculated and handled internally.
    """

    def __init__(self, p_gain: float = 0, i_gain: float = 0, d_gain: float = 0):
        self._p_gain = p_gain
        self._i_gain = i_gain
        self._d_gain = d_gain

        self._integrated_error = 0.0
        self._previous_error = 0.0
        self._previous_control_signal_ts: Optional[float] = None

    def control_signal(self, error: float) -> float:
        if self._previous_control_signal_ts is not None:
            time_step = time.perf_counter() - self._previous_control_signal_ts
            self._integrated_error += time_step * error
            error_rate_of_change = (error - self._previous_error) / time_step
            self._previous_control_signal_ts += time_step
        else:
            error_rate_of_change = 0.0
            self._previous_control_signal_ts = time.perf_counter()

        self._previous_error = error

        return (
            self._calc_p_control(error)
            + self._calc_i_control()
            + self._calc_d_control(error_rate_of_change)
        )

    def reset(self) -> None:
        self._integrated_error = 0.0
        self._previous_error = 0.0
        self._previous_control_signal_ts = None

    def _calc_p_control(self, error: float) -> float:
        return self._p_gain * error

    def _calc_i_control(self) -> float:
        return self._i_gain * self._integrated_error

    def _calc_d_control(self, error_rate_of_change: float) -> float:
        return self._d_gain * error_rate_of_change
