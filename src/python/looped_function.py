import asyncio
import functools
import time
from typing import Any, Callable, Optional

import geometry


def _calc_overshoot(
    last_finish_time: float, period: float, last_overshoot: float
) -> float:
    additional_overshoot = time.perf_counter() - last_finish_time
    # Limit adjustment for consistency.
    abs_adjustment = min(abs(additional_overshoot) * 0.3, period / 4)
    adjustment = geometry.sign(additional_overshoot) * abs_adjustment
    return last_overshoot + adjustment


async def _run_func(func: Callable, is_coro: bool) -> Any:
    return await func() if is_coro else func()


async def loop_function(func: Callable, frequency: float) -> Any:
    """Loops the function at the target frequency."""
    overshoot = 0.0
    period = 1 / frequency
    finish_time: Optional[float] = None
    is_coro = asyncio.iscoroutinefunction(functools._unwrap_partial(func))  # type: ignore[attr-defined]

    while True:
        if finish_time is not None:
            overshoot = _calc_overshoot(finish_time, period, overshoot)

        finish_time = time.perf_counter() + period
        if (output := await _run_func(func, is_coro)) is not None:
            return output

        remaining_time = finish_time - time.perf_counter()
        # Sleep time is based off the current remaining time minus our estimated
        # overshoot from other processes running. We always sleep a small minimum time
        # to allow the scheduler to release this looped function.
        sleep_time = max(remaining_time - overshoot, 1e-5)
        await asyncio.sleep(sleep_time)
