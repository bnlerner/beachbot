import asyncio
import functools
from typing import Callable, List, Optional


class BaseNode:
    """A node which defines common functionality used in other nodes"""

    def __init__(self) -> None:
        # Maintains a reference to the task running in the node so its cancellable.
        self._node_task: Optional[asyncio.Task] = None
        self._background_tasks: List[asyncio.Task] = []
        self._task_functions: List[Callable] = []

    def add_tasks(self, *functions: Callable) -> None:
        self._task_functions.extend(functions)

    def start(self) -> None:
        asyncio.run(self._async_run())

    async def _async_run(self) -> None:
        self._node_task = asyncio.create_task(self._main())
        await self._node_task

    async def _main(self) -> None:
        exception: Optional[Exception] = None
        try:
            self._create_tasks()
            await asyncio.gather(*self._background_tasks)
        except Exception as err:
            exception = err
        finally:
            self.shutdown()
            await self.shutdown_hook()
            await self._raise_exceptions(exception)

    def shutdown(self) -> None:
        for task in self._background_tasks:
            task.cancel()

    def _create_tasks(self) -> None:
        for function in self._task_functions:
            if asyncio.iscoroutinefunction(functools._unwrap_partial(function)):  # type: ignore[attr-defined]
                coroutine = function()
            else:
                coroutine = asyncio.to_thread(function)

            self._background_tasks.append(asyncio.create_task(coroutine))

    async def shutdown_hook(self) -> None:
        """Runs prior to the node shutting down. Overridden by implementation of the node."""
        ...

    async def _raise_exceptions(self, exception: Optional[BaseException]) -> None:
        results = await asyncio.gather(*self._background_tasks, return_exceptions=True)
        gather_exceptions = [r for r in results if isinstance(r, BaseException)]
        if exception:
            gather_exceptions.append(exception)

        for exception in gather_exceptions:
            if isinstance(exception, (asyncio.CancelledError, KeyboardInterrupt)):
                pass
            else:
                raise exception
