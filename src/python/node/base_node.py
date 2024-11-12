import asyncio
import functools
import signal
from types import FrameType
from typing import Callable, Dict, List, Optional

import log
from ipc import core, pubsub


class BaseNode:
    """A node which defines common functionality used in other nodes"""

    def __init__(self, node_id: core.NodeID) -> None:
        self._node_id = node_id
        # Maintains a reference to the task running in the node so its cancellable.
        self._node_task: Optional[asyncio.Task] = None
        self._background_tasks: List[asyncio.Task] = []
        self._task_functions: List[Callable] = []

        # IPC Pub sub
        self._subscriber_callbacks: Dict[core.ChannelSpec, Callable] = {}
        self._subscribers: List[pubsub.Subscriber] = []
        self._publishers: Dict[core.ChannelSpec, pubsub.Publisher] = {}
        self._add_cleanup_signals()

    def start(self) -> None:
        """Starts the node and spins until its finished."""
        log.info(f"Starting node: {self._node_id}")
        try:
            asyncio.run(self._async_run())
        except BaseException as err:
            self._raise_exception(err)
        finally:
            log.info(f"Finished shutting down node: {self._node_id}")

    def add_tasks(self, *functions: Callable) -> None:
        self._task_functions.extend(functions)

    def add_subscribers(self, channels: Dict[core.ChannelSpec, Callable]) -> None:
        self._subscriber_callbacks.update(channels)

    def add_publishers(self, *channels: core.ChannelSpec) -> None:
        for channel in channels:
            self._publishers[channel] = pubsub.Publisher(self._node_id, channel)

    def publish(self, channel: core.ChannelSpec, msg: core.BaseMessage) -> None:
        if channel not in self._publishers:
            raise RuntimeError(
                f"Unable to publish message without setting up the {channel=} first."
            )

        self._publishers[channel].publish(msg)

    async def _async_run(self) -> None:
        self._node_task = asyncio.create_task(self._main())
        await self._node_task

    async def _main(self) -> None:
        exception: Optional[Exception] = None
        try:
            self._add_subscriber_functions()
            self._create_tasks()
            await asyncio.gather(*self._background_tasks)
        except Exception as err:
            exception = err
        finally:
            self.shutdown()
            await self.shutdown_hook()
            await self._gather_exceptions(exception)

    def shutdown(self) -> None:
        log.info(f"Stopping node: {self._node_id}")
        for task in self._background_tasks:
            task.cancel()

        for sub in self._subscribers:
            sub.close()

        for pub in self._publishers.values():
            pub.close()

    def _add_subscriber_functions(self) -> None:
        for channel, callback in self._subscriber_callbacks.items():
            sub = pubsub.Subscriber(self._node_id, channel, callback)
            self._subscribers.append(sub)
            self._task_functions.append(sub.listen)

    def _create_tasks(self) -> None:
        for function in self._task_functions:
            if asyncio.iscoroutinefunction(functools._unwrap_partial(function)):  # type: ignore[attr-defined]
                coroutine = function()
            else:
                log.info(f"Called to thread {function=}")
                coroutine = asyncio.to_thread(function)

            self._background_tasks.append(asyncio.create_task(coroutine))

    def _add_cleanup_signals(self) -> None:
        catchable_signals = set(signal.Signals) - {
            signal.SIGKILL,
            signal.SIGSTOP,
            signal.SIGCHLD,
        }
        for signal_ in catchable_signals:
            signal.signal(signal_, self._rcv_signal)

    def _rcv_signal(self, signal_: Optional[int], frame: Optional[FrameType]) -> None:
        log.info(f"Received shutdown signal {signal_=}")
        self.shutdown()

    async def shutdown_hook(self) -> None:
        """Runs prior to the node shutting down. Overridden by implementation of the node."""
        ...

    async def _gather_exceptions(self, exception: Optional[BaseException]) -> None:
        """Gathers and raises any relevant exceptions."""
        results = await asyncio.gather(*self._background_tasks, return_exceptions=True)
        gather_exceptions = [r for r in results if isinstance(r, BaseException)]
        if exception:
            gather_exceptions.append(exception)

        for exception in gather_exceptions:
            self._raise_exception(exception)

    def _raise_exception(self, exception: BaseException) -> None:
        """Raises the exception unless its a common, not important one."""
        if not isinstance(exception, (asyncio.CancelledError, KeyboardInterrupt)):
            raise exception
