import asyncio
import functools
import ipaddress
import logging
import signal
from types import FrameType
from typing import Callable, Dict, List, Optional, Sequence, Union

import log
import looped_function
import uvicorn
from ipc import core, pubsub, request
from starlette import applications, middleware, routing
from starlette.middleware import cors

RouteType = Union[routing.Route, routing.BaseRoute]

_MIDDLEWARE = [
    middleware.Middleware(
        cors.CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["*"],
        allow_headers=["*"],
        allow_credentials=True,
    )
]
_LOCAL_HOST = ipaddress.IPv4Address("0.0.0.0")
_CERT_PATH = "env/auth/cert.pem"
_KEY_PATH = "env/auth/key.pem"


class _NoSignalInterruptServer(uvicorn.Server):
    """Custom child of the uvicorn server that doesn't interfere with the signal
    handling in BaseNode. https://github.com/encode/uvicorn/issues/1579
    """

    def install_signal_handlers(self) -> None:
        pass


class _CancelledErrorFilter(logging.Filter):
    """Skips logging asyncio CancelledErrors."""

    def filter(self, record: logging.LogRecord) -> bool:
        return "asyncio.exceptions.CancelledError" not in record.getMessage()


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
        self._request_clients: Dict[core.RequestSpec, request.RequestClient] = {}
        self._request_server: Optional[request.RequestServer] = None
        self._http_server: Optional[uvicorn.Server] = None
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

    def add_looped_tasks(self, looped_funcs: Dict[Callable, float]) -> None:
        for func, loop_rate in looped_funcs.items():
            partial_func = functools.partial(
                looped_function.loop_function, func, loop_rate
            )
            functools.update_wrapper(partial_func, looped_function.loop_function)
            self.add_tasks(partial_func)

    def add_subscribers(self, channels: Dict[core.ChannelSpec, Callable]) -> None:
        self._subscriber_callbacks.update(channels)

    def add_publishers(self, *channels: core.ChannelSpec) -> None:
        for channel in channels:
            self._publishers[channel] = pubsub.Publisher(self._node_id, channel)

    def add_request_clients(self, *request_specs: core.RequestSpec) -> None:
        for request_spec in request_specs:
            self._request_clients[request_spec] = request.RequestClient(
                self._node_id, request_spec
            )

    def set_request_server(
        self, request_spec: core.RequestSpec, request_func: Callable
    ) -> None:
        """Sets the request this server will respond to."""
        self._request_server = request.RequestServer(
            self._node_id, request_spec, request_func
        )

    def set_http_server(self, port: int, routes: Sequence[RouteType]) -> None:
        server_config = uvicorn.Config(
            applications.Starlette(debug=True, routes=routes, middleware=_MIDDLEWARE),
            host=str(_LOCAL_HOST),
            port=port,
            ssl_keyfile=_KEY_PATH,
            ssl_certfile=_CERT_PATH,
            use_colors=True,
            log_level="error",
        )
        logging.getLogger("uvicorn.error").addFilter(_CancelledErrorFilter())
        self._http_server = _NoSignalInterruptServer(server_config)

    def publish(self, channel: core.ChannelSpec, msg: core.BaseMessage) -> None:
        if channel not in self._publishers:
            raise RuntimeError(
                f"Unable to publish message without setting up the {channel=} first."
            )

        self._publishers[channel].publish(msg)

    async def send_request(
        self, spec: core.RequestSpec, request_msg: core.Request
    ) -> core.RequestResponse:
        if spec not in self._request_clients:
            raise RuntimeError(
                "Unrecognized Request spec, did you forget to add it? "
                f"{spec.base_channel.upper()}"
            )

        return await self._request_clients[spec].send(request_msg)

    async def _async_run(self) -> None:
        self._node_task = asyncio.create_task(self._main())
        await self._node_task

    async def _main(self) -> None:
        exception: Optional[Exception] = None
        try:
            self._add_pubsub_functions()
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

        for client in self._request_clients.values():
            client.close()

        if self._request_server:
            self._request_server.close()

    def _add_pubsub_functions(self) -> None:
        for channel, callback in self._subscriber_callbacks.items():
            sub = pubsub.Subscriber(self._node_id, channel, callback)
            self._subscribers.append(sub)
            self._task_functions.append(sub.listen)

        if self._request_server:
            self._task_functions.append(self._request_server.start)

        if self._http_server is not None:
            self._task_functions.append(self._http_server.serve)

    def _create_tasks(self) -> None:
        for function in self._task_functions:
            if asyncio.iscoroutinefunction(functools._unwrap_partial(function)):  # type: ignore[attr-defined]
                coroutine = function()
            else:
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
