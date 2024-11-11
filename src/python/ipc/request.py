import asyncio
from typing import Callable, Dict, Optional

import log
from typing_helpers import req

from ipc import core, pubsub


class _RequestExecutable:
    """Wraps the request execution providing some functionality when executing and
    cancelling requests.
    """

    def __init__(self, msg: core.Request, func: Callable):
        self._request = msg
        self._func = func
        self._request_task: Optional[asyncio.Task] = None

    @property
    def request(self) -> core.Request:
        return self._request

    async def execute(self) -> core.RequestResponse:
        self._request_task = asyncio.create_task(self._func(self._request))
        ret = None
        cancelled = False

        try:
            ret = await self._request_task
        except asyncio.CancelledError:
            cancelled = True
        except BaseException as err:
            log.error("Error while executing Request: \n" + err.__str__())

        self._request_task = None
        return core.RequestResponse(
            request=self._request, result=ret, cancelled=cancelled
        )

    def cancel(self) -> None:
        if self._request_task:
            self._request_task.cancel()


class RequestServer:
    """A server that accepts requests and executes a function taking the incoming
    request message as input.
    """

    def __init__(
        self,
        node_id: core.NodeID,
        request_spec: core.RequestSpec,
        request_function: Callable,
    ):
        if not asyncio.iscoroutinefunction(request_function):
            raise ValueError("Expected coroutine function")

        self._node_id = node_id
        self._request_spec = request_spec
        self._request_function = request_function
        self._request_subscriber = pubsub.Subscriber[core.Request](
            self._node_id, self._request_spec.request_channel, self._process_request
        )
        self._cancel_subscriber = pubsub.Subscriber[core.RequestCancel](
            self._node_id, self._request_spec.cancel_channel, self._cancel_request
        )
        self._response_publishers: Dict[core.NodeID, pubsub.Publisher] = {}

        self._cur_request: Optional[_RequestExecutable] = None

    async def start(self) -> None:
        await asyncio.gather(
            self._request_subscriber.listen(), self._cancel_subscriber.listen()
        )

    async def _process_request(self, msg: core.Request) -> None:
        self._cur_request = _RequestExecutable(msg, self._request_function)
        response = await self._cur_request.execute()

        self._response_publishers[req(response.request.origin)].publish(response)

    def _cancel_request(self, msg: core.RequestCancel) -> None:
        if self._cur_request and self._cur_request.request == msg.request:
            self._cur_request.cancel()

    def close(self) -> None:
        self._request_subscriber.close()
        self._cancel_subscriber.close()


class RequestClient:
    """Client to send a request to a server. Requests require a response from the server
    to the client.
    """

    def __init__(self, node_id: core.NodeID, request_spec: core.RequestSpec):
        self._node_id = node_id
        self._request_spec = request_spec
        self._request_publisher = pubsub.Publisher[core.Request](
            self._node_id, self._request_spec.request_channel
        )
        self._cancel_publisher = pubsub.Publisher[core.RequestCancel](
            self._node_id, self._request_spec.cancel_channel
        )
        self._response_subscriber = pubsub.Subscriber[core.RequestResponse](
            self._node_id, self._request_spec.response_channel(self._node_id), None
        )

    async def send(self, msg: core.Request) -> core.RequestResponse:
        log.info(
            f"Sending request from {self._node_id=} at Channel: {self._request_spec.base_channel}"
        )
        self._request_publisher.publish(msg)

        while True:
            try:
                if response := await self._response_subscriber.wait_for_message():
                    return response
            except asyncio.CancelledError:
                self._cancel_publisher.publish(core.RequestCancel(request=msg))

    def close(self) -> None:
        self._response_subscriber.close()
