import asyncio
from typing import Callable, Generic, List, Optional, Tuple, Type, TypeVar

import can

from drivers.can import enums, messages

ODriveCanMessageT = TypeVar("ODriveCanMessageT", bound="messages.OdriveCanMessage")
_ODRIVE_BAUDRATE = 250_000


class CANSimpleListener(can.Listener, Generic[ODriveCanMessageT]):
    """Listens to CAN messages and provides ways to add callbacks or get the message
    if desired.
    """

    def __init__(
        self, msg_class: Type[ODriveCanMessageT], *, callback: Optional[Callable] = None
    ):
        self._msg_class = msg_class
        self._callback = callback
        self._bus_error: Optional[Exception] = None

        self._is_stopped = False
        self._can_msg_queue: asyncio.Queue = asyncio.Queue()

    def on_message_received(self, msg: can.Message) -> None:
        if not self._is_stopped and self._msg_class.matches(msg):
            self._can_msg_queue.put_nowait(msg)

    def on_error(self, exc: Exception) -> None:
        self._bus_error = exc

    async def get_message(self) -> ODriveCanMessageT:
        can_msg = await self._can_msg_queue.get()
        return self._msg_class.from_can_message(can_msg)

    async def wait_for_message(self, duration: float) -> Optional[ODriveCanMessageT]:
        try:
            can_msg = await asyncio.wait_for(self._can_msg_queue.get(), duration)
            return self._msg_class.from_can_message(can_msg)
        except TimeoutError:
            return None

    async def listen(self) -> None:
        while self._bus_error is None:
            if msg := await self.wait_for_message(0.01):
                if self._callback:
                    await self._callback(msg)

        self.stop()
        raise self._bus_error

    def stop(self) -> None:
        self._is_stopped = True


class CANSimple:
    """A wrapper on top of of the odrive class which enables easier interaction
    with the motors.
    """

    def __init__(
        self, can_interface: enums.CANInterface, bustype: enums.BusType
    ) -> None:
        self._can_bus = can.interface.Bus(
            can_interface.value, interface=bustype.value, bitrate=_ODRIVE_BAUDRATE
        )
        self._flush_bus()
        self._notifier: Optional[can.Notifier] = None
        self._listeners: List[CANSimpleListener] = []
        self._listen_tasks: List[asyncio.Task] = []
        self._reader = can.AsyncBufferedReader()

    def register_callbacks(
        self, *msg_cls_callbacks: Tuple[Type[messages.OdriveCanMessage], Callable]
    ) -> None:
        """Registers callbacks on receiving a message."""
        for msg_cls, callback in msg_cls_callbacks:
            if not asyncio.iscoroutinefunction(callback):
                raise TypeError("Callbacks registered must be a coroutine function")

            self._listeners.append(CANSimpleListener(msg_cls, callback=callback))

    async def send(self, msg: messages.OdriveCanMessage) -> None:
        """Sends a CAN message."""
        can_msg = msg.as_can_message()
        self._can_bus.send(can_msg)
        await asyncio.sleep(0)

    async def listen(self) -> None:
        """Listens to incoming messages via registered callbacks and spins."""
        loop = asyncio.get_running_loop()
        self._notifier = can.Notifier(self._can_bus, self._listeners, loop=loop)
        self._listen_tasks = [
            asyncio.create_task(listener.listen()) for listener in self._listeners
        ]
        await asyncio.gather(*self._listen_tasks)

    async def await_parameter_response(
        self, node_id: int, value_type: messages.VALUE_TYPES, timeout: float = 1.0
    ) -> Optional[messages.ParameterResponse]:
        """Waits until a specific message type is received on the can bus."""
        self._flush_reader()
        self._add_listener_to_notifier(self._reader)
        # Set the class instance var to the value type. Not the cleanest but works
        # in this message architecture.
        messages.ParameterResponse.value_type = value_type
        arb_id = messages.ParameterResponse(node_id).arbitration_id

        return await asyncio.wait_for(
            self._poll_for_parameter_response(arb_id.value), timeout
        )

    async def _poll_for_parameter_response(
        self, arbitration_id: int
    ) -> Optional[messages.ParameterResponse]:
        """Continuously polls the can bus for a specific arbitration id until a message
        with the same arbitration id is found. In this case, the Odrive sends back
        a parameter response to a get or set parameter notification.
        """
        async for can_msg in self._reader:
            if can_msg.arbitration_id == arbitration_id:
                return messages.ParameterResponse.from_can_message(can_msg)

        return None

    def shutdown(self) -> None:
        if self._notifier is not None:
            self._notifier.stop()
        for task in self._listen_tasks:
            task.cancel()
        self._clean_up_can_bus()

    def _add_listener_to_notifier(self, listener: can.Listener) -> None:
        """Adds a listener to the notifier, creating the notifier if its not already
        instanstatiated and checking to ensure the listener is added twice.
        """
        if self._notifier is None:
            self._notifier = can.Notifier(
                self._can_bus, [listener], loop=asyncio.get_running_loop()
            )
        elif self._reader not in self._notifier.listeners:
            self._notifier.add_listener(listener)

    def _flush_bus(self) -> None:
        # Flush CAN RX buffer so there are no more old pending messages
        while self._can_bus.recv(timeout=0) is not None:
            pass

    def _flush_reader(self) -> None:
        while not self._reader.buffer.empty():
            self._reader.buffer.get_nowait()

    def _clean_up_can_bus(self) -> None:
        try:
            self._can_bus.shutdown()
        except can.CanError:
            pass
