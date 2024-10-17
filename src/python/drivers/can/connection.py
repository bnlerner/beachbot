from typing import Tuple, Callable, Optional, List, Type, Generic, TypeVar
import can
from drivers.can import messages, enums
from typing_helpers import req
import asyncio

ODriveCanMessageT = TypeVar("ODriveCanMessageT", bound="messages.OdriveCanMessage")


class CANSimpleListener(can.Listener):

    def __init__(self, msg_class: ODriveCanMessageT, callback: Callable):
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
        print(f"starting to listen with {self._msg_class=}, {self._callback=}, {self._can_msg_queue=}")
        while self._bus_error is None:
            if msg := await self.wait_for_message(0.01):
                await self._callback(msg)

        self.stop()
        raise self._bus_error

    def stop(self) -> None:
        self._is_stopped = True


class CANSimple:
    """A wrapper on top of of the odrive class which enables easier interaction
    with the motors.
    """

    def __init__(self, can_interface: enums.CANInterface, bustype: enums.BusType) -> None:
        self._can_bus = can.interface.Bus(
            can_interface.value,
            interface=bustype.value
        )
        self._flush_bus()
        self._notifier: Optional[can.Notifier] = None
        self._listeners: List[CANSimpleListener] = []
        self._listen_tasks: List[asyncio.Task] = []

    def register_callbacks(self, *msg_cls_callbacks: Tuple[Type[messages.OdriveCanMessage], Callable]) -> None:
        """Registers callbacks on receiving a message."""
        for msg_cls, callback in msg_cls_callbacks:
            if not asyncio.iscoroutinefunction(callback):
                raise TypeError(
                    "Callbacks registered must be a coroutine function"
                )
            # msg_cls is not a valid type hint because it only exists at runtime.
             # type: ignore[valid-type]
            self._listeners.append(CANSimpleListener(msg_cls, callback))

    async def send(self, msg: messages.OdriveCanMessage) -> None:
        can_msg = msg.as_can_message()
        self._can_bus.send(can_msg)
        await asyncio.sleep(0)

    async def listen(self) -> None:
        loop = asyncio.get_running_loop()
        self._notifier = can.Notifier(self._can_bus, self._listeners, loop=loop)
        self._listen_tasks =[
            asyncio.create_task(listener.listen()) for listener in self._listeners
        ]
        await asyncio.gather(*self._listen_tasks)

    def shutdown(self) -> None:
        if self._notifier is not None:
            self._notifier.stop()
        for task in self._listen_tasks:
            task.cancel()
        self._clean_up_can_bus()

    def _flush_bus(self) -> None:
        # Flush CAN RX buffer so there are no more old pending messages
        while not (self._can_bus.recv(timeout=0) is None):
            pass

    def _clean_up_can_bus(self) -> None:
        try:
            self._can_bus.shutdown()
        except can.CanError:
            pass