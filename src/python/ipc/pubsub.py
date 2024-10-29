import asyncio
import functools
import pathlib
import pickle
import time
from multiprocessing.shared_memory import SharedMemory
from typing import Callable, Generic, Optional, TypeVar

from ipc import core

# In general no message size should exceed this number.
_MSG_SIZE = 1000
_POLL_INTERVAL = 0.001

BaseMessageT = TypeVar("BaseMessageT", bound="core.BaseMessage")


class Publisher(Generic[BaseMessageT]):
    """A generic publisher for sending messages from a specific channel."""

    def __init__(self, node_id: core.NodeID, channel: core.ChannelSpec):
        self._node_id = node_id
        self._shm = _init_shm(channel)
        self._shm_id = _get_shm_id(self._shm)

    def publish(self, msg: BaseMessageT) -> None:
        """Sends a message to the channel.

        NOTE: Messages are not queued and only the latest message can be
        read by the subscriber.
        """
        _raise_if_shm_id_changed(self._shm_id, _get_shm_id(self._shm))
        msg.origin = self._node_id
        msg.creation = time.perf_counter()
        serialized_msg = pickle.dumps(msg)
        self._write_to_shm(serialized_msg)

    def close(self) -> None:
        """Stops the publiser, closing any open data links permanently."""
        _close(self._shm)

    def _write_to_shm(self, data: bytes) -> None:
        buffer = b"\x00" * (_MSG_SIZE - len(data))
        data_w_buffer = data + buffer
        self._shm.buf[:_MSG_SIZE] = data_w_buffer


class Subscriber(Generic[BaseMessageT]):
    """A generic subscriber for receiving messages sent to a specific channel. Once received,
    the message is passed to a callback function for processing.
    """

    def __init__(
        self,
        node_id: core.NodeID,
        channel: core.ChannelSpec[BaseMessageT],
        callback: Callable,
    ):
        self._node_id = node_id
        self._shm = _init_shm(channel)
        self._callback = callback
        self._shm_id = _get_shm_id(self._shm)

        self._last_msg: Optional[BaseMessageT] = None
        self._running = True

    async def listen(self) -> None:
        """Listens indefinitely for the latest unique message and processes it via a
        callback.
        """
        while self._running:
            _raise_if_shm_id_changed(self._shm_id, _get_shm_id(self._shm))
            # Copy the data to help avoid race conditions
            data = bytes(self._shm.buf)
            msg = pickle.loads(data)
            if msg == self._last_msg:
                await asyncio.sleep(_POLL_INTERVAL)
            else:
                self._last_msg = msg
                if _is_coroutine(self._callback):
                    await self._callback(msg)
                else:
                    self._callback(msg)
                    # Gives up the thread temporarily to let other processes run
                    await asyncio.sleep(0.0)

    def close(self) -> None:
        """Stops the subscriber for running, permanently closing any open links to
        the channel.
        """
        self._running = False
        _close(self._shm)


def _close(shm: SharedMemory) -> None:
    """Closes the SHM by first calling close then unlink to permanently
    release the memory.
    """
    try:
        shm.close()
    except BufferError:
        pass

    try:
        shm.unlink()
    except FileNotFoundError:
        pass


def _get_shm_id(shm: SharedMemory) -> Optional[int]:
    """The SHM id as represented by a unique identifier of the file path."""
    try:
        return _shm_path(shm).stat().st_ino
    except FileNotFoundError:
        return None


def _shm_path(shm: SharedMemory) -> pathlib.Path:
    return pathlib.Path("/dev/shm") / shm.name


def _init_shm(channel: core.ChannelSpec) -> SharedMemory:
    name = channel.name()
    try:
        shm = SharedMemory(name, create=True, size=_MSG_SIZE)
    except FileExistsError:
        shm = SharedMemory(name)

    return shm


def _is_coroutine(callback: Callable) -> bool:
    return asyncio.iscoroutinefunction(functools._unwrap_partial(callback))  # type: ignore[attr-defined]


def _raise_if_shm_id_changed(shm_id_1: Optional[int], shm_id_2: Optional[int]) -> None:
    if shm_id_1 != shm_id_2:
        raise ValueError(f"SHM ID has changed, file closed. {shm_id_1=}, {shm_id_2=}")
