import asyncio
import functools
import pathlib
import pickle
import time
from multiprocessing.shared_memory import SharedMemory
from typing import Callable, Generic, Optional, TypeVar

import filelock
import log
from typing_helpers import req

from ipc import core

_LOCK_DIR = "/home/brianlerner/beachbot/var/locks/"
_LOCK_SUFFIX = "-lock"

# Minumum interval in seconds between when checking for a new message.
_POLL_INTERVAL = 0.001

BaseMessageT = TypeVar("BaseMessageT", bound="core.BaseMessage")


class Publisher(Generic[BaseMessageT]):
    """A generic publisher for sending messages from a specific channel."""

    def __init__(self, node_id: core.NodeID, channel: core.ChannelSpec):
        self._node_id = node_id
        self._channel = channel
        self._shm = _init_shm(channel)
        # Should always be non-None because we just init the SHM file.
        self._shm_id = req(_get_shm_id(self._shm))
        self._lock = _gen_lock(self._shm.name)

    def publish(self, msg: BaseMessageT) -> None:
        """Sends a message to the channel.

        NOTE: Messages are not queued and only the latest message can be
        read by the subscriber.
        """
        with self._lock:
            if (cur_shm_id := _get_shm_id(self._shm)) is None:
                log.error(
                    f"{self._node_id} is attempting to publish to SHM {self._shm.name} which is closed."
                )
                return

            _raise_if_shm_id_changed(self._shm_id, cur_shm_id)
            self._send_message(msg)

    def close(self) -> None:
        """Stops the publisher, closing any open data links permanently."""
        with self._lock:
            _close(self._shm)

    def _send_message(self, msg: BaseMessageT) -> None:
        msg.origin = self._node_id
        msg.creation = time.perf_counter()
        serialized_msg = pickle.dumps(msg)
        self._raise_if_message_too_large(serialized_msg)
        self._write_to_shm(serialized_msg)

    def _write_to_shm(self, data: bytes) -> None:
        """Writes to SHM with a buffer if the message is not the exact size of the
        registry.
        """
        buffer = b"\x00" * (self._channel.msg_size - len(data))
        data_w_buffer = data + buffer
        self._shm.buf[: self._channel.msg_size] = data_w_buffer

    def _raise_if_message_too_large(self, msg: bytes) -> None:
        if len(msg) > self._channel.msg_size:
            raise ValueError(
                "Message is larger than the channel size. "
                f"Message size: {len(msg)}, "
                f"Channel size: {self._channel.msg_size}"
            )


class Subscriber(Generic[BaseMessageT]):
    """A generic subscriber for receiving messages sent to a specific channel. Once
    received, the message is passed to a callback function for processing.
    """

    def __init__(
        self,
        node_id: core.NodeID,
        channel: core.ChannelSpec[BaseMessageT],
        callback: Optional[Callable],
    ):
        self._node_id = node_id
        self._shm = _init_shm(channel)
        self._callback = callback
        # Should always be non-None because we just init the SHM file.
        self._shm_id = req(_get_shm_id(self._shm))
        self._lock = _gen_lock(self._shm.name)

        # Prime last message so we only return new messages
        self._last_msg: Optional[BaseMessageT] = self._get_msg()
        self._running = True

    async def wait_for_message(self) -> Optional[BaseMessageT]:
        """Waits for a message to be returned. Returns None in cases where the
        subscriber stops running or no further messages are expected due to the channel
        closing.
        """
        while self._running:
            if (cur_shm_id := _get_shm_id(self._shm)) is None:
                # If the current shm switches to None then we know that some other
                # process has closed and unlinked the shm file so we choose to just exit
                # the listen function vs raise.
                self._running = False
                log.info("Shutting down listen as the SHM file is unlinked.")
                return None

            _raise_if_shm_id_changed(self._shm_id, cur_shm_id)
            if (msg := self._get_msg()) is None or msg == self._last_msg:
                # Small sleep to let other concurrent processes run and not block.
                await asyncio.sleep(_POLL_INTERVAL)
            else:
                self._last_msg = msg
                return msg

        return None

    async def listen(self) -> None:
        """Listens indefinitely for the latest unique message and processes it via the
        configured callback.
        """
        if self._callback is None:
            raise ValueError("Callback must be specified to listen for messages.")

        while msg := await self.wait_for_message():
            if _is_coroutine(self._callback):
                await self._callback(msg)
            else:
                self._callback(msg)
                # Gives up the thread temporarily for regular callbacks to let other
                # concurrent processes run.
                await asyncio.sleep(0.0)

    def close(self) -> None:
        """Stops the subscriber for running, permanently closing any open links to
        the channel.
        """
        self._running = False
        with self._lock:
            _close(self._shm)

    def _get_msg(self) -> Optional[BaseMessageT]:
        try:
            # Copy the data to help avoid race conditions. Force an unpickling
            # error if the buffer is none.
            with self._lock:
                data = bytes(self._shm.buf or b"\x00")

            msg = pickle.loads(data)

        # Pickling errors can happen if the shm has no data. Set message to None.
        except pickle.UnpicklingError:
            msg = None

        return msg


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


@functools.lru_cache
def _shm_path(shm: SharedMemory) -> pathlib.Path:
    return pathlib.Path("/dev/shm") / shm.name


def _init_shm(channel: core.ChannelSpec) -> SharedMemory:
    name = channel.name()
    try:
        shm = SharedMemory(name, create=True, size=channel.msg_size)
    except FileExistsError:
        shm = SharedMemory(name)

    return shm


def _gen_lock(shm_name: str) -> filelock._unix.UnixFileLock:
    """Generates a thread agnostic filelock for the given shm name used to avoid race
    conditions.
    """
    lock_path = _LOCK_DIR + shm_name + _LOCK_SUFFIX
    return filelock._unix.UnixFileLock(lock_path, thread_local=False)


@functools.lru_cache
def _is_coroutine(callback: Callable) -> bool:
    return asyncio.iscoroutinefunction(functools._unwrap_partial(callback))  # type: ignore[attr-defined]


def _raise_if_shm_id_changed(shm_id_1: int, shm_id_2: int) -> None:
    if shm_id_1 != shm_id_2:
        raise ValueError(f"SHM ID has changed, file closed. {shm_id_1=}, {shm_id_2=}")
