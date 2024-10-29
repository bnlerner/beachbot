import asyncio
import functools
import pathlib
import pickle
from multiprocessing.shared_memory import SharedMemory
from typing import Callable, Generic, Optional

from ipc import core, messages

# In general no message size should exceed this number.
_MSG_SIZE = 500
_POLL_INTERVAL = 0.001


class Publisher(Generic[messages.BaseMessageT]):
    def __init__(self, channel: core.ChannelSpec):
        self._shm = _init_shm(channel)
        self._shm_id = _get_shm_id(self._shm)

    def publish(self, msg: messages.BaseMessageT) -> None:
        self._raise_if_shm_id_changed()

        serialized_msg = pickle.dumps(msg)
        buffer = b"\x00" * (_MSG_SIZE - len(serialized_msg))
        msg_w_buffer = serialized_msg + buffer
        self._shm.buf[:_MSG_SIZE] = msg_w_buffer

    def close(self) -> None:
        _close(self._shm)

    def _raise_if_shm_id_changed(self) -> None:
        cur_shm_id = _get_shm_id(self._shm)
        if cur_shm_id != self._shm_id:
            raise ValueError(f"SHM ID has changed, file closed. {cur_shm_id=}, {self._shm_id=}")


class Subscriber(Generic[messages.BaseMessageT]):
    def __init__(self, channel: core.ChannelSpec, callback: Callable):
        self._shm = _init_shm(channel)
        self._callback = callback
        self._shm_id = _get_shm_id(self._shm)

        self._msg_class = channel.msg_class
        self._last_msg: Optional[messages.BaseMessageT] = None
        self._running = True

    async def listen(self) -> None:
        while self._running:
            # Copy the data to help avoid race conditions
            data = bytes(self._shm.buf)
            msg = pickle.loads(data)
            if msg == self._last_msg:
                await asyncio.sleep(_POLL_INTERVAL)
                continue
            else:
                if self._callback_is_coroutine():
                    await self._callback(msg)
                else:
                    self._callback(msg)

            self._last_msg = msg

    def _callback_is_coroutine(self) -> bool:
        return asyncio.iscoroutinefunction(functools._unwrap_partial(self._callback))  # type: ignore[attr-defined]

    def close(self) -> None:
        self._running = False
        _close(self._shm)


def _close(shm: SharedMemory) -> None:
    try:
        shm.close()
    except BufferError:
        pass

    try:
        shm.unlink()
    except FileNotFoundError:
        pass


def _get_shm_id(shm: SharedMemory) -> Optional[int]:
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
