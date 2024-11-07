from multiprocessing import resource_tracker
from typing import Sized


def _remove_shm_from_resource_tracker() -> None:
    """Monkey-patch multiprocessing.resource_tracker so SharedMemory won't be tracked.
    This function is taken from mprt_monkeypatch.py here:
    https://bugs.python.org/issue38119
    """

    def fix_register(name: Sized, rtype: str) -> None:
        if rtype == "shared_memory":
            return
        return resource_tracker._resource_tracker.register(self, name, rtype)  # type: ignore[arg-type, call-arg, name-defined] # noqa

    resource_tracker.register = fix_register

    def fix_unregister(name: Sized, rtype: str) -> None:
        if rtype == "shared_memory":
            return
        return resource_tracker._resource_tracker.unregister(self, name, rtype)  # type: ignore[arg-type, call-arg, name-defined] # noqa

    resource_tracker.unregister = fix_unregister

    if "shared_memory" in resource_tracker._CLEANUP_FUNCS:  # type: ignore[attr-defined]
        del resource_tracker._CLEANUP_FUNCS["shared_memory"]  # type: ignore[attr-defined]


_remove_shm_from_resource_tracker()
