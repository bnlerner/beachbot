import pathlib
import sys
from datetime import datetime, timezone
from typing import BinaryIO

_LOG_DIRECTORY = pathlib.Path("/var/log/beachbot")


class Logger:
    """A simple class for logging."""

    def __init__(self) -> None:
        self._log_file = self._open_log_file()

    def _open_log_file(self) -> BinaryIO:
        filepath = pathlib.Path(sys.argv[0])
        # If sys.argv returns an empty string e.g. when in a python3 shell, we assign a
        # generic filename.
        filename = filepath.stem if filepath.stem else "unknown"
        file_path = (_LOG_DIRECTORY / filename).with_suffix(".log")

        return open(file_path, "ab", buffering=0)

    def debug(self, msg: str) -> None:
        self._write_to_log("DEBUG", msg)

    def info(self, msg: str) -> None:
        self._write_to_log("INFO", msg)

    def error(self, msg: str) -> None:
        self._write_to_log("ERROR", msg)

    def warning(self, msg: str) -> None:
        self._write_to_log("WARNING", msg)

    def _write_to_log(self, level: str, msg: str) -> None:
        log_str = f"{_iso_time()} {level.upper()}: {msg}\n"
        self._log_file.write(log_str.encode("utf-8"))


def _iso_time() -> str:
    """Current timestamp as a string."""
    return datetime.now(tz=timezone.utc).isoformat()
