from __future__ import annotations

import pathlib
import signal
import subprocess
import time
from types import FrameType
from typing import Dict, List, Optional

import pydantic
import system_info

_NODE_FILE_PATH = system_info.get_root_project_directory() / "src/python/node/"
_CMD = "python3"
_OPTIONS = "-u"


class NodeConfig(pydantic.BaseModel):
    file_name: str
    args: str = ""
    env_vars: Dict[str, str] = {}

    @property
    def file_path(self) -> pathlib.Path:
        return _NODE_FILE_PATH / self.file_name

    @classmethod
    def rc_node(cls) -> NodeConfig:
        return NodeConfig(file_name="rc_node.py", env_vars={"DISPLAY": ":1"})

    @classmethod
    def motor_control_node(cls) -> NodeConfig:
        return NodeConfig(file_name="motor_control_node.py")

    @classmethod
    def ublox_data_node(cls) -> NodeConfig:
        return NodeConfig(file_name="ublox_data_node.py")


class Orchestrator:
    def __init__(self, profile: str):
        self._profile = _gen_profile(profile)
        # List to keep track of child processes
        self._processes: List[subprocess.Popen] = []
        self._add_cleanup_signals()
        self._running: bool

    async def run(self) -> None:
        """Start a new node process."""
        self._running = True
        self._create_sub_processes()
        await self._spin_and_poll_subprocesses()

    def _create_sub_processes(self) -> None:
        for node in self._profile:
            process = subprocess.Popen(
                [_CMD, _OPTIONS, node.file_path, node.args],  # Command and arguments
                env=node.env_vars,
                stdout=subprocess.PIPE,  # Redirect stdout for monitoring
                stderr=subprocess.PIPE,  # Redirect stderr for monitoring
                text=True,
                start_new_session=True,  # Start each process in a new process group
            )
            self._processes.append(process)
            print(f"PID: {process.pid}, Started Node: {node.file_name}")

    async def _spin_and_poll_subprocesses(self) -> None:
        while True:
            if any([p.poll() is not None for p in self._processes]):
                self._gather_all_errors()
                return

            time.sleep(0.1)

    def _gather_all_errors(self) -> None:
        for process in self._processes:
            stdout, stderr = process.communicate()
            if process.returncode == 0:
                continue

            print(
                f"PID: {process.pid}, Result: {process.returncode=} \n"
                f"\tError : {stderr} \n\tOut: {stdout}"
            )

    def stop(self) -> None:
        """Send a SIGINT signal to all child processes to stop them."""
        if not self._running:
            return

        for process in self._processes:
            try:
                print(f"PID: {process.pid}, Sending interrupt to process.")
                process.send_signal(signal.SIGINT.value)
                # Wait for process to terminate
                process.wait(5)
            except ProcessLookupError:
                print(f"PID: {process.pid}, Unable to find process")
            else:
                print(
                    f"PID: {process.pid}, Stopped process with result {process.returncode}"
                )

        self._running = False

    def _rcv_signal(self, signal_: Optional[int], frame: Optional[FrameType]) -> None:
        print(f"Received shutdown signal {signal_=}")
        self.stop()

    def _add_cleanup_signals(self) -> None:
        catchable_signals = set(signal.Signals) - {
            signal.SIGKILL,
            signal.SIGSTOP,
            signal.SIGCHLD,
        }
        for signal_ in catchable_signals:
            signal.signal(signal_, self._rcv_signal)


def _gen_profile(profile: str) -> List[NodeConfig]:
    if profile == "hw":
        return [NodeConfig.ublox_data_node(), NodeConfig.motor_control_node()]
    elif profile == "rc":
        return [
            NodeConfig.ublox_data_node(),
            NodeConfig.motor_control_node(),
            NodeConfig.rc_node(),
        ]
    else:
        raise NotImplementedError(f"Unexpected profile: {profile}")
