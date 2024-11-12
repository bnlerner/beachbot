from __future__ import annotations

import asyncio
import pathlib
import signal
import subprocess
from types import FrameType
from typing import Dict, List, Literal, Optional

import pydantic
import system_info

_NODE_FILE_PATH = system_info.get_root_project_directory() / "src/python/node/"
_CMD = "python3"
_OPTIONS = "-u"


class NodeConfig(pydantic.BaseModel):
    """Describes the node and how to run it."""

    file_name: str
    args: str = ""
    env_vars: Dict[str, str] = {}

    @property
    def file_path(self) -> pathlib.Path:
        return _NODE_FILE_PATH / self.file_name

    ####################################################################################
    # CONFIGURED NODES #################################################################
    ####################################################################################

    @classmethod
    def rc_node(cls) -> NodeConfig:
        return NodeConfig(file_name="rc_node.py", env_vars={"DISPLAY": ":1"})

    @classmethod
    def motor_control_node(cls) -> NodeConfig:
        return NodeConfig(file_name="motor_control_node.py")

    @classmethod
    def ublox_data_node(cls) -> NodeConfig:
        return NodeConfig(file_name="ublox_data_node.py")

    @classmethod
    def ui_node(cls) -> NodeConfig:
        return NodeConfig(file_name="ui_node.py")


class Orchestrator:
    """Allows running multiple nodes, each in a different process via a profile.
    Cancellation of all nodes is possible by stopping the orchestrator. Monitors
    nodes after creation and stops all other nodes if one has exited.

    Opts not to use containers for size constraints and no need to separate environments
    but this architecture could support that in the future using either docker or docker
    compose.
    """

    def __init__(self, mode: Literal["ui", "rc"]):
        self._profile = _gen_profile(mode)
        # List to keep track of child processes
        self._processes: List[subprocess.Popen] = []
        self._add_cleanup_signals()
        self._running: bool = False

    async def run(self) -> None:
        """Start a new node process."""
        self._running = True
        self._create_processes()
        # Any error causing a node to exit effectively causes this method to exit,
        # allowing the user to stop the orchestrator if desired.
        await self._monitor_subprocesses()

    def _create_processes(self) -> None:
        for node in self._profile:
            process = subprocess.Popen(
                [_CMD, _OPTIONS, node.file_path, node.args],
                env=node.env_vars,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                # Start each process in a new process group. Each process is within a
                # new thread as well to allow the CPU scheduler to load balance between
                # CPU cores as desired. We opt to not force processes onto CPU cores.
                start_new_session=True,
            )
            self._processes.append(process)
            print(f"PID: {process.pid}, Started Node: {node.file_name}")

    async def _monitor_subprocesses(self) -> None:
        """Monitors each created subprocess for any nodes that have exited and prints
        any output. Any process that exits for any reason triggers all other processes
        to exit.
        """
        while True:
            if any([p.poll() is not None for p in self._processes]):
                self._gather_all_errors()
                return

            await asyncio.sleep(0.1)

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
        """Stops all node processes if they are still running. Safe to call multiple
        times as processes are only stopped once.
        """
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


def _gen_profile(profile: Literal["ui", "rc"]) -> List[NodeConfig]:
    if profile == "ui":
        return [
            NodeConfig.ublox_data_node(),
            NodeConfig.motor_control_node(),
            NodeConfig.ui_node(),
        ]
    elif profile == "rc":
        return [
            NodeConfig.ublox_data_node(),
            NodeConfig.motor_control_node(),
            NodeConfig.rc_node(),
        ]
    else:
        raise NotImplementedError(f"Unexpected profile: {profile}")
