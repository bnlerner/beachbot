from __future__ import annotations

import os
import pathlib
import signal
import subprocess
import time
from typing import List

import pydantic
import system_info

_NODE_FILE_PATH = system_info.get_root_project_directory() / "src/python/node/"
_CMD = "python3"
_OPTIONS = "-u"


class NodeConfig(pydantic.BaseModel):
    file_name: str
    args: str = ""

    @property
    def file_path(self) -> pathlib.Path:
        return _NODE_FILE_PATH / self.file_name

    @classmethod
    def rc_node(cls) -> NodeConfig:
        return NodeConfig(file_name="rc_node.py")

    @classmethod
    def motor_control_node(cls) -> NodeConfig:
        return NodeConfig(file_name="motor_control_node.py")

    @classmethod
    def ublox_data_node(cls) -> NodeConfig:
        return NodeConfig(file_name="ublox_data_node.py")


class Orchestrator:
    def __init__(self, profile: str):
        if profile == "hw":
            self._profile = [
                NodeConfig.ublox_data_node(),
                NodeConfig.motor_control_node(),
            ]
        else:
            raise NotImplementedError(f"Unexpected profile: {profile}")
        # List to keep track of child processes
        self._processes: List[subprocess.Popen] = []

    async def run(self) -> None:
        """Start a new node process."""
        self._create_sub_processes()
        await self._spin_and_poll_subprocesses()

    def _create_sub_processes(self) -> None:
        for node in self._profile:
            process = subprocess.Popen(
                [_CMD, _OPTIONS, node.file_path, node.args],  # Command and arguments
                stdout=subprocess.PIPE,  # Redirect stdout for monitoring
                stderr=subprocess.PIPE,  # Redirect stderr for monitoring
                preexec_fn=os.setsid,  # Start each process in a new process group
            )
            self._processes.append(process)
            print(f"Started Node: {node.file_name} with PID {process.pid}")

    async def _spin_and_poll_subprocesses(self) -> None:
        while True:
            for process in self._processes:
                if poll_result := process.poll():
                    print(f"{process.args=}, {poll_result=}")

            time.sleep(0.1)

    def stop(self) -> None:
        """Send a SIGINT signal to all child processes to stop them."""
        for process in self._processes:
            # Send SIGINT to process group
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait()  # Wait for process to terminate
            print(f"Stopped {process=} with PID {process.pid}")
