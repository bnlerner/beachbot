"""
A simple example to print RC commands.
"""
import asyncio
import os
import sys
from typing import Optional, Union

from pynput import keyboard

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from node import base_node


class RCRobotNode(base_node.BaseNode):
    def __init__(self) -> None:
        super().__init__()
        self._rc_listener = keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release
        )
        self._rc_listener.start()

        self.add_tasks(self._loop_and_no_nothing)

    def _on_press(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]) -> None:
        try:
            if key == keyboard.Key.up:
                print("Up arrow key pressed")
            elif key == keyboard.Key.down:
                print("Down arrow key pressed")
            elif key == keyboard.Key.left:
                print("Left arrow key pressed")
            elif key == keyboard.Key.right:
                print("Right arrow key pressed")
            elif key == keyboard.Key.esc:
                sys.exit(0)
        except AttributeError:
            print("special key {0} pressed".format(key))

    def _on_release(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]) -> None:
        if key == keyboard.Key.up:
            print("Up arrow key released")
        elif key == keyboard.Key.down:
            print("Down arrow key released")
        elif key == keyboard.Key.left:
            print("Left arrow key released")
        elif key == keyboard.Key.right:
            print("Right arrow key released")

    async def _loop_and_no_nothing(self) -> None:
        while True:
            await asyncio.sleep(10)

    async def shutdown_hook(self) -> None:
        self._rc_listener.stop()


if __name__ == "__main__":
    node = RCRobotNode()
    node.start()
