import asyncio
import os
import sys
import time

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ipc import messages, pubsub, registry

_MSG_TYPE = messages.CameraImageMessage
_CHANNEL = registry.Channels.FRONT_CAMERA_IMAGE
_MSGS_RECEIVED = 0


def _print_msg(msg: _MSG_TYPE) -> None:
    global _MSGS_RECEIVED
    _MSGS_RECEIVED += 1
    if image := msg.image.serialized():
        length = len(image)
    else:
        length = 0
    print(f"{msg.origin=}, {msg.creation=}, {length=}")


async def main(sub: pubsub.Subscriber) -> None:
    print("Starting to listen to messages.")
    await sub.listen()


if __name__ == "__main__":
    sub = pubsub.Subscriber(registry.NodeIDs.TEST0, _CHANNEL, _print_msg)
    try:
        start = time.perf_counter()
        asyncio.run(main(sub))
    except KeyboardInterrupt:
        end = time.perf_counter()
        rate = _MSGS_RECEIVED / (end - start)
        print(f"\n{_CHANNEL.name} received at {rate} hz")
