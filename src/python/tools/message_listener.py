import asyncio
import os
import sys
import time

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ipc import messages, pubsub, registry

_MSG_TYPE = messages.IMUMessage
_CHANNEL = registry.Channels.IMU
_MSGS_RECEIVED = 0


def _print_msg(msg: _MSG_TYPE) -> None:
    global _MSGS_RECEIVED
    _MSGS_RECEIVED += 1
    print(f"{msg=}")


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
