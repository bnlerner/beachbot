import asyncio
import os
import sys

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ipc import messages, pubsub, registry, request
from typing_helpers import req

_REQUEST_SPEC = registry.Requests.NAVIGATE


async def main(client: request.RequestClient) -> None:
    sub = pubsub.Subscriber(
        registry.NodeIDs.TEST0, registry.Channels.BODY_KINEMATICS, None
    )
    kin_msg = req(await sub.wait_for_message())
    target = kin_msg.pose.from_local(1, 0, 0)
    msg = messages.NavigateRequest(target=target)

    try:
        response = await client.send(msg)
    except KeyboardInterrupt:
        pass
    else:
        print(f"{response=}")
    finally:
        request_client.close()


if __name__ == "__main__":
    request_client = request.RequestClient(registry.NodeIDs.TEST0, _REQUEST_SPEC)
    asyncio.run(main(request_client))
