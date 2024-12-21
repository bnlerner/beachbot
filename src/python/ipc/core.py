from __future__ import annotations

import time
from typing import Any, Generic, Optional, TypeVar

import pydantic
from typing_helpers import req

BaseMessageT = TypeVar("BaseMessageT")


class NodeID(pydantic.BaseModel):
    """Identifies a node that is running."""

    name: str

    def __hash__(self) -> int:
        return hash((self.__class__, self.name))

    def __str__(self) -> str:
        return self.name


class BaseMessage(pydantic.BaseModel):
    """The base class for all messages sent."""

    origin: Optional[NodeID] = None
    creation: Optional[float] = None

    # Message default lifetime
    lifetime: float = 0.5

    def is_expired(self) -> bool:
        # Cannot expire if never created?
        return req(self.creation) + self.lifetime < time.perf_counter()


class ChannelSpec(pydantic.BaseModel, Generic[BaseMessageT]):
    """The identification of a method of sending a message from
    a publisher to a subscriber.
    """

    channel: str

    def name(self) -> str:
        return self.channel

    def __hash__(self) -> int:
        return hash((self.__class__, self.name))


class RequestSpec:
    """A request is a message that requires a response from the receiver."""

    def __init__(self, base_channel: str, msg_cls: type[Request]):
        self.base_channel = base_channel
        self._msg_cls = msg_cls

    @property
    def cancel_channel(self) -> ChannelSpec:
        return ChannelSpec(channel="cancel-" + self.base_channel)

    @property
    def request_channel(self) -> ChannelSpec:
        return ChannelSpec(channel="request-" + self.base_channel)

    def response_channel(self, node_id: NodeID) -> ChannelSpec:
        return ChannelSpec(channel="response-" + self.base_channel + "-" + node_id.name)


class Request(BaseMessage):
    """A Request message to complete a specified action."""

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Request):
            return NotImplemented

        return (
            self.origin == other.origin
            and self.creation == other.creation
            and self.__class__ == other.__class__
        )


class RequestResponse(BaseMessage):
    """Response to a specified request."""

    request: Request
    result: Any
    cancelled: bool


class RequestCancel(BaseMessage):
    """Request to cancel a specific request."""

    request: Request
