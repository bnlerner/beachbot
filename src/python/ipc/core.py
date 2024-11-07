from __future__ import annotations

from typing import Generic, Optional, TypeVar

import pydantic

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


class ChannelSpec(pydantic.BaseModel, Generic[BaseMessageT]):
    """The identification of a method of sending a message from
    a publisher to a subscriber.
    """

    channel: str

    def name(self) -> str:
        return self.channel

    def __hash__(self) -> int:
        return hash((self.__class__, self.name))
