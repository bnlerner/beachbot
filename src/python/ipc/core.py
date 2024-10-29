from typing import Generic, Type, TypeVar

import pydantic

MessageT = TypeVar("MessageT")


class NodeID(pydantic.BaseModel):
    name: str

    def __hash__(self) -> int:
        return hash((self.__class__, self.name))

    def __str__(self) -> str:
        return self.name


class ChannelSpec(pydantic.BaseModel, Generic[MessageT]):
    channel: str
    msg_class: Type[MessageT]

    def name(self) -> str:
        return self.channel

    def __hash__(self) -> int:
        return hash((self.__class__, self.name))
