from __future__ import annotations

import json
import pathlib
import sys
from typing import Optional, Type, TypeVar

import pydantic
import system_info
from config import motor_config

_SUFFIX = ".json"
_MSG_DIR = system_info.get_root_project_directory() / "var/msgs"
BaseMessageT = TypeVar("BaseMessageT", bound="BaseMessage")


class BaseMessage(pydantic.BaseModel):
    def write(self, *name_extensions: str) -> None:
        with open(self.filepath(*name_extensions), "w") as fp:
            msg_dict = self.model_dump()
            msg_dict["ts"] = system_info.timestamp()
            json.dump(msg_dict, fp)

    @classmethod
    def read(cls: Type[BaseMessageT], *name_extensions: str) -> Optional[BaseMessageT]:
        if cls.filepath(*name_extensions).exists():
            with open(cls.filepath(*name_extensions), "r") as fp:
                msg_data = json.load(fp)
                return cls.model_validate(msg_data)
        else:
            return None

    @classmethod
    def filepath(cls, *name_extensions: str) -> pathlib.Path:
        return (_MSG_DIR / cls.filename(*name_extensions)).with_suffix(_SUFFIX)

    @classmethod
    def filename(cls, *name_extensions: str) -> str:
        return f"{cls.__name__}_" + "_".join(name_extensions)

    def size(self) -> int:
        """Size of the message in bytes."""
        return sys.getsizeof(self)


class GPSMessage(BaseMessage):
    """The GPS data from the body mounted GPS receiver."""

    latitude: float
    longitude: float


class VehicleDynamicsMessage(BaseMessage):
    """The vehicle dynamics data that comes from the body mounted IMU."""

    roll: float
    pitch: float
    heading: float
    roll_rate: float
    pitch_rate: float
    yaw_rate: float
    roll_acceleration: float
    pitch_acceleration: float
    heading_acceleration: float


class MotorCommandMessage(BaseMessage):
    """The target motor commands to use for control of each motor."""

    motor: motor_config.MotorConfig
    velocity: float
