from __future__ import annotations

import json
import pathlib
from typing import Optional, TypeVar

import pydantic
import system_info

_MSG_DIR = "var/gps_msgs"
BaseMessageT = TypeVar("BaseMessageT", bound="BaseMessage")


class BaseMessage(pydantic.BaseModel):
    filename: str

    @property
    def filepath(self) -> pathlib.Path:
        return system_info.get_root_project_directory() / _MSG_DIR / self.filename

    def write(self) -> None:
        with open(self.filepath, "w") as fp:
            json.dump(self.model_dump(), fp)

    def read(self) -> Optional[BaseMessageT]:
        if self.filepath.exists():
            with open(self.filepath, "r") as fp:
                msg_data = json.load(fp)
                return self.model_validate(msg_data)
        else:
            return None


class GPSMessage(BaseMessage):
    """The GPS data from the body mounted GPS receiver."""

    latitude: float
    longitude: float

    filename = "gps_msg.json"


class VehicleDynamicsMessage(BaseMessage):
    """The vehicle dynamics data that comes from the body mounted IMU."""

    roll: float
    pitch: float
    heading: float
    roll_acceleration: float
    pitch_acceleration: float
    heading_acceleration: float

    filename = "vehicle_dynamics_msg.json"
