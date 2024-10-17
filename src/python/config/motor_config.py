from typing import Literal
import pydantic
import enum


class MotorLocation(enum.Enum):
    FRONT_LEFT = "FRONT_LEFT"
    FRONT_RIGHT = "FRONT_RIGHT"
    REAR_LEFT = "REAR_LEFT"
    REAR_RIGHT = "REAR_RIGHT"


class MotorConfig(pydantic.BaseModel):
    node_id: int
    location: MotorLocation

    @property
    def direction(self) -> Literal[1, -1]:
        if self.location in (MotorLocation.FRONT_LEFT, MotorLocation.REAR_LEFT):
            return -1
        else:
            return 1