import pydantic
from typing import Dict

class GPSMessage(pydantic.BaseModel):
    """The GPS data from the body mounted GPS receiver."""
    
    latitude: float
    longitude: float

    filename = "gps_msg.json"

    def to_json(self) -> Dict:
        return self.json()

class VehicleDynamicsMessage(pydantic.BaseModel):
    """The vehicle dynamics data that comes from the body mounted IMU."""

    roll: float
    pitch: float
    heading: float
    roll_acceleration: float
    pitch_acceleration: float
    heading_acceleration: float
