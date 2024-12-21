from __future__ import annotations

import enum
from typing import Optional


class CalibrationStatus(enum.Enum):
    """Whether the IMU is calibrated or not. Different internal levels of accuracy
    determine if the IMU is calibrated. Still returns values if not calibrated just they
    may not be reliable.
    """

    UNRELIABLE = 0
    LOW_ACC = 1
    MED_ACC = 2
    HIGH_ACC = 3

    @property
    def is_calibrated(self) -> bool:
        return self.value > 1


class StabilityStatus(enum.Enum):
    """The Status of this objects stability according to the IMU. Contains several basic
    states to indicate if the object is moving or not.
    """

    UNKNOWN = "UNKNOWN"
    STABLE = "STABLE"
    IN_MOTION = "IN_MOTION"

    @classmethod
    def from_stability_classification(cls, value: Optional[str]) -> StabilityStatus:
        """Returns the stability status based on the IMU's internal classification."""
        if value in ("On Table", "Stationary", "Stable"):
            return cls.STABLE
        elif value == "In motion":
            return cls.IN_MOTION
        elif value == "Unknown":
            return cls.UNKNOWN
        else:
            raise ValueError(f"Unknown classification: '{value}'")
