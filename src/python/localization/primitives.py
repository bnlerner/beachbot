from __future__ import annotations

from typing import Literal

import pydantic


class UTMZone(pydantic.BaseModel):
    """A UTM zone which defines the zone number, hemisphere and its epsg code."""

    zone_number: int
    hemisphere: Literal["north", "south"]
    epsg_code: str

    @property
    def zone_central_longitude(self) -> float:
        """The central meridian of the UTM zone."""
        return (self.zone_number - 1) * 6 - 180 + 3

    @classmethod
    def from_coordinates(cls, longitude: float, latitude: float) -> UTMZone:
        # Calculate UTM zone number from longitude
        zone_number = int((longitude + 180) / 6) + 1

        # EPSG code for UTM depends on hemisphere
        # 32600 + zone for northern hemisphere
        # 32700 + zone for southern hemisphere
        # Northern Hemisphere has latitude > 0
        hemisphere: Literal["north", "south"] = "north" if latitude >= 0 else "south"
        epsg_code = (
            "EPSG:" + f"326{zone_number}" if latitude >= 0 else f"327{zone_number}"
        )

        return UTMZone(
            zone_number=zone_number, hemisphere=hemisphere, epsg_code=epsg_code
        )

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, UTMZone):
            return NotImplemented

        return (
            self.zone_number == other.zone_number
            and self.hemisphere == other.hemisphere
            and self.epsg_code == other.epsg_code
        )

    def __str__(self) -> str:
        return (
            f"{self.__class__}(zone: {self.zone_number}, hemisphere: {self.hemisphere}, "
            f"epsg_code: {self.epsg_code})"
        )
