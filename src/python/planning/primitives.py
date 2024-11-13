from __future__ import annotations

import math
from typing import List, Literal, Tuple

import pydantic
import rsplan


class NavigationPath(pydantic.BaseModel):
    points: List[NavigationPoint]
    turn_radius: float

    @classmethod
    def from_rs_nav_path(cls, rs_nav_path: rsplan.Path) -> NavigationPath:
        points = [NavigationPoint.from_rs_wp(wp) for wp in rs_nav_path.waypoints()]
        return NavigationPath(points=points, turn_radius=rs_nav_path.turn_radius)


class NavigationPoint(pydantic.BaseModel):
    point: GPSPoint
    # Yaw is an angle.
    yaw: float
    driving_direction: Literal[-1, 1]

    def to_rs_nav_plan_point(self) -> Tuple[float, float, float]:
        # NOTE: Requires a yaw angle.
        return self.point.latitude, self.point.longitude, self.yaw

    @classmethod
    def from_rs_wp(cls, waypoint: rsplan.Waypoint) -> NavigationPoint:
        x, y, yaw = waypoint.pose_2d_tuple
        gps_point = GPSPoint(latitude=x, longitude=y)
        return NavigationPoint(
            point=gps_point, yaw=yaw, driving_direction=waypoint.driving_direction
        )


class GPSPoint(pydantic.BaseModel):
    latitude: float
    longitude: float

    def __add__(self, other: GPSPoint) -> GPSPoint:
        return GPSPoint(
            latitude=self.latitude + other.latitude,
            longitude=self.longitude + other.longitude,
        )

    def __sub__(self, other: GPSPoint) -> GPSPoint:
        return GPSPoint(
            latitude=self.latitude - other.latitude,
            longitude=self.longitude - other.longitude,
        )

    def angle_to(self, other: GPSPoint) -> float:
        """The absolute angle in the global GPS frame between the two GPS points."""
        translation = self - other
        return math.degrees(math.atan2(translation.latitude, translation.longitude))


class Obstacle(pydantic.BaseModel):
    """A 2D obstacle located in the MAP."""

    center: GPSPoint
    radius: float
