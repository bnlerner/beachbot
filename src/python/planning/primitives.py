from __future__ import annotations

from typing import List, Literal, Optional

import geometry
import pydantic
import rsplan


class NavigationPath(pydantic.BaseModel):
    points: List[NavigationPoint]
    turn_radius: float
    step_size: float

    @classmethod
    def from_rs_nav_path(cls, rs_nav_path: rsplan.Path) -> NavigationPath:
        rs_wps = rs_nav_path.waypoints()
        points = []
        for cur_wp, next_wp in zip(rs_wps[:-1], rs_wps[1:], strict=True):
            is_cusp_point = cur_wp.driving_direction != next_wp.driving_direction
            points.append(NavigationPoint.from_rs_wp(cur_wp, is_cusp_point))

        # add the last waypoint
        points.append(NavigationPoint.from_rs_wp(rs_wps[-1], False))
        return NavigationPath(
            points=points,
            turn_radius=rs_nav_path.turn_radius,
            step_size=rs_nav_path.step_size,
        )

    @property
    def end(self) -> NavigationPoint:
        return self.points[-1]

    @property
    def start(self) -> NavigationPoint:
        return self.points[0]


class NavigationPoint(pydantic.BaseModel):
    point: geometry.Position
    # Yaw is an angle.
    yaw: float
    driving_direction: Literal[-1, 1]
    turn_direction: Literal[-1, 0, 1]
    is_cusp_point: bool

    @property
    def pose_2d(self) -> geometry.Pose:
        return geometry.Pose(
            self.point.to_2d(), geometry.Orientation(self.point.frame, 0, 0, self.yaw)
        )

    @classmethod
    def from_rs_wp(
        cls, waypoint: rsplan.Waypoint, is_cusp_point: bool
    ) -> NavigationPoint:
        x, y, yaw = waypoint.pose_2d_tuple
        return NavigationPoint(
            point=geometry.Position(geometry.UTM, x, y, 0.0),
            yaw=yaw,
            driving_direction=waypoint.driving_direction,
            turn_direction=waypoint.turn_direction,
            is_cusp_point=is_cusp_point,
        )

    def signed_turn_radius(self, turn_radius: float) -> Optional[float]:
        if self.turn_direction == 0:
            return None

        return self.turn_direction * turn_radius


class Obstacle(pydantic.BaseModel):
    """A 2D obstacle located in the MAP."""

    center: geometry.Position
    radius: float
