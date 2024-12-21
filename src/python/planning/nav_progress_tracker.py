from typing import Optional

import geometry
import log
from models import constants
from typing_helpers import req

from planning import primitives

_NAV_PATH_DISTANCE_TRACKING_TOL = 0.6
_RAMP_WIDTH = 1.0


class NavProgressTracker:
    """Tracks the progress along the navigation path, providing useful information."""

    def __init__(self, nav_path: primitives.NavigationPath):
        self._nav_path = nav_path

        self._cur_point_ix = 1

        self._target_speed = (
            # TODO: Move slow for now until we gain confidence in the system.
            self.current_navpoint.driving_direction * constants.MAX_LINEAR_SPEED * 0.1
        )
        self._cached_ref_pose_2d: Optional[geometry.Pose] = None

    @property
    def current_navpoint(self) -> primitives.NavigationPoint:
        return self._nav_path.points[self._cur_point_ix]

    @property
    def previous_navpoint(self) -> primitives.NavigationPoint:
        return self._nav_path.points[self._cur_point_ix - 1]

    @property
    def ref_pose_2d(self) -> geometry.Pose:
        return req(self._cached_ref_pose_2d)

    def is_ready(self) -> bool:
        """Tracker has been updated and ready to use."""
        return self._cached_ref_pose_2d is not None

    def update(self, cur_pose: geometry.Pose) -> None:
        self._cached_ref_pose_2d = cur_pose
        if (next_navpoint := self._get_navpoint(self._cur_point_ix + 1)) is not None:
            next_navpoint_dist = self.ref_pose_2d.position.distance(next_navpoint.point)
            previous_waypoint_dist = self.ref_pose_2d.position.distance(
                self.previous_navpoint.pose_2d.position
            )

            if self.current_navpoint.is_cusp_point:
                self._cur_point_ix += 1

                self._target_speed = (
                    next_navpoint.driving_direction * constants.MAX_LINEAR_SPEED
                )
            elif next_navpoint_dist < previous_waypoint_dist:
                self._cur_point_ix += 1

    def reference_speed_along_path(self) -> float:
        """A signed reference speed along the path. Positive goes forward. Negative
        goes backward.
        """
        distance_remaining = self._distance_to_next_stopping_point(self.ref_pose_2d)
        # A linear ramp speed reference that decreases as we approach the stopping
        # point. Ends at a minimum value to avoid moving too slowly.
        ramp_speed = max(0.3, distance_remaining / _RAMP_WIDTH)
        # Can only move as fast as the target speed.
        abs_reference_speed = min(ramp_speed, abs(self._target_speed))

        return geometry.sign(self._target_speed) * abs_reference_speed

    def is_finished(self) -> bool:
        """Opt to just stop when our target is the last point in the nav path."""
        return self._nav_path.end == self.current_navpoint

    def is_off_nav_path(self) -> bool:
        cur_pos = self.ref_pose_2d.position.to_2d()
        dist_to_current_navpoint = cur_pos.distance(
            self.current_navpoint.pose_2d.position
        )

        if dist_to_current_navpoint > _NAV_PATH_DISTANCE_TRACKING_TOL:
            log.error(
                f"Too far off nav path, {dist_to_current_navpoint=}, {cur_pos=}, "
                f"{self.current_navpoint.pose_2d.position=}"
            )
            return True
        else:
            return False

    def _distance_to_next_stopping_point(self, pose: geometry.Pose) -> float:
        distance_to_current_navpoint = pose.position.distance(
            self.current_navpoint.pose_2d.position
        )

        stopping_point_ix = len(self._nav_path.points) - 1
        for ix, proximal_navpoint in enumerate(
            self._nav_path.points[self._cur_point_ix :]
        ):
            if proximal_navpoint.is_cusp_point:
                stopping_point_ix = ix + self._cur_point_ix
                break

        distance_along_nav_path_until_stop = (
            stopping_point_ix - self._cur_point_ix
        ) * self._nav_path.step_size
        return distance_to_current_navpoint + distance_along_nav_path_until_stop

    def _get_navpoint(self, ix: int) -> Optional[primitives.NavigationPoint]:
        """Returns the Navpoint at the index. Returns None if at the end of the path."""
        try:
            return self._nav_path.points[ix]
        except IndexError:
            # We have reached the end of the path.
            return None
