from typing import List

import rsplan

from planning import primitives

# The turn radius of the reed sheeps path. In meters
_TURN_RADIUS = 2
# Keep a small step size between nav path points. In meters.
_STEP_SIZE = 0.05
# No runway needed because dont care too much about final orientation.
_RUNWAY_SIZE = 0.0


class NavigationPathPlanner:
    def __init__(self, obstacles: List[primitives.Obstacle]):
        if len(obstacles) != 0:
            raise ValueError("Obstacles not supported at the moment.")

        self._obstacles = obstacles

    def gen_path(
        self, start: primitives.GPSPoint, start_yaw: float, end: primitives.GPSPoint
    ) -> primitives.NavigationPath:
        # Replace with way to calculate the state of the robot. Poss
        end_yaw = start.angle_to(end)
        start_pose = primitives.NavigationPoint(
            point=start, yaw=start_yaw, driving_direction=1
        ).to_rs_nav_plan_point()
        end_pose = primitives.NavigationPoint(
            point=end, yaw=end_yaw, driving_direction=1
        ).to_rs_nav_plan_point()

        path = rsplan.path(start_pose, end_pose, _TURN_RADIUS, _RUNWAY_SIZE, _STEP_SIZE)

        # TODO: Respect obstacles and use RRT to create a node graph around them.
        return primitives.NavigationPath.from_rs_nav_path(path)
