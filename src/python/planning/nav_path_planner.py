from typing import List

import geometry
import rsplan

from planning import primitives

# The turn radius of the reed sheeps path. In meters
_TURN_RADIUS = 2
# Keep a small step size between nav path points. In meters.
_STEP_SIZE = 0.05
# No runway needed because dont care too much about final orientation.
_RUNWAY_SIZE = 0.0


class NavPathPlanner:
    def __init__(self, obstacles: List[primitives.Obstacle]):
        if len(obstacles) != 0:
            raise ValueError("Obstacles not supported at the moment.")

        self._obstacles = obstacles

    def gen_path(
        self, start: geometry.Pose, end: geometry.Position
    ) -> primitives.NavigationPath:
        # Replace with way to calculate the state of the robot. Poss
        end_yaw = start.position.angle_to(end)
        start_pose = (start.position.x, start.position.y, start.orientation.yaw)
        end_pose = (end.x, end.y, end_yaw)

        path = rsplan.path(start_pose, end_pose, _TURN_RADIUS, _RUNWAY_SIZE, _STEP_SIZE)

        # TODO: Respect obstacles and use RRT to create a node graph around them.
        return primitives.NavigationPath.from_rs_nav_path(path)
