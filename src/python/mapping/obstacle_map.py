from __future__ import annotations

from typing import List

from drivers.camera import primitives


class ObstacleMap:
    def __init__(self, obstacles: List = []):
        self._obstacles = obstacles

    def update(self, tracked_objects: List[primitives.TrackedObjects]) -> None:
        ...

    def insert(self, obstacles: List[primitives.TrackedObjects]) -> None:
        ...
