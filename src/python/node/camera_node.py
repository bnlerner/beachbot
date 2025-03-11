import asyncio
import collections
import os
import sys
from concurrent.futures import ThreadPoolExecutor
from typing import DefaultDict, List

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import geometry
from drivers.camera import async_camera, primitives
from ipc import messages, registry

from node import base_node

# NOTE: Publish slightly slower than the grab rate.
_PUBLISH_RATE = 5


class CameraNode(base_node.BaseNode):
    """Manages access to the ZED cameras, updating relevant objects and publishing
    images for use in other nodes. It is recommended by ZED to only access the cameras
    in a single process to avoid deadlocks, race conditions and other CUDA errors.
    """

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.CAMERA)
        self._forward_camera = async_camera.AsyncCamera(geometry.FRONT_CAMERA)
        self._rear_camera = async_camera.AsyncCamera(geometry.REAR_CAMERA)

        self._depth_map: primitives.DepthMap
        self._obstacles: DefaultDict[
            geometry.ReferenceFrame, List[primitives.TrackedObjects]
        ] = collections.defaultdict(lambda: [])

        self.add_publishers(
            registry.Channels.FRONT_CAMERA_IMAGE,
            registry.Channels.FRONT_OBSTACLES,
            registry.Channels.REAR_OBSTACLES,
        )
        self.add_tasks(self._process_cameras)
        self.add_looped_tasks(
            {
                self._publish_front_image: _PUBLISH_RATE,
                self._publish_front_obstacles: _PUBLISH_RATE,
                self._publish_rear_obstacles: _PUBLISH_RATE,
            }
        )

    async def _process_cameras(self) -> None:
        await self._start_cameras()
        with ThreadPoolExecutor(max_workers=4) as executor:
            forward_task = asyncio.create_task(
                self._update_camera(geometry.FRONT_CAMERA, executor)
            )
            rear_task = asyncio.create_task(
                self._update_camera(geometry.REAR_CAMERA, executor)
            )
            await asyncio.gather(forward_task, rear_task)

    async def _update_camera(
        self, frame: geometry.ReferenceFrame, executor: ThreadPoolExecutor
    ) -> None:
        camera = self._get_camera(frame)
        while True:
            await camera.update(executor)
            self._obstacles[frame] = camera.tracked_objects()
            await asyncio.sleep(0.01)

    async def _start_cameras(self) -> None:
        with ThreadPoolExecutor(max_workers=1) as executor:
            await self._forward_camera.open(executor)
            self._forward_camera.enable_object_detection()

            await self._rear_camera.open(executor)
            self._rear_camera.enable_object_detection()

    def _publish_front_image(self) -> None:
        if self._forward_camera.is_open and (image := self._forward_camera.image()):
            msg = messages.CameraImageMessage(image=image)
            self.publish(registry.Channels.FRONT_CAMERA_IMAGE, msg)

    def _publish_front_obstacles(self) -> None:
        obstacles = self._obstacles[geometry.FRONT_CAMERA]
        msg = messages.TrackedObjectsMessage(
            frame=geometry.FRONT_CAMERA, objects=obstacles
        )
        self.publish(registry.Channels.FRONT_OBSTACLES, msg)

    def _publish_rear_obstacles(self) -> None:
        obstacles = self._obstacles[geometry.REAR_CAMERA]
        msg = messages.TrackedObjectsMessage(
            frame=geometry.REAR_CAMERA, objects=obstacles
        )
        self.publish(registry.Channels.REAR_OBSTACLES, msg)

    def _get_camera(self, frame: geometry.ReferenceFrame) -> async_camera.AsyncCamera:
        if frame == geometry.FRONT_CAMERA:
            return self._forward_camera
        elif frame == geometry.REAR_CAMERA:
            return self._rear_camera
        else:
            raise ValueError(f"Unknown frame {frame=}")

    async def shutdown_hook(self) -> None:
        self._forward_camera.close()
        self._rear_camera.close()


if __name__ == "__main__":
    node = CameraNode()
    node.start()
