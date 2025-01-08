import asyncio
import os
import sys
from concurrent.futures import ThreadPoolExecutor
from typing import List

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import geometry
from drivers.camera import async_camera, primitives
from ipc import messages, registry

from node import base_node

# NOTE: Publish slightly slower than the grab rate as encoding the image with cv2 is
# very slow and we should probably just use numpy instead.
_IMAGE_PUBLISH_RATE = 8


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
        self._obstacles: List[primitives.TrackedObjects]

        self.add_publishers(registry.Channels.FRONT_CAMERA_IMAGE)
        self.add_tasks(self._process_cameras)
        self.add_looped_tasks({self._publish_front_image: _IMAGE_PUBLISH_RATE})

    async def _process_cameras(self) -> None:
        await self._start_cameras()
        with ThreadPoolExecutor(max_workers=4) as executor:
            forward_task = asyncio.create_task(
                self._update_camera(self._forward_camera, executor)
            )
            rear_task = asyncio.create_task(
                self._update_camera(self._rear_camera, executor)
            )
            await asyncio.gather(forward_task, rear_task)

    async def _update_camera(
        self, camera: async_camera.AsyncCamera, executor: ThreadPoolExecutor
    ) -> None:
        while True:
            await camera.update(executor)
            await asyncio.sleep(0.01)
            self._update_obstacle_map(camera)

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

    def _update_obstacle_map(self, camera: async_camera.AsyncCamera) -> None:
        if depth_map := camera.depth_map():
            self._depth_map = depth_map

        if obstacles := camera.tracked_objects():
            self._obstacles = obstacles

    async def shutdown_hook(self) -> None:
        self._forward_camera.close()
        self._rear_camera.close()


if __name__ == "__main__":
    node = CameraNode()
    node.start()
