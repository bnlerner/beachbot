import asyncio
import os
import sys
import time
from concurrent.futures import Executor, ThreadPoolExecutor

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import geometry
from drivers.camera import async_camera


async def process_camera(camera: async_camera.AsyncCamera, executor: Executor) -> None:
    while True:
        start = time.perf_counter()
        await camera.update(executor)
        total_time = time.perf_counter() - start
        print(f"{camera._frame} {total_time=}, {1/total_time} hz")
        await asyncio.sleep(0.01)
        depth_map = camera.depth_map()
        tracked_objects = camera.tracked_objects()
        image = camera.image()


async def create_camera(
    frame: geometry.ReferenceFrame, executor: Executor
) -> async_camera.AsyncCamera:
    camera = async_camera.AsyncCamera(frame)
    await camera.open(executor)
    camera.enable_object_detection()

    return camera


async def main() -> None:
    with ThreadPoolExecutor(max_workers=4) as executor:
        front_camera = await create_camera(geometry.FRONT_CAMERA, executor)
        rear_camera = await create_camera(geometry.REAR_CAMERA, executor)
        try:
            front_process_task = asyncio.create_task(
                process_camera(front_camera, executor)
            )
            rear_process_task = asyncio.create_task(
                process_camera(rear_camera, executor)
            )
            await asyncio.gather(front_process_task, rear_process_task)
        except KeyboardInterrupt:
            pass
        finally:
            front_camera.close()
            rear_camera.close()


if __name__ == "__main__":
    asyncio.run(main())
