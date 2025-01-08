import asyncio
from concurrent.futures import Executor
from typing import List, Literal, Optional

import geometry
import log
from pyzed import sl  # type:ignore[import-untyped]

from drivers import camera
from drivers.camera import primitives


class AsyncCamera:
    """An asynchronous enabled ZED camera. Used for grabbing frames, processing them and
    enabling access to the underlying image, objects or depth map.

    NOTE: At this time localization is not possible due to IMU fusion being turned off.
    Also because the cameras are flipped but that seems fixable.
    """

    def __init__(self, frame: geometry.ReferenceFrame):
        self._frame = frame
        self._serial_number = self._get_serial_number(frame)
        self._runtime_parameters = sl.RuntimeParameters(confidence_threshold=50)

        self._camera = sl.Camera()
        self._objects = sl.Objects()
        self._image = sl.Mat()
        self._depth_map = sl.Mat()

        self._is_open: bool = False
        self._resolution: sl.Resolution

        self._cached_tracked_objects: List[primitives.TrackedObjects] = []

    @property
    def is_open(self) -> bool:
        return self._is_open

    @property
    def width(self) -> int:
        return self._resolution.width

    @property
    def height(self) -> int:
        return self._resolution.height

    def image(self) -> Optional[primitives.Image]:
        """Image from the camera including any bounding boxes from tracked objects."""
        if image := primitives.Image.from_zed_image(self._image):
            image.add_bounding_boxes(self._cached_tracked_objects)
            image.reduce()
            return image
        else:
            return None

    def depth_map(self) -> Optional[primitives.DepthMap]:
        """The depth map in the camera frame."""
        array = self._depth_map.get_data()
        if array.size != 0:
            return primitives.DepthMap(array)
        else:
            return None

    def tracked_objects(self) -> List[primitives.TrackedObjects]:
        """Tracked objects from obstacle detection. Only available if object detection
        is enabled.
        """
        if self._objects.is_new:
            self._cached_tracked_objects = [
                primitives.TrackedObjects.from_zed_object(self._frame, obj)
                for obj in self._objects.object_list
            ]

        return self._cached_tracked_objects

    async def open(self, executor: Executor) -> bool:
        """Opens the ZED camera allowing for it to start grabbing data. Runs in an
        executor to prevent blocking the entire process.
        """
        init_params = sl.InitParameters(
            depth_mode=sl.DEPTH_MODE.ULTRA,
            coordinate_units=sl.UNIT.METER,
            # Right handed coordinate system with the X-axis pointed out of the left
            # camera lens and the z-axis pointed upward relvative to the camera. Origin
            # appears to be at the camera itself and not the front of the camera glass.
            coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD,
            camera_image_flip=sl.FLIP_MODE.ON,
            depth_maximum_distance=20.0,
            depth_minimum_distance=0.3,
            # NOTE: The default but we may want to change it.
            async_image_retrieval=False,
        )

        init_params.set_from_serial_number(self._serial_number)

        status = await asyncio.get_event_loop().run_in_executor(
            executor, self._camera.open, init_params
        )
        self._is_open = status == sl.ERROR_CODE.SUCCESS
        self._resolution = (
            self._camera.get_camera_information().camera_configuration.resolution
        )
        return self._is_open

    def enable_object_detection(self, enable_tracking: bool = True) -> None:
        """Enables object detection in the camera."""
        if enable_tracking:
            self._enable_positional_tracking()

        batch_parameters = sl.BatchParameters(enable=True)
        obj_param = sl.ObjectDetectionParameters(
            batch_trajectories_parameters=batch_parameters,
            detection_model=sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST,
            enable_tracking=enable_tracking,
        )
        state = self._camera.enable_object_detection(obj_param)
        if state != sl.ERROR_CODE.SUCCESS:
            log.error(f"Unable to start object detection: {self._serial_number}")

    def _enable_positional_tracking(self) -> None:
        """Enable positional tracking. Required for object detection."""
        pose_tracking_params = sl.PositionalTrackingParameters()
        # NOTE: GEN_2 is a higher performance mode for positional tracking, allowing
        # better accuracy but has a memory leak when using IMU fusion!
        pose_tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1
        pose_tracking_params.enable_imu_fusion = False
        positional_init = self._camera.enable_positional_tracking(pose_tracking_params)
        if positional_init != sl.ERROR_CODE.SUCCESS:
            log.error(f"Can't start tracking of camera: {self._serial_number}")

    def enable_odometry_publishing(self, *, fusion: Optional[sl.Fusion] = None) -> None:
        """Enables odometry publishing. Required for localization via the fusion object.
        Also used for reading camera sensor values like the IMU.
        """
        configuration = sl.CommunicationParameters()
        self._camera.start_publishing(configuration)
        if fusion is not None:
            uuid = sl.CameraIdentifier(self._serial_number)
            zero_transform = sl.Transform(0, 0, 0)
            fusion.subscribe(uuid, configuration, zero_transform)

    async def update(self, executor: Executor) -> None:
        """Updates the camera frame by grabbing frames asynchronously, updating the
        image, object list and depth map.
        """
        if await self._grab(executor):
            await self._retrieve_image(executor)
            await self._retrieve_objects(executor)
            await self._retrieve_depth_map(executor)
        else:
            log.error(f"Camera {self._serial_number}: Grab failed.")

    async def _grab(self, executor: Executor) -> bool:
        """Grabs the latest frame from the camera asynchronously."""
        status = await asyncio.get_event_loop().run_in_executor(
            executor, self._camera.grab, self._runtime_parameters
        )
        return status == sl.ERROR_CODE.SUCCESS

    def _get_serial_number(self, frame: geometry.ReferenceFrame) -> int:
        if frame == geometry.FRONT_CAMERA:
            return camera.FORWARD_CAMERA_SN
        elif frame == geometry.REAR_CAMERA:
            return camera.REAR_CAMERA_SN
        else:
            raise ValueError(f"Unknown Frame: {frame}")

    async def _retrieve_image(
        self, executor: Executor, view: Literal["left", "right"] = "left"
    ) -> None:
        """Retrieves the image asynchronously after grabbing."""
        view = sl.VIEW.LEFT if view == "left" else sl.VIEW.RIGHT
        status = await asyncio.get_event_loop().run_in_executor(
            executor, self._camera.retrieve_image, self._image, view
        )
        if status != sl.ERROR_CODE.SUCCESS:
            log.error(f"Unable to retrieve image: {self._serial_number}")

    async def _retrieve_depth_map(self, executor: Executor) -> None:
        """Retrieves the depth map asynchronously from the camera."""
        status = await asyncio.get_event_loop().run_in_executor(
            executor, self._camera.retrieve_measure, self._depth_map, sl.MEASURE.DEPTH
        )
        if status != sl.ERROR_CODE.SUCCESS:
            log.error(f"Unable to retrieve depth map: {self._serial_number}")

    async def _retrieve_objects(self, executor: Executor) -> None:
        """Retrieves the detected objects asynchronously from the camera."""
        detection_parameters_rt = sl.ObjectDetectionRuntimeParameters(
            detection_confidence_threshold=60
        )
        status = await asyncio.get_event_loop().run_in_executor(
            executor,
            self._camera.retrieve_objects,
            self._objects,
            detection_parameters_rt,
        )
        if status != sl.ERROR_CODE.SUCCESS:
            log.error(f"Unable to retrieve objects: {self._serial_number}")

    def close(self) -> None:
        """Closes the ZED camera."""
        self._is_open = False
        self._camera.disable_object_detection()
        self._camera.close()
