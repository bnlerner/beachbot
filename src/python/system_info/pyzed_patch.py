"""Pyzed patch module - provides cross-platform compatibility for ZED Camera SDK.
On Linux: imports real pyzed library
On other platforms: provides mock implementations for development/testing
"""
import sys
from typing import Any, Optional

import numpy as np

# Conditional import for pyzed - only available on Linux
if sys.platform == "linux":
    from pyzed import sl  # type:ignore[import-untyped]

    # Export real pyzed classes and enums
    ObjectData = sl.ObjectData
    OBJECT_CLASS = sl.OBJECT_CLASS
    Mat = sl.Mat
    RuntimeParameters = sl.RuntimeParameters
    Camera = sl.Camera
    Objects = sl.Objects
    InitParameters = sl.InitParameters
    Resolution = sl.Resolution
    BatchParameters = sl.BatchParameters
    ObjectDetectionParameters = sl.ObjectDetectionParameters
    PositionalTrackingParameters = sl.PositionalTrackingParameters
    Fusion = sl.Fusion
    CommunicationParameters = sl.CommunicationParameters
    CameraIdentifier = sl.CameraIdentifier
    Transform = sl.Transform
    ObjectDetectionRuntimeParameters = sl.ObjectDetectionRuntimeParameters
    InitFusionParameters = sl.InitFusionParameters
    GNSSCalibrationParameters = sl.GNSSCalibrationParameters
    PositionalTrackingFusionParameters = sl.PositionalTrackingFusionParameters

    # Export enums/constants
    ERROR_CODE = sl.ERROR_CODE
    DEPTH_MODE = sl.DEPTH_MODE
    UNIT = sl.UNIT
    COORDINATE_SYSTEM = sl.COORDINATE_SYSTEM
    FLIP_MODE = sl.FLIP_MODE
    OBJECT_DETECTION_MODEL = sl.OBJECT_DETECTION_MODEL
    POSITIONAL_TRACKING_MODE = sl.POSITIONAL_TRACKING_MODE
    VIEW = sl.VIEW
    MEASURE = sl.MEASURE
    FUSION_ERROR_CODE = sl.FUSION_ERROR_CODE

else:
    # Mock implementations for development on non-Linux platforms
    class _MockEnum:
        """Base class for mock enums"""

        pass

    class ERROR_CODE(_MockEnum):  # noqa: N801
        SUCCESS = 0
        FAILURE = 1

    class FUSION_ERROR_CODE(_MockEnum):  # noqa: N801
        SUCCESS = 0
        FAILURE = 1

    class DEPTH_MODE(_MockEnum):  # noqa: N801
        ULTRA = "ULTRA"
        QUALITY = "QUALITY"
        PERFORMANCE = "PERFORMANCE"

    class UNIT(_MockEnum):  # noqa: N801
        METER = "METER"
        CENTIMETER = "CENTIMETER"
        MILLIMETER = "MILLIMETER"

    class COORDINATE_SYSTEM(_MockEnum):  # noqa: N801
        RIGHT_HANDED_Z_UP_X_FWD = "RIGHT_HANDED_Z_UP_X_FWD"
        LEFT_HANDED_Y_UP = "LEFT_HANDED_Y_UP"

    class FLIP_MODE(_MockEnum):  # noqa: N801
        OFF = "OFF"
        ON = "ON"
        AUTO = "AUTO"

    class OBJECT_DETECTION_MODEL(_MockEnum):  # noqa: N801
        MULTI_CLASS_BOX_FAST = "MULTI_CLASS_BOX_FAST"
        MULTI_CLASS_BOX_ACCURATE = "MULTI_CLASS_BOX_ACCURATE"

    class POSITIONAL_TRACKING_MODE(_MockEnum):  # noqa: N801
        GEN_1 = "GEN_1"
        GEN_2 = "GEN_2"

    class VIEW(_MockEnum):  # noqa: N801
        LEFT = "left"  # Match the Literal type expectations
        RIGHT = "right"

    class MEASURE(_MockEnum):  # noqa: N801
        DEPTH = "DEPTH"
        DISPARITY = "DISPARITY"

    class OBJECT_CLASS:  # noqa: N801
        """Mock OBJECT_CLASS that behaves like an enum"""

        PERSON = "PERSON"
        VEHICLE = "VEHICLE"
        BAG = "BAG"
        ANIMAL = "ANIMAL"
        ELECTRONICS = "ELECTRONICS"
        FRUIT_VEGETABLE = "FRUIT_VEGETABLE"
        SPORT = "SPORT"

    class ObjectData:
        """Mock ObjectData for testing"""

        def __init__(self) -> None:
            self.id = 0
            self.label = OBJECT_CLASS.PERSON
            self.position = np.array([0.0, 0.0, 0.0])
            self.velocity = np.array([0.0, 0.0, 0.0])
            self.dimensions = [1.0, 1.0, 1.0]
            self.bounding_box_2d = [[100, 100], [300, 100], [300, 300], [100, 300]]
            self.bounding_box = [np.array([0.0, 0.0, 0.0]) for _ in range(8)]
            self.confidence = 90.0

    class Mat:
        """Mock Mat for testing"""

        def __init__(self) -> None:
            self._data = np.zeros((480, 640, 3), dtype=np.uint8)

        def get_data(self) -> np.ndarray:
            return self._data

    class Resolution:
        """Mock Resolution for testing"""

        def __init__(self) -> None:
            self.width = 1280
            self.height = 720

    class RuntimeParameters:
        """Mock RuntimeParameters for testing"""

        def __init__(self, confidence_threshold: int = 50) -> None:
            self.confidence_threshold = confidence_threshold

    class Camera:
        """Mock Camera for testing"""

        def __init__(self) -> None:
            self._is_open = False
            self._camera_info = self._MockCameraInfo()

        class _MockCameraInfo:
            def __init__(self) -> None:
                self.camera_configuration = self._MockCameraConfig()

            class _MockCameraConfig:
                def __init__(self) -> None:
                    self.resolution = Resolution()

        def open(self, init_params: "InitParameters") -> int:
            self._is_open = True
            return ERROR_CODE.SUCCESS

        def grab(self, runtime_params: RuntimeParameters) -> int:
            return ERROR_CODE.SUCCESS

        def retrieve_image(self, mat: Mat, view: str) -> int:
            return ERROR_CODE.SUCCESS

        def retrieve_measure(self, mat: Mat, measure: str) -> int:
            return ERROR_CODE.SUCCESS

        def retrieve_objects(
            self, objects: "Objects", params: "ObjectDetectionRuntimeParameters"
        ) -> int:
            return ERROR_CODE.SUCCESS

        def enable_object_detection(self, params: "ObjectDetectionParameters") -> int:
            return ERROR_CODE.SUCCESS

        def enable_positional_tracking(
            self, params: "PositionalTrackingParameters"
        ) -> int:
            return ERROR_CODE.SUCCESS

        def start_publishing(self, params: "CommunicationParameters") -> None:
            pass

        def get_camera_information(self) -> "_MockCameraInfo":
            return self._camera_info

        def disable_object_detection(self) -> int:
            """Disable object detection"""
            return ERROR_CODE.SUCCESS

        def close(self) -> int:
            """Close the camera"""
            self._is_open = False
            return ERROR_CODE.SUCCESS

    class Objects:
        """Mock Objects for testing"""

        def __init__(self) -> None:
            self.object_list: list[ObjectData] = []
            self.is_new = True  # Add missing is_new attribute

    class InitParameters:
        """Mock InitParameters for testing"""

        def __init__(
            self,
            depth_mode: str = DEPTH_MODE.ULTRA,
            coordinate_units: str = UNIT.METER,
            coordinate_system: str = COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD,
            camera_image_flip: str = FLIP_MODE.ON,
            depth_maximum_distance: float = 20.0,
            depth_minimum_distance: float = 0.3,
            async_image_retrieval: bool = False,  # Add missing parameter
        ) -> None:
            self.depth_mode = depth_mode
            self.coordinate_units = coordinate_units
            self.coordinate_system = coordinate_system
            self.camera_image_flip = camera_image_flip
            self.depth_maximum_distance = depth_maximum_distance
            self.depth_minimum_distance = depth_minimum_distance
            self.async_image_retrieval = async_image_retrieval

        def set_from_serial_number(self, serial_number: int) -> None:
            """Set parameters from serial number"""
            # Mock implementation - just store the serial number
            self.serial_number = serial_number

    class BatchParameters:
        """Mock BatchParameters for testing"""

        def __init__(self, enable: bool = True) -> None:
            self.enable = enable

    class ObjectDetectionParameters:
        """Mock ObjectDetectionParameters for testing"""

        def __init__(
            self,
            batch_trajectories_parameters: Optional[BatchParameters] = None,
            detection_model: str = OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST,
            enable_tracking: bool = True,
        ) -> None:
            self.batch_trajectories_parameters = batch_trajectories_parameters
            self.detection_model = detection_model
            self.enable_tracking = enable_tracking

    class ObjectDetectionRuntimeParameters:
        """Mock ObjectDetectionRuntimeParameters for testing"""

        def __init__(self, detection_confidence_threshold: int = 60) -> None:
            self.detection_confidence_threshold = detection_confidence_threshold

    class PositionalTrackingParameters:
        """Mock PositionalTrackingParameters for testing"""

        def __init__(self) -> None:
            self.mode = POSITIONAL_TRACKING_MODE.GEN_1
            self.enable_imu_fusion = False

    class CommunicationParameters:
        """Mock CommunicationParameters for testing"""

        def __init__(self) -> None:
            pass

    class CameraIdentifier:
        """Mock CameraIdentifier for testing"""

        def __init__(self, serial_number: int) -> None:
            self.serial_number = serial_number

    class Transform:
        """Mock Transform for testing"""

        def __init__(self, x: float, y: float, z: float) -> None:
            self.x = x
            self.y = y
            self.z = z

    class Fusion:
        """Mock Fusion for testing"""

        def __init__(self) -> None:
            pass

        def init(self, params: "InitFusionParameters") -> int:
            return FUSION_ERROR_CODE.SUCCESS

        def subscribe(
            self,
            uuid: CameraIdentifier,
            params: CommunicationParameters,
            transform: Transform,
        ) -> None:
            pass

        def enable_positionnal_tracking(
            self, params: "PositionalTrackingFusionParameters"
        ) -> int:
            return FUSION_ERROR_CODE.SUCCESS

    class InitFusionParameters:
        """Mock InitFusionParameters for testing"""

        def __init__(self, coordinate_units: str = UNIT.METER) -> None:
            self.coordinate_units = coordinate_units

    class GNSSCalibrationParameters:
        """Mock GNSSCalibrationParameters for testing"""

        def __init__(
            self,
            lever_arm: Optional[Any] = None,
            antenna_position: Optional[Any] = None,
            target_yaw_uncertainty: float = 0.1,
            enable_translation_uncertainty_target: bool = False,
            target_translation_uncertainty: float = 0.15,
            enable_reinitialization: bool = True,
            gnss_vio_reinit_threshold: int = 5,
        ) -> None:
            self.lever_arm = lever_arm
            self.antenna_position = antenna_position
            self.target_yaw_uncertainty = target_yaw_uncertainty
            self.enable_translation_uncertainty_target = (
                enable_translation_uncertainty_target
            )
            self.target_translation_uncertainty = target_translation_uncertainty
            self.enable_reinitialization = enable_reinitialization
            self.gnss_vio_reinit_threshold = gnss_vio_reinit_threshold

    class PositionalTrackingFusionParameters:
        """Mock PositionalTrackingFusionParameters for testing"""

        def __init__(
            self,
            enable_gnss_fusion: bool = False,
            enable_GNSS_fusion: bool = False,  # noqa: N803 - matches API
            gnss_calibration_parameters: Optional[GNSSCalibrationParameters] = None,
        ) -> None:
            # Support both naming conventions (the real API might use either)
            self.enable_gnss_fusion = enable_gnss_fusion or enable_GNSS_fusion
            self.enable_GNSS_fusion = enable_gnss_fusion or enable_GNSS_fusion
            self.gnss_calibration_parameters = gnss_calibration_parameters
