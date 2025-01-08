from __future__ import annotations

import enum
from typing import List, Optional, Tuple

import cv2
import geometry
import numpy as np
import pydantic
from pyzed import sl  # type:ignore[import-untyped]
from scipy.ndimage import distance_transform_edt  # type:ignore[import-untyped]

_IMAGE_SCALE_PCT = 0.25


class TrackedObjects(pydantic.BaseModel):
    """An object the camera is tracking as well as relevant telemetry data on it."""

    id: int
    label: ObjectType
    position: geometry.Position
    velocity: geometry.Velocity
    image_bounding_box: Tuple[Tuple[int, int], Tuple[int, int]]
    bounding_box: List[geometry.Position]
    width: float
    height: float
    length: float
    confidence: float

    @classmethod
    def from_zed_object(
        cls, frame: geometry.ReferenceFrame, obj: sl.ObjectData
    ) -> TrackedObjects:
        width, height, length = obj.dimensions
        bbox = obj.bounding_box_2d
        return TrackedObjects(
            id=obj.id,
            label=ObjectType.from_zed_object(obj.label),
            position=geometry.Position.from_array(frame, obj.position),
            velocity=geometry.Velocity.from_array(frame, obj.velocity),
            image_bounding_box=((bbox[0][0], bbox[0][1]), (bbox[2][0], bbox[2][1])),
            bounding_box=[
                geometry.Position.from_array(frame, pos_arr)
                for pos_arr in obj.bounding_box
            ],
            width=width,
            height=height,
            length=length,
            confidence=obj.confidence,
        )

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self.label}, {self.position=}, {self.confidence=:0.6g})"


class ObjectType(enum.Enum):
    # For people detection
    PERSON = "PERSON"
    # For vehicle detection (cars, trucks, buses, motorcycles, etc.)
    VEHICLE = "VEHICLE"
    # For bag detection (backpack, handbag, suitcase, etc.)
    BAG = "BAG"
    # For animal detection (cow, sheep, horse, dog, cat, bird, etc.)
    ANIMAL = "ANIMAL"
    # For electronic device detection (cellphone, laptop, etc.)
    ELECTRONICS = "ELECTRONICS"
    # For fruit and vegetable detection (banana, apple, orange, carrot, etc.)
    FRUIT_VEGETABLE = "FRUIT_VEGETABLE"
    # For sport-related object detection (sport ball, etc.)
    SPORT = "SPORT"

    @classmethod
    def from_zed_object(cls, obj: sl.OBJECT_CLASS) -> ObjectType:
        if obj == sl.OBJECT_CLASS.PERSON:
            return ObjectType("PERSON")
        elif obj == sl.OBJECT_CLASS.VEHICLE:
            return ObjectType("VEHICLE")
        elif obj == sl.OBJECT_CLASS.BAG:
            return ObjectType("BAG")
        elif obj == sl.OBJECT_CLASS.ANIMAL:
            return ObjectType("ANIMAL")
        elif obj == sl.OBJECT_CLASS.ELECTRONICS:
            return ObjectType("ELECTRONICS")
        elif obj == sl.OBJECT_CLASS.FRUIT_VEGETABLE:
            return ObjectType("FRUIT_VEGETABLE")
        elif obj == sl.OBJECT_CLASS.SPORT:
            return ObjectType("SPORT")
        else:
            raise ValueError(f"Unknown object {obj}, type: {type(obj)}")


class DepthMap:
    """
    In STANDARD mode, holes (black pixels) have different values depending on their type. They are associated with an enum:

    NAN referred as OCCLUSION_VALUE. The depth of the pixel cannot be estimated as it is occluded or an outlier.
    -INFINITY referred as TOO_CLOSE. The depth of the pixel cannot be estimated as it is too close to the camera.
    INFINITY referred as TOO_FAR. The depth of the pixel cannot be estimated as it is too far from the camera.
    You can check for valid depth values using isValidMeasure().

    """

    grid_size = 0.1  # 10 cm per cell
    width = 10
    height = 10  # 10m x 10m grid

    def __init__(self, array: np.ndarray):
        self._array = array

    @property
    def array(self) -> np.ndarray:
        return self._array

    def filtered(self) -> np.ndarray:
        return np.where((self._array > 0.5) & (self._array < 5.0), self._array, 0)

    def min(self) -> float:
        return np.nanmin(self._array)

    def signed_distance_field(self) -> np.ndarray:
        obstacle_map = np.zeros(
            (int(self.width / self.grid_size), int(self.height / self.grid_size))
        )
        filtered_depth = self.filtered()
        for x in range(filtered_depth.shape[0]):
            for y in range(filtered_depth.shape[1]):
                depth = filtered_depth[x, y]
                if depth > 0:  # Obstacle detected
                    grid_x = int(x * self.grid_size)
                    grid_y = int(y * self.grid_size)
                    obstacle_map[grid_x, grid_y] = 1

        # Compute distance to nearest obstacle for free space
        free_space_distance = distance_transform_edt(obstacle_map == 0)

        # Compute distance to the nearest free space for obstacles
        obstacle_distance = distance_transform_edt(obstacle_map == 1)

        # Combine to create signed distance field
        return free_space_distance - obstacle_distance


class Image:
    """An image from the camera."""

    def __init__(self, array: np.ndarray):
        self._array = array

    @classmethod
    def from_zed_image(cls, zed_image: sl.Mat) -> Optional[Image]:
        """Creates an image from the zed image matrix object. If the image is empty
        returns None instead of an invalid Image.
        """
        array = zed_image.get_data()
        if array.size == 0:
            return None
        else:
            return Image(array)

    def serialized(self) -> Optional[bytes]:
        """A serialized version of this Image as a JPEG."""
        # NOTE: cv2 encoding is very slow so dont call too often if you can help it.
        _, buffer = cv2.imencode(
            ".jpg", self._array, [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        )
        return buffer.tobytes()

    def add_bounding_boxes(self, tracked_objects: List[TrackedObjects]) -> None:
        """Adds 2D bounding boxes to the image based on the tracked objects."""
        for obj in tracked_objects:
            pt0, pt1 = obj.image_bounding_box

            # Draw bounding box
            cv2.rectangle(self._array, pt0, pt1, (0, 255, 0), 2)

            # Add label with confidence score
            label = f"{obj.label.value} ({obj.confidence}%)"
            text_pt = (pt0[0], pt1[1] - 10)
            cv2.putText(
                self._array,
                label,
                text_pt,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

    def reduce(self) -> None:
        """Modifies this image, reducing its size to save space."""
        width = int(self._array.shape[1] * _IMAGE_SCALE_PCT)
        height = int(self._array.shape[0] * _IMAGE_SCALE_PCT)
        self._array = cv2.resize(
            self._array, (width, height), interpolation=cv2.INTER_AREA
        )
