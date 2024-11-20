from __future__ import annotations

import dataclasses
import math
from typing import Type, TypeVar, Union

import numpy as np

from geometry import frames
from python.geometry import math_helpers

BaseAngleTypeT = TypeVar("BaseAngleTypeT", bound="BaseAngleType")
BaseVectorTypeT = TypeVar("BaseVectorTypeT", bound="BaseVectorType")


@dataclasses.dataclass
class BaseAngleType:
    """In degrees"""

    frame: frames.ReferenceFrame
    roll: float
    pitch: float
    yaw: float

    def as_matrix(self) -> np.ndarray:
        """Implied this is an extrinsic matrix."""
        return math_helpers.extrinsic_xyz_rotation_matrix(
            self.roll, self.pitch, self.yaw
        )

    @classmethod
    def from_matrix(
        cls: Type[BaseAngleTypeT], frame: frames.ReferenceFrame, rot_matrix: np.ndarray
    ) -> BaseAngleTypeT:
        return cls(frame, *math_helpers.as_euler_xyz(rot_matrix))

    def rotated(
        self: BaseAngleTypeT, rotation: Rotation, *, intrinsic: bool = True
    ) -> BaseAngleTypeT:
        """A rotation of the orientation."""
        if intrinsic:
            rotated_matrix = math_helpers.dot(self.as_matrix(), rotation.as_matrix())
        else:
            rotated_matrix = math_helpers.dot(rotation.as_matrix(), self.as_matrix())

        return self.from_matrix(self.frame, rotated_matrix)

    def to_2d(self: BaseAngleTypeT) -> BaseAngleTypeT:
        return self.__class__(self.frame, 0.0, 0.0, self.yaw)

    @classmethod
    def zero(cls: Type[BaseAngleTypeT], frame: frames.ReferenceFrame) -> BaseAngleTypeT:
        return cls(frame, 0, 0, 0)


@dataclasses.dataclass
class BaseVectorType:
    frame: frames.ReferenceFrame
    x: float
    y: float
    z: float

    def as_vector(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def as_direction(self) -> Direction:
        return Direction(self.frame, self.x, self.y, self.z)

    @property
    def magnitude(self) -> float:
        return (self.x**2 + self.y**2 + self.z**2) ** 0.5

    def rotated(self: BaseVectorTypeT, rotation: Rotation) -> BaseVectorTypeT:
        """Extrinsic rotation."""
        rot_vec = math_helpers.dot(rotation.as_matrix(), self.as_vector())
        return self.__class__(self.frame, *rot_vec)

    def to_2d(self: BaseVectorTypeT) -> BaseVectorTypeT:
        return self.__class__(self.frame, self.x, self.y, 0.0)

    def angle_to(self: BaseVectorTypeT, other: BaseVectorTypeT) -> float:
        self_vec = self.as_vector()
        other_vec = other.as_vector()
        dot_product = math_helpers.dot(self_vec, other_vec)
        cross_product = math_helpers.cross_3d_vector(self_vec, other_vec)
        return math.acos(dot_product / cross_product)

    @classmethod
    def zero(
        cls: Type[BaseVectorTypeT], frame: frames.ReferenceFrame
    ) -> BaseVectorTypeT:
        return cls(frame, 0, 0, 0)

    @classmethod
    def unit_x(
        cls: Type[BaseVectorTypeT], frame: frames.ReferenceFrame
    ) -> BaseVectorTypeT:
        return cls(frame, 1, 0, 0)

    @classmethod
    def unit_y(
        cls: Type[BaseVectorTypeT], frame: frames.ReferenceFrame
    ) -> BaseVectorTypeT:
        return cls(frame, 0, 1, 0)

    @classmethod
    def unit_z(
        cls: Type[BaseVectorTypeT], frame: frames.ReferenceFrame
    ) -> BaseVectorTypeT:
        return cls(frame, 0, 0, 1)

    def __add__(self: BaseVectorTypeT, other: BaseVectorTypeT) -> BaseVectorTypeT:
        return self.__class__(
            self.frame, self.x + other.x, self.y + other.y, self.z + other.z
        )

    def __sub__(self: BaseVectorTypeT, other: BaseVectorTypeT) -> BaseVectorTypeT:
        return self.__class__(
            self.frame, self.x - other.x, self.y - other.y, self.z - other.z
        )

    def __mul__(self: BaseVectorTypeT, scalar: float) -> BaseVectorTypeT:
        scaled_x, scaled_y, scaled_z = self.x * scalar, self.y * scalar, self.z * scalar
        return self.__class__(self.frame, scaled_x, scaled_y, scaled_z)


class Orientation(BaseAngleType):
    @classmethod
    def from_intrinsic_rpy(
        cls, frame: frames.ReferenceFrame, roll: float, pitch: float, yaw: float
    ) -> Orientation:
        rot_matrix = math_helpers.intrinsic_xyz_rotation_matrix(roll, pitch, yaw)
        return Orientation(frame, *math_helpers.as_euler_xyz(rot_matrix))

    def as_rotation(self) -> Rotation:
        return Rotation(self.frame, self.roll, self.pitch, self.yaw)

    def x_axis(self) -> Direction:
        rot_mat = self.as_matrix()
        x_axis_vec = rot_mat[:, 0]
        return Direction(self.frame, *x_axis_vec)

    def y_axis(self) -> Direction:
        rot_mat = self.as_matrix()
        y_axis_vec = rot_mat[:, 1]
        return Direction(self.frame, *y_axis_vec)

    def z_axis(self) -> Direction:
        rot_mat = self.as_matrix()
        z_axis_vec = rot_mat[:, 2]
        return Direction(self.frame, *z_axis_vec)


class Rotation(BaseAngleType):
    def as_orientation(self) -> Orientation:
        return Orientation(self.frame, self.roll, self.pitch, self.yaw)

    def inverted(self) -> Rotation:
        return Rotation.from_matrix(self.frame, math_helpers.transpose(self.as_matrix()))


class Position(BaseVectorType):
    """Position in meters."""

    def distance(self, other: Position) -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return (dx**2 + dy**2 + dz**2) ** 0.5


class Velocity(BaseVectorType):
    """meters/second"""

    @classmethod
    def from_direction(
        cls, frame: frames.ReferenceFrame, direction: Direction, magnitude: float
    ) -> Velocity:
        vel_vec = direction.as_vector() * magnitude
        return Velocity(frame, *vel_vec)


class Direction(BaseVectorType):
    """A direction vector."""

    def __post_init__(self) -> None:
        """Ensures the direction is a unit vector."""
        self.x, self.y, self.z = self.as_vector() / self.magnitude


class AngularVelocity(BaseVectorType):
    """A representation of angular velocity in degrees/second. For example, the value of
    x represents the magnitude of rotation (ω) about the x-axis.
    """

    def to_2d(self) -> AngularVelocity:
        return AngularVelocity(self.frame, 0, 0, self.z)


@dataclasses.dataclass
class Pose:
    position: Position
    orientation: Orientation

    def __post_init__(self) -> None:
        _raise_if_frames_different(self.position, self.orientation)

    def to_2d(self) -> Pose:
        return Pose(self.position.to_2d(), self.orientation.to_2d())

    def transform(self, other: Pose) -> Pose:
        """Transforms the other object into this Pose's frame."""
        rot_inv = self.orientation.as_rotation().inverted()
        ori_in_self = rot_inv.rotated(other.orientation.as_rotation()).as_orientation()
        pos_in_self = (other.position - self.position).rotated(rot_inv)
        return Pose(pos_in_self, ori_in_self)


@dataclasses.dataclass
class Twist:
    velocity: Velocity
    spin: AngularVelocity

    def to_2d(self) -> Twist:
        return Twist(self.velocity.to_2d(), self.spin.to_2d())

    def __post_init__(self) -> None:
        _raise_if_frames_different(self.velocity, self.spin)

    def __add__(self, other: Twist) -> Twist:
        return Twist(self.velocity + other.velocity, self.spin + other.spin)


def _raise_if_frames_different(*objs: Union[BaseAngleType, BaseVectorType]) -> None:
    if len(set(obj.frame for obj in objs)) > 1:
        raise ValueError(f"Expected only one frame for objects {objs=}")
