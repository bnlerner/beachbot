from __future__ import annotations

import dataclasses
import functools
import math
from typing import List, Literal, Tuple, Type, TypeVar, Union, overload

import numpy as np

from geometry import frames, math_helpers

BaseAngleTypeT = TypeVar("BaseAngleTypeT", bound="BaseAngleType")
BaseVectorTypeT = TypeVar("BaseVectorTypeT", bound="BaseVectorType")


@dataclasses.dataclass
class BaseAngleType:
    """An angle type that is compose of RPY in degrees."""

    frame: frames.ReferenceFrame
    roll: float
    pitch: float
    yaw: float

    def __post_init__(self) -> None:
        # Ensures that values are floats.
        self.roll = float(self.roll)
        self.pitch = float(self.pitch)
        self.yaw = float(self.yaw)

    @property
    def data(self) -> Tuple[float, float, float]:
        return self.roll, self.pitch, self.yaw

    def as_matrix(self) -> np.ndarray:
        """Returns this object as a numpy matrix, an extrinsic matrix by default."""
        return math_helpers.extrinsic_xyz_rotation_matrix(
            math.radians(self.roll), math.radians(self.pitch), math.radians(self.yaw)
        )

    def is_close(
        self: BaseAngleTypeT,
        other: BaseAngleTypeT,
        *,
        atol: float = math_helpers.DEFAULT_ATOL,
    ) -> bool:
        """Compares two Angle type objects and if they are relatively close to one
        another. A good way to account for different float quirks.
        """
        _raise_if_frames_different(self, other)
        return all(
            abs(d2 - d1) < atol for d1, d2 in zip(self.data, other.data, strict=True)
        )

    def rotated(
        self: BaseAngleTypeT, rotation: Rotation, *, intrinsic: bool = True
    ) -> BaseAngleTypeT:
        """This object rotated about a standard rotation matrix either intrinsically or
        extrinsically.
        """
        _raise_if_frames_different(self, rotation)
        if intrinsic:
            rotated_matrix = np.dot(self.as_matrix(), rotation.as_matrix())
        else:
            rotated_matrix = np.dot(rotation.as_matrix(), self.as_matrix())

        return self.from_matrix(self.frame, rotated_matrix)

    def to_2d(self: BaseAngleTypeT) -> BaseAngleTypeT:
        return self.__class__(self.frame, 0.0, 0.0, self.yaw)

    @classmethod
    def zero(cls: Type[BaseAngleTypeT], frame: frames.ReferenceFrame) -> BaseAngleTypeT:
        return cls(frame, 0, 0, 0)

    @classmethod
    def from_matrix(
        cls: Type[BaseAngleTypeT], frame: frames.ReferenceFrame, rot_matrix: np.ndarray
    ) -> BaseAngleTypeT:
        array = math_helpers.as_euler_xyz(rot_matrix)
        return cls(
            frame,
            math.degrees(array[0]),
            math.degrees(array[1]),
            math.degrees(array[2]),
        )

    def sin(self, attribute: Literal["roll", "pitch", "yaw"]) -> float:
        return math.sin(math.radians(getattr(self, attribute)))

    def cos(self, attribute: Literal["roll", "pitch", "yaw"]) -> float:
        return math.cos(math.radians(getattr(self, attribute)))

    def tan(self, attribute: Literal["roll", "pitch", "yaw"]) -> float:
        return math.tan(math.radians(getattr(self, attribute)))

    def __repr__(self) -> str:
        num_str = f"roll={self.roll:0.6g}, pitch={self.pitch:0.6g}, yaw={self.yaw:0.6g}"
        return f"{self.__class__.__name__}({self.frame.value!r}, {num_str})"


@dataclasses.dataclass
class BaseVectorType:
    frame: frames.ReferenceFrame
    x: float
    y: float
    z: float = 0.0

    def __post_init__(self) -> None:
        # Ensures that values are floats.
        self.x = float(self.x)
        self.y = float(self.y)
        self.z = float(self.z)

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def as_direction(self) -> Direction:
        return Direction(self.frame, self.x, self.y, self.z)

    def as_position(self) -> Position:
        return Position(self.frame, self.x, self.y, self.z)

    @property
    def magnitude(self) -> float:
        return (self.x**2 + self.y**2 + self.z**2) ** 0.5

    @property
    def data(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.z

    def rotated(
        self: BaseVectorTypeT, rotation: Rotation, *, intrinsic: bool = True
    ) -> BaseVectorTypeT:
        """This object rotated about a standard rotation matrix either intrinsically or
        extrinsically.
        """
        _raise_if_frames_different(self, rotation)
        if intrinsic:
            rot_vec = np.dot(self.as_array(), rotation.as_matrix())
        else:
            rot_vec = np.dot(rotation.as_matrix(), self.as_array())

        return self.from_array(self.frame, rot_vec)

    def to_2d(self: BaseVectorTypeT) -> BaseVectorTypeT:
        return self.__class__(self.frame, self.x, self.y, 0.0)

    def angle_to(self: BaseVectorTypeT, other: BaseVectorTypeT) -> float:
        _raise_if_frames_different(self, other)
        self_vec = self.as_array() / self.magnitude
        other_vec = other.as_array() / other.magnitude
        dot_product = np.dot(self_vec, other_vec)
        rad_angle = math.acos(dot_product)
        return math.degrees(rad_angle)

    def is_close(
        self: BaseVectorTypeT,
        other: BaseVectorTypeT,
        *,
        atol: float = math_helpers.DEFAULT_ATOL,
    ) -> bool:
        """Compares two vector type objects and if they are relatively close to one
        another. A good way to account for different float quirks.
        """
        _raise_if_frames_different(self, other)
        return all(
            abs(d2 - d1) < atol for d1, d2 in zip(self.data, other.data, strict=True)
        )

    @classmethod
    def from_array(
        cls: Type[BaseVectorTypeT], frame: frames.ReferenceFrame, array: np.ndarray
    ) -> BaseVectorTypeT:
        return cls(frame, float(array[0]), float(array[1]), float(array[2]))

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

    def sin(self, attribute: Literal["x", "y", "z"]) -> float:
        return math.sin(math.radians(getattr(self, attribute)))

    def cos(self, attribute: Literal["x", "y", "z"]) -> float:
        return math.cos(math.radians(getattr(self, attribute)))

    def tan(self, attribute: Literal["x", "y", "z"]) -> float:
        return math.tan(math.radians(getattr(self, attribute)))

    def __add__(self: BaseVectorTypeT, other: BaseVectorTypeT) -> BaseVectorTypeT:
        _raise_if_frames_different(self, other)
        return self.__class__(
            self.frame, self.x + other.x, self.y + other.y, self.z + other.z
        )

    def __sub__(self: BaseVectorTypeT, other: BaseVectorTypeT) -> BaseVectorTypeT:
        _raise_if_frames_different(self, other)
        return self.__class__(
            self.frame, self.x - other.x, self.y - other.y, self.z - other.z
        )

    def __mul__(self: BaseVectorTypeT, scalar: float) -> BaseVectorTypeT:
        scaled_x, scaled_y, scaled_z = self.x * scalar, self.y * scalar, self.z * scalar
        return self.__class__(self.frame, scaled_x, scaled_y, scaled_z)

    def __truediv__(self: BaseVectorTypeT, scalar: float) -> BaseVectorTypeT:
        scaled_x, scaled_y, scaled_z = self.x / scalar, self.y / scalar, self.z / scalar
        return self.__class__(self.frame, scaled_x, scaled_y, scaled_z)

    def __repr__(self) -> str:
        num_str = f"x={self.x:0.6g}, y={self.y:0.6g}, z={self.z:0.6g}"
        return f"{self.__class__.__name__}({self.frame.value!r}, {num_str})"


class Orientation(BaseAngleType):
    """An orientation specified in RPY with angles in degrees. Naively considers inputs
    in degrees and doesnt double check you, so be aware...
    """

    @classmethod
    def from_intrinsic_rpy(
        cls, frame: frames.ReferenceFrame, roll: float, pitch: float, yaw: float
    ) -> Orientation:
        """The instrinsic orienation considering RPY as degrees."""
        rot_matrix = math_helpers.intrinsic_xyz_rotation_matrix(
            math.radians(roll), math.radians(pitch), math.radians(yaw)
        )
        return Orientation.from_matrix(frame, rot_matrix)

    @classmethod
    def from_quaternion(
        cls, frame: frames.ReferenceFrame, w: float, x: float, y: float, z: float
    ) -> Orientation:
        """Create an Orientation from a quaternion where w is the scalar part of the
        quaternion and x, y, z are the vector part of the quaternion.
        """
        roll, pitch, yaw = math_helpers.quaternion_to_euler(w, x, y, z)

        return cls(frame=frame, roll=roll, pitch=pitch, yaw=yaw)

    def as_quaternion(self) -> Tuple[float, float, float, float]:
        """Returns the quaternion representation of the orientation."""
        w, x, y, z = math_helpers.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        return w, x, y, z

    def as_rotation(self) -> Rotation:
        return Rotation(self.frame, self.roll, self.pitch, self.yaw)

    def x_axis(self) -> Direction:
        rot_mat = self.as_matrix()
        x_axis_vec = rot_mat[:, 0]
        return Direction.from_array(self.frame, x_axis_vec)

    def y_axis(self) -> Direction:
        rot_mat = self.as_matrix()
        y_axis_vec = rot_mat[:, 1]
        return Direction.from_array(self.frame, y_axis_vec)

    def z_axis(self) -> Direction:
        rot_mat = self.as_matrix()
        z_axis_vec = rot_mat[:, 2]
        return Direction.from_array(self.frame, z_axis_vec)


class Rotation(BaseAngleType):
    """A rotatation matrix expressed in RPY with angles in degrees."""

    @classmethod
    def from_intrinsic_rpy(
        cls, frame: frames.ReferenceFrame, roll: float, pitch: float, yaw: float
    ) -> Rotation:
        rot_matrix = math_helpers.intrinsic_xyz_rotation_matrix(
            math.radians(roll), math.radians(pitch), math.radians(yaw)
        )
        return Rotation.from_matrix(frame, rot_matrix)

    def as_orientation(self) -> Orientation:
        return Orientation(self.frame, self.roll, self.pitch, self.yaw)

    def inverted(self) -> Rotation:
        return Rotation.from_matrix(
            self.frame, math_helpers.transpose(self.as_matrix())
        )


class Position(BaseVectorType):
    """A position in cartesian space represented in meters."""

    def distance(self, other: Position) -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return (dx**2 + dy**2 + dz**2) ** 0.5


class Velocity(BaseVectorType):
    """A velocity in cartesian space represented in meters/second"""

    @classmethod
    def from_direction(cls, direction: Direction, magnitude: float) -> Velocity:
        vel_vec = direction.as_array() * magnitude
        return Velocity.from_array(direction.frame, vel_vec)


class Direction(BaseVectorType):
    """A direction in cartesian space represented in its xyz coordinates. Considered a
    unit vector.
    """

    def __post_init__(self) -> None:
        """Ensures the direction is a unit vector."""
        self.x, self.y, self.z = self.as_array() / self.magnitude

    @classmethod
    def from_celestial(
        cls, frame: frames.ReferenceFrame, altitude: float, azimuth: float
    ) -> Direction:
        """Takes the altitude and azimuth in degreses and constructs a direction."""
        array = math_helpers.celestial_coordinates(
            math.radians(altitude), math.radians(azimuth)
        )
        return cls.from_array(frame, array)

    def azimuth(self) -> float:
        """The azimuth of this vector in the celestial coordinate frame (degrees)."""
        return math.degrees(math.atan2(self.y, self.x))

    def elevation(self) -> float:
        """The elevation of this vector in the celestial coordinate frame (degrees)."""
        base_len = (self.x**2 + self.y**2) ** 0.5
        return math.degrees(math.atan2(self.z, base_len))


class AngularVelocity(BaseVectorType):
    """An angular velocity in cartesian space represented in degrees/second. For
    example, the value of x represents the magnitude of rotation (ω) about the x-axis.
    """

    def to_2d(self) -> AngularVelocity:
        return AngularVelocity(self.frame, 0, 0, self.z)

    def speed(self) -> float:
        """Speed the angular velocity is rotating about the axis of rotation in deg/s.
        Speed is always positive.
        """
        return math_helpers.norm_3d_vector(self.as_array())

    def axis(self) -> Direction:
        """Axis of rotation."""
        array = self.as_array()
        unit_array = array / math_helpers.norm_3d_vector(array)
        return Direction.from_array(self.frame, unit_array)


class AngularAcceleration(BaseVectorType):
    """An angular acceleration in cartesian space represented in degrees/second^2"""

    def to_2d(self) -> AngularAcceleration:
        return AngularAcceleration(self.frame, 0, 0, self.z)


class Acceleration(BaseVectorType):
    """An acceleration in cartesian space represented in meters/second^2"""

    def to_2d(self) -> Acceleration:
        return Acceleration(self.frame, self.x, self.y, 0)


########################################################################################
# Objects containing other cartesian objects ###########################################
########################################################################################


@dataclasses.dataclass
class Pose:
    """A pose in 3D cartesian space with a position and orientation."""

    position: Position
    orientation: Orientation

    def __post_init__(self) -> None:
        _raise_if_frames_different(self.position, self.orientation)

    @functools.cached_property
    def rotation(self) -> Rotation:
        """The orientation of the pose represented as a rotation."""
        return self.orientation.as_rotation()

    @functools.cached_property
    def inverted_rotation(self) -> Rotation:
        """Inverse rotation of the associated orientation."""
        return self.rotation.inverted()

    def to_2d(self) -> Pose:
        return Pose(self.position.to_2d(), self.orientation.to_2d())

    @overload
    def to_local(self, other: Pose) -> Pose:
        ...

    @overload
    def to_local(self, other: Position) -> Position:
        ...

    @overload
    def to_local(self, other: Orientation) -> Orientation:
        ...

    def to_local(
        self, other: Union[Pose, Position, Orientation]
    ) -> Union[Pose, Position, Orientation]:
        """Represents the other object into this Pose's own local frame."""
        if isinstance(other, Pose):
            return Pose(
                self._pos_in_local(other.position),
                self._ori_in_local(other.orientation),
            )
        elif isinstance(other, Position):
            return self._pos_in_local(other)
        elif isinstance(other, Orientation):
            return self._ori_in_local(other)
        else:
            raise NotImplementedError(f"Transform not implemented for {type(other)}")

    def _ori_in_local(self, orientation: Orientation) -> Orientation:
        return self.inverted_rotation.rotated(
            orientation.as_rotation()
        ).as_orientation()

    def _pos_in_local(self, position: Position) -> Position:
        return (position - self.position).rotated(
            self.inverted_rotation, intrinsic=False
        )

    def from_local(self, x: float, y: float, z: float) -> Position:
        """Outputs a position relative to the local pose's frame."""
        local_pos = Position(self.position.frame, x, y, z)
        rot = self.orientation.as_rotation()
        pos_delta = local_pos.rotated(rot, intrinsic=False)
        return self.position + pos_delta

    def is_close(self, other: Pose, *, atol: float = math_helpers.DEFAULT_ATOL) -> bool:
        return self.position.is_close(
            other.position, atol=atol
        ) and self.orientation.is_close(other.orientation, atol=atol)

    def update_frame(self, frame: frames.ReferenceFrame) -> None:
        """Changes this object's frame."""
        self.position.frame = frame
        self.orientation.frame = frame

    @classmethod
    def zero(cls, frame: frames.ReferenceFrame) -> Pose:
        return Pose(Position.zero(frame), Orientation.zero(frame))


@dataclasses.dataclass
class Twist:
    """A representation of the velocity and angular velocity of an object."""

    velocity: Velocity
    spin: AngularVelocity

    def __post_init__(self) -> None:
        _raise_if_frames_different(self.velocity, self.spin)

    def __add__(self, other: Twist) -> Twist:
        return Twist(self.velocity + other.velocity, self.spin + other.spin)

    def __truediv__(self, scalar: float) -> Twist:
        return Twist(self.velocity / scalar, self.spin / scalar)

    def to_2d(self) -> Twist:
        return Twist(self.velocity.to_2d(), self.spin.to_2d())

    def is_close(
        self, other: Twist, *, atol: float = math_helpers.DEFAULT_ATOL
    ) -> bool:
        return self.velocity.is_close(other.velocity, atol=atol) and self.spin.is_close(
            other.spin, atol=atol
        )

    def update_frame(self, frame: frames.ReferenceFrame) -> None:
        """Changes this object's frame."""
        self.velocity.frame = frame
        self.spin.frame = frame

    @classmethod
    def zero(cls, frame: frames.ReferenceFrame) -> Twist:
        return Twist(Velocity.zero(frame), AngularVelocity.zero(frame))


def _raise_if_frames_different(*objs: Union[BaseAngleType, BaseVectorType]) -> None:
    obj_frames = set(obj.frame for obj in objs)
    if len(obj_frames) > 1:
        raise ValueError(f"Multiple frames found: {obj_frames=}")


@dataclasses.dataclass
class Polygon:
    """An XY polygon made up of at least 3 points."""

    points: List[Position]

    def __post_init__(self) -> None:
        _raise_if_frames_different(*self.points)
        self._raise_if_not_enough_points()

    @property
    def frame(self) -> frames.ReferenceFrame:
        return self.points[0].frame

    @functools.cached_property
    def array(self) -> np.ndarray:
        return np.array([pt.data[:2] for pt in self.points])

    def contains(self, point: Position) -> frames.ReferenceFrame:
        """Whether the point is contained within the polygon. Points on the Polygon line
        are considered contained inside of it.
        """
        pt_2d = point.data[:2]
        return math_helpers.polygon_contains(pt_2d, self.array)

    def _raise_if_not_enough_points(self) -> None:
        if len(self.points) < 3:
            raise ValueError(f"Not enough points to make a Polygon {self.points=}")


@dataclasses.dataclass
class Plane:
    ...
