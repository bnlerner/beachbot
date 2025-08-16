from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import geometry
import numpy as np

from kinematics import primitives


@dataclass
class BaseFrameNode:
    """Base class for frame nodes representing different joint types.

    This module provides specialized frame node classes for different joint types.
    Each class is tailored to the specific constraints and behaviors of a particular
    joint type, making it easier to enforce constraints and update state correctly.
    """

    frame: geometry.ReferenceFrame
    origin: geometry.Position
    orientation: geometry.Orientation
    joint_type: primitives.JointType

    rotary_axis: Optional[geometry.Direction] = field(default=None)
    velocity: Optional[geometry.Velocity] = field(default=None)
    acceleration: Optional[geometry.Acceleration] = field(default=None)
    angular_velocity: Optional[geometry.AngularVelocity] = field(default=None)
    angular_acceleration: Optional[geometry.AngularAcceleration] = field(default=None)

    parent: Optional[BaseFrameNode] = field(default=None)
    children: List[BaseFrameNode] = field(default_factory=list, init=False)

    @property
    def pose(self) -> geometry.Pose:
        return geometry.Pose(self.origin, self.orientation)

    def get_transformation_matrix(self) -> np.ndarray:
        """Get the 4x4 homogeneous transformation matrix from parent to this frame."""
        raise NotImplementedError("Subclasses must implement this method")

    def get_inverse_transformation_matrix(self) -> np.ndarray:
        """Get the 4x4 homogeneous transformation matrix from this frame to parent."""
        return np.linalg.inv(self.get_transformation_matrix())

    def add_child(self, child: BaseFrameNode) -> None:
        if self.children is None:
            self.children = []

        self.children.append(child)

    def _raise_if_frame_mismatch(self, other: geometry.ReferenceFrame) -> None:
        if self.frame != other:
            raise ValueError(f"Frame {other} doesn't match node {self.frame=}")


@dataclass
class FixedFrameNode(BaseFrameNode):
    """Frame node for fixed joints. Fixed joints have a rigid connection to their parent
    frame and do not allow any relative motion.
    """

    joint_type: primitives.JointType = field(
        default=primitives.JointType.FIXED, init=False
    )

    def get_transformation_matrix(self) -> np.ndarray:
        """Get the 4x4 homogeneous transformation matrix from parent to this frame."""
        return self.pose.as_transform_matrix()


@dataclass
class RotaryFrameNode(BaseFrameNode):
    """Frame node for rotary joints. Rotary joints allow rotation around a specific
    axis.
    """

    rotary_axis: geometry.Direction
    velocity: geometry.Velocity
    acceleration: geometry.Acceleration

    limits: Tuple[float, float] = (-float("inf"), float("inf"))
    angle: float = 0.0

    joint_type: primitives.JointType = field(
        default=primitives.JointType.ROTARY, init=False
    )

    def get_transformation_matrix(self) -> np.ndarray:
        """Get the 4x4 homogeneous transformation matrix from parent to this frame."""
        return geometry.create_rotary_joint_transform(
            *self.origin.data,
            *self.orientation.data,
            *self.rotary_axis.data,
            self.angle,
        )

    def set_angular_velocity_from_joint_velocity(self, joint_velocity: float) -> None:
        """Set the angular velocity based on joint velocity.

        This is a more physically correct way to update the angular velocity
        for a rotary joint, ensuring it is aligned with the joint axis.

        Args:
            joint_velocity: Angular velocity in radians per second
        """
        # Create an angular velocity along the joint axis
        self.angular_velocity = geometry.AngularVelocity.from_array(
            self.frame, self.rotary_axis.as_array() * joint_velocity
        )

    def set_angular_acceleration_from_joint_acceleration(
        self, joint_acceleration: float
    ) -> None:
        """Set the angular acceleration based on joint acceleration.

        This is a more physically correct way to update the angular acceleration
        for a rotary joint, ensuring it is aligned with the joint axis.

        Args:
            joint_acceleration: Angular acceleration in radians per second squared
        """
        # Create an angular acceleration along the joint axis
        self.angular_acceleration = geometry.AngularAcceleration.from_array(
            self.frame, self.rotary_axis.as_array() * joint_acceleration
        )
