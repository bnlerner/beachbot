import dataclasses
import enum
from typing import Optional

import geometry
import numpy as np


class JointType(enum.Enum):
    """Types of joints in a kinematic chain. Fixed has no movement relative to its
    parent. Rotary rotates about a single axis.
    """

    FIXED = 0
    ROTARY = 1


@dataclasses.dataclass
class LinkParameters:
    """Parameters that define a link in a kinematic chain using Denavit-Hartenberg
    parameters.
    """

    length: float  # a in DH convention
    twist: float  # alpha in DH convention
    offset: float  # d in DH convention
    joint_type: JointType


class Link:
    """Represents a link in a kinematic chain. A link connects two joints and is defined
    by its DH parameters and joint configuration.
    """

    def __init__(
        self,
        parameters: LinkParameters,
        joint_type: JointType,
        frame: geometry.ReferenceFrame,
        axis: np.ndarray = np.array([0, 0, 1]),  # Default joint axis is z-axis
    ) -> None:
        """
        Initialize a link with given parameters and joint.

        Args:
            parameters: The Denavit-Hartenberg parameters of the link.
            joint: The joint at the start of the link.
            frame: The reference frame in which the link is defined.
            axis: The axis of rotation/translation for the joint.
        """
        self.parameters = parameters
        self.joint_type = joint_type
        self.frame = frame
        self.axis = axis / np.linalg.norm(axis)  # Normalize the axis

    def transformation_matrix(self) -> np.ndarray:
        """
        Calculate the homogeneous transformation matrix for this link.

        Returns:
            A 4x4 transformation matrix representing the position and orientation
            of the end of this link relative to its start.
        """
        # Get the joint value based on joint type
        theta = 0.0
        d = self.parameters.offset

        if self.parameters.joint_type == JointType.ROTARY:
            theta = self.joint_type.value

        # Calculate the DH transformation matrix
        return self._dh_transformation_matrix(
            theta, d, self.parameters.length, self.parameters.twist
        )

    def _dh_transformation_matrix(
        self, theta: float, d: float, a: float, alpha: float
    ) -> np.ndarray:
        """
        Calculate the Denavit-Hartenberg transformation matrix.

        This implements the standard DH transformation:
        1. Rotate around z-axis by theta
        2. Translate along z-axis by d
        3. Translate along x-axis by a
        4. Rotate around x-axis by alpha

        Args:
            theta: Joint angle (for rotary joints) in radians.
            d: Link offset (for prismatic joints) in meters.
            a: Link length in meters.
            alpha: Link twist in radians.

        Returns:
            A 4x4 homogeneous transformation matrix.
        """
        # Compute trigonometric values
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)

        # Create the transformation matrix
        transform = np.array(
            [
                [
                    cos_theta,
                    -sin_theta * cos_alpha,
                    sin_theta * sin_alpha,
                    a * cos_theta,
                ],
                [
                    sin_theta,
                    cos_theta * cos_alpha,
                    -cos_theta * sin_alpha,
                    a * sin_theta,
                ],
                [0, sin_alpha, cos_alpha, d],
                [0, 0, 0, 1],
            ]
        )

        return transform


@dataclasses.dataclass
class KinematicState:
    """
    Represents the kinematic state of a link in the chain.

    Attributes:
        pose: Position and orientation
        velocity: Linear velocity
        angular_velocity: Angular velocity
        acceleration: Linear acceleration
        angular_acceleration: Angular acceleration
    """

    pose: geometry.Pose
    velocity: geometry.Velocity
    angular_velocity: geometry.AngularVelocity
    acceleration: Optional[geometry.Acceleration] = None
    angular_acceleration: Optional[geometry.AngularAcceleration] = None


@dataclasses.dataclass
class MultiEndEffectorConfig:
    """
    Configuration for multiple end effectors on a kinematic chain.

    Attributes:
        name: Name identifier for the end effector
        link_index: Index of the link to which this end effector is attached
        offset_pose: Pose offset from the link's end to the actual end effector position
    """

    name: str
    link_index: int
    offset_pose: Optional[geometry.Pose] = None

    def __post_init__(self) -> None:
        if self.offset_pose is None:
            # Default to zero offset at the link's frame
            frame = geometry.BODY  # Default frame
            self.offset_pose = geometry.Pose.zero(frame)
