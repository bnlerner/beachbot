"""Factory for creating standard robot arm frame trees with different configurations.

This factory can create robot arms with varying degrees of freedom (DOF),
suitable for testing and demonstration purposes.

The fixture factory creates pre-configured robot arm frame trees that can be used for:
- Unit testing of frame tree and kinematics functionality
- Demonstrations and examples of robot kinematics
- Simulation and visualization of standard robot arm configurations
- Benchmarking and performance testing

The factory methods load robot descriptions from URDF files stored in the env/urdf
directory, providing standardized configurations with 3, 4, 5, or 6 degrees of freedom.
Each configuration represents a typical robot arm with revolute joints arranged in
common industrial robot configurations.

The factory pattern simplifies the creation of complex frame trees and ensures
consistency across different parts of the codebase when working with standard
robot configurations.

Typical usage:
    # Create a 5-DOF robot arm
    arm_tree = create_5dof_arm()

    # Use the tree for transformations
    robot_pos = cartesian_objects.Position(frame="end_effector", x=0.1, y=0, z=0)
    base_pos = arm_tree.transform_position(robot_pos, "base_link")
"""

import os
from typing import List

import geometry
import numpy as np

from kinematics import kinematic_tree, urdf_parser

# Path to the URDF files
_URDF_DIR = os.path.join("env", "urdf")


def gen_3dof_arm() -> kinematic_tree.KinematicTree:
    """Creats a 3-DOF robot arm frame tree with:
    - Base (fixed)
    - Shoulder joint (rotary) - rotates around Z
    - Elbow joint (rotary) - rotates around Y
    - Wrist joint (rotary) - rotates around Y
    - End effector (fixed)

    Returns:
        A KinematicTree representing a 3-DOF robot arm
    """
    urdf_path = os.path.join(_URDF_DIR, "3dof_arm.urdf")
    return urdf_parser.parse_kinematic_tree(urdf_path)


def gen_4dof_arm() -> kinematic_tree.KinematicTree:
    """Creats a 4-DOF robot arm frame tree with:
    - Base (fixed)
    - Base rotation joint (rotary) - rotates around Z
    - Shoulder joint (rotary) - rotates around Y
    - Elbow joint (rotary) - rotates around Y
    - Wrist joint (rotary) - rotates around Y
    - End effector (fixed)

    Returns:
        A KinematicTree representing a 4-DOF robot arm
    """
    urdf_path = os.path.join(_URDF_DIR, "4dof_arm.urdf")
    return urdf_parser.parse_kinematic_tree(urdf_path)


def gen_5dof_arm() -> kinematic_tree.KinematicTree:
    """
    Create a 5-DOF robot arm frame tree.

    The arm consists of:
    - Base (fixed)
    - Base rotation joint (rotary) - rotates around Z
    - Shoulder joint (rotary) - rotates around Y
    - Elbow joint (rotary) - rotates around Y
    - Wrist pitch joint (rotary) - rotates around Y
    - Wrist roll joint (rotary) - rotates around X
    - End effector (fixed)

    Returns:
        A KinematicTree representing a 5-DOF robot arm
    """
    urdf_path = os.path.join(_URDF_DIR, "5dof_arm.urdf")
    return urdf_parser.parse_kinematic_tree(urdf_path)


def gen_6dof_arm() -> kinematic_tree.KinematicTree:
    """
    Create a 6-DOF industrial robot arm frame tree.

    The arm consists of:
    - Base (fixed)
    - Base rotation joint (rotary) - rotates around Z
    - Shoulder joint (rotary) - rotates around Y
    - Elbow joint (rotary) - rotates around Y
    - Wrist 1 joint (rotary) - rotates around X
    - Wrist 2 joint (rotary) - rotates around Y
    - Wrist 3 joint (rotary) - rotates around X
    - End effector (fixed)

    Returns:
        A KinematicTree representing a 6-DOF industrial robot arm
    """
    urdf_path = os.path.join(_URDF_DIR, "6dof_arm.urdf")
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"6-DOF URDF file not found: {urdf_path}")

    return urdf_parser.parse_kinematic_tree(urdf_path)


def from_urdf(file_path: str) -> kinematic_tree.KinematicTree:
    """
    Create a frame tree from a URDF file.

    Args:
        file_path: Path to the URDF file

    Returns:
        A KinematicTree representing the robot described in the URDF file
    """
    return urdf_parser.parse_kinematic_tree(file_path)


def get_available_urdf_files() -> List[str]:
    """
    Get a list of available URDF files in the URDF directory.

    Returns:
        List of URDF file names without extension
    """
    if not os.path.exists(_URDF_DIR):
        return []

    urdf_files = [
        os.path.splitext(f)[0] for f in os.listdir(_URDF_DIR) if f.endswith(".urdf")
    ]

    return urdf_files


def simple_frame_tree() -> kinematic_tree.KinematicTree:
    """Creates a simple kinematic tree with three revolute joints.

    Returns:
        A KinematicTree representing a 3-DOF robot arm
    """
    tree = kinematic_tree.KinematicTree(geometry.ReferenceFrame.BASE)

    # Add frames with their respective joints and parameters
    tree.add_rotary_link(
        frame=geometry.ReferenceFrame.ARM,
        parent=geometry.ReferenceFrame.BASE,
        origin=geometry.Position(geometry.ReferenceFrame.BASE, 0, 0, 0.5),
        orientation=geometry.Orientation(geometry.ReferenceFrame.BASE, 0, 0, 0),
        axis=geometry.Direction(
            geometry.ReferenceFrame.BASE, 0, 0, 1
        ),  # Rotation around Z
        limits=(-np.pi, np.pi),
    )

    tree.add_rotary_link(
        frame=geometry.ReferenceFrame.FOREARM,
        parent=geometry.ReferenceFrame.ARM,
        origin=geometry.Position(geometry.ReferenceFrame.ARM, 0.4, 0, 0),
        orientation=geometry.Orientation(geometry.ReferenceFrame.ARM, 0, 0, 0),
        axis=geometry.Direction(
            geometry.ReferenceFrame.ARM, 0, 1, 0
        ),  # Rotation around Y
        limits=(-np.pi / 2, np.pi / 2),
    )

    tree.add_rotary_link(
        frame=geometry.ReferenceFrame.HAND,
        parent=geometry.ReferenceFrame.FOREARM,
        origin=geometry.Position(geometry.ReferenceFrame.FOREARM, 0.3, 0, 0),
        orientation=geometry.Orientation(geometry.ReferenceFrame.FOREARM, 0, 0, 0),
        axis=geometry.Direction(
            geometry.ReferenceFrame.FOREARM, 0, 1, 0
        ),  # Rotation around Y
        limits=(-np.pi, np.pi),
    )

    return tree


class KinematicTreeFixtureFactory:
    """
    Factory class providing standard kinematic tree fixtures for testing and examples.

    This class provides factory methods for creating different robot kinematic structures
    for testing and examples.
    """

    @staticmethod
    def from_urdf(file_path: str) -> kinematic_tree.KinematicTree:
        """
        Create a KinematicTree from a URDF file.

        Args:
            file_path: The path to the URDF file

        Returns:
            A KinematicTree representing the robot described in the URDF file
        """
        return urdf_parser.parse_kinematic_tree(file_path)

    @staticmethod
    def simple_arm() -> kinematic_tree.KinematicTree:
        """
        Create a simple 3DOF robot arm with a revolute joint at each link.

        The arm has the following structure:
        - Base
          - Arm (rotates around Z axis)
            - Forearm (rotates around Z axis)
              - Hand (rotates around Z axis)

        Returns:
            A KinematicTree representing a simple 3DOF arm
        """
        tree = kinematic_tree.KinematicTree()

        # Define frames
        arm_frame = geometry.ReferenceFrame("arm")
        forearm_frame = geometry.ReferenceFrame("forearm")
        hand_frame = geometry.ReferenceFrame("hand")

        # Add links with revolute joints
        tree.add_rotary_link(
            frame=arm_frame,
            parent=geometry.BASE,
            origin=geometry.Position(geometry.BASE, 0, 0, 0),
            orientation=geometry.Orientation(geometry.BASE, 0, 0, 0),
            axis=geometry.Direction(geometry.BASE, 0, 0, 1),
        )

        tree.add_rotary_link(
            frame=forearm_frame,
            parent=arm_frame,
            origin=geometry.Position(arm_frame, 1, 0, 0),
            orientation=geometry.Orientation(arm_frame, 0, 0, 0),
            axis=geometry.Direction(arm_frame, 0, 0, 1),
        )

        tree.add_rotary_link(
            frame=hand_frame,
            parent=forearm_frame,
            origin=geometry.Position(forearm_frame, 1, 0, 0),
            orientation=geometry.Orientation(forearm_frame, 0, 0, 0),
            axis=geometry.Direction(forearm_frame, 0, 0, 1),
        )

        return tree

    @staticmethod
    def planar_arm() -> kinematic_tree.KinematicTree:
        """
        Create a 3DOF planar arm with revolute joints.

        The arm has three links that rotate in the XY plane.

        Returns:
            A KinematicTree representing a 3DOF planar arm
        """
        tree = kinematic_tree.KinematicTree()

        # Define frames
        link1_frame = geometry.ReferenceFrame("link1")
        link2_frame = geometry.ReferenceFrame("link2")
        link3_frame = geometry.ReferenceFrame("link3")

        # Add links with revolute joints
        tree.add_rotary_link(
            frame=link1_frame,
            parent=geometry.BASE,
            origin=geometry.Position(geometry.BASE, 0, 0, 0),
            orientation=geometry.Orientation(geometry.BASE, 0, 0, 0),
            axis=geometry.Direction(geometry.BASE, 0, 0, 1),
        )

        tree.add_rotary_link(
            frame=link2_frame,
            parent=link1_frame,
            origin=geometry.Position(link1_frame, 0.5, 0, 0),
            orientation=geometry.Orientation(link1_frame, 0, 0, 0),
            axis=geometry.Direction(link1_frame, 0, 0, 1),
        )

        tree.add_rotary_link(
            frame=link3_frame,
            parent=link2_frame,
            origin=geometry.Position(link2_frame, 0.5, 0, 0),
            orientation=geometry.Orientation(link2_frame, 0, 0, 0),
            axis=geometry.Direction(link2_frame, 0, 0, 1),
        )

        return tree

    @staticmethod
    def industrial_arm() -> kinematic_tree.KinematicTree:
        """
        Create a 6DOF industrial robot arm with revolute joints.

        The arm has the structure similar to a standard industrial robot with 6 degrees of freedom.

        Returns:
            A KinematicTree representing a 6DOF industrial arm
        """
        tree = kinematic_tree.KinematicTree()

        # Define frames
        base_link_frame = geometry.ReferenceFrame("base_link")
        shoulder_frame = geometry.ReferenceFrame("shoulder")
        upper_arm_frame = geometry.ReferenceFrame("upper_arm")
        forearm_frame = geometry.ReferenceFrame("forearm")
        wrist_1_frame = geometry.ReferenceFrame("wrist_1")
        wrist_2_frame = geometry.ReferenceFrame("wrist_2")
        tool_frame = geometry.ReferenceFrame("tool")

        # Add base link (fixed)
        tree.add_stationary_link(
            frame=base_link_frame,
            parent=geometry.BASE,
            origin=geometry.Position(geometry.BASE, 0, 0, 0.1),
            orientation=geometry.Orientation(geometry.BASE, 0, 0, 0),
        )

        # Add shoulder (rotates around Z)
        tree.add_rotary_link(
            frame=shoulder_frame,
            parent=base_link_frame,
            origin=geometry.Position(base_link_frame, 0, 0, 0.089),
            orientation=geometry.Orientation(base_link_frame, 0, 0, 0),
            axis=geometry.Direction(base_link_frame, 0, 0, 1),
        )

        # Add upper arm (rotates around Y)
        tree.add_rotary_link(
            frame=upper_arm_frame,
            parent=shoulder_frame,
            origin=geometry.Position(shoulder_frame, 0, 0, 0),
            orientation=geometry.Orientation(
                shoulder_frame, 0, 1.5708, 0
            ),  # 90 degrees in Y
            axis=geometry.Direction(shoulder_frame, 0, 1, 0),
        )

        # Add forearm (rotates around Y)
        tree.add_rotary_link(
            frame=forearm_frame,
            parent=upper_arm_frame,
            origin=geometry.Position(upper_arm_frame, 0, -0.425, 0),
            orientation=geometry.Orientation(upper_arm_frame, 0, 0, 0),
            axis=geometry.Direction(upper_arm_frame, 0, 1, 0),
        )

        # Add wrist 1 (rotates around Y)
        tree.add_rotary_link(
            frame=wrist_1_frame,
            parent=forearm_frame,
            origin=geometry.Position(forearm_frame, 0, -0.392, 0),
            orientation=geometry.Orientation(forearm_frame, 0, 0, 0),
            axis=geometry.Direction(forearm_frame, 0, 1, 0),
        )

        # Add wrist 2 (rotates around Z)
        tree.add_rotary_link(
            frame=wrist_2_frame,
            parent=wrist_1_frame,
            origin=geometry.Position(wrist_1_frame, 0, 0, 0.127),
            orientation=geometry.Orientation(
                wrist_1_frame, 0, 1.5708, 0
            ),  # 90 degrees in Y
            axis=geometry.Direction(wrist_1_frame, 0, 0, 1),
        )

        # Add tool (rotates around Y)
        tree.add_rotary_link(
            frame=tool_frame,
            parent=wrist_2_frame,
            origin=geometry.Position(wrist_2_frame, 0, 0, 0),
            orientation=geometry.Orientation(
                wrist_2_frame, 0, 1.5708, 0
            ),  # 90 degrees in Y
            axis=geometry.Direction(wrist_2_frame, 0, 1, 0),
        )

        return tree

    @staticmethod
    def gen_3dof_arm() -> kinematic_tree.KinematicTree:
        """
        Create a 3DOF arm with revolute joints at different orientations.

        Returns:
            A KinematicTree representing a 3DOF arm
        """
        return KinematicTreeFixtureFactory.simple_arm()

    @staticmethod
    def gen_4dof_arm() -> kinematic_tree.KinematicTree:
        """
        Create a 4DOF arm with revolute joints.

        Returns:
            A KinematicTree representing a 4DOF arm
        """
        tree = kinematic_tree.KinematicTree()

        # Define frames
        link1_frame = geometry.ReferenceFrame("link1")
        link2_frame = geometry.ReferenceFrame("link2")
        link3_frame = geometry.ReferenceFrame("link3")
        link4_frame = geometry.ReferenceFrame("link4")

        # Add links with revolute joints
        tree.add_rotary_link(
            frame=link1_frame,
            parent=geometry.BASE,
            origin=geometry.Position(geometry.BASE, 0, 0, 0.1),
            orientation=geometry.Orientation(geometry.BASE, 0, 0, 0),
            axis=geometry.Direction(geometry.BASE, 0, 0, 1),
        )

        tree.add_rotary_link(
            frame=link2_frame,
            parent=link1_frame,
            origin=geometry.Position(link1_frame, 0, 0, 0.2),
            orientation=geometry.Orientation(
                link1_frame, 0, 1.5708, 0
            ),  # 90 degrees in Y
            axis=geometry.Direction(link1_frame, 0, 1, 0),
        )

        tree.add_rotary_link(
            frame=link3_frame,
            parent=link2_frame,
            origin=geometry.Position(link2_frame, 0, 0, 0.3),
            orientation=geometry.Orientation(link2_frame, 0, 0, 0),
            axis=geometry.Direction(link2_frame, 0, 1, 0),
        )

        tree.add_rotary_link(
            frame=link4_frame,
            parent=link3_frame,
            origin=geometry.Position(link3_frame, 0, 0, 0.2),
            orientation=geometry.Orientation(link3_frame, 0, 0, 0),
            axis=geometry.Direction(link3_frame, 1, 0, 0),
        )

        return tree

    @staticmethod
    def gen_5dof_arm() -> kinematic_tree.KinematicTree:
        """
        Create a 5DOF arm with revolute joints.

        Returns:
            A KinematicTree representing a 5DOF arm
        """
        tree = kinematic_tree.KinematicTree()

        # Define frames
        link1_frame = geometry.ReferenceFrame("link1")
        link2_frame = geometry.ReferenceFrame("link2")
        link3_frame = geometry.ReferenceFrame("link3")
        link4_frame = geometry.ReferenceFrame("link4")
        link5_frame = geometry.ReferenceFrame("link5")

        # Add links with revolute joints
        tree.add_rotary_link(
            frame=link1_frame,
            parent=geometry.BASE,
            origin=geometry.Position(geometry.BASE, 0, 0, 0.1),
            orientation=geometry.Orientation(geometry.BASE, 0, 0, 0),
            axis=geometry.Direction(geometry.BASE, 0, 0, 1),
        )

        tree.add_rotary_link(
            frame=link2_frame,
            parent=link1_frame,
            origin=geometry.Position(link1_frame, 0, 0, 0.1),
            orientation=geometry.Orientation(link1_frame, 0, 1.5708, 0),
            axis=geometry.Direction(link1_frame, 0, 1, 0),
        )

        tree.add_rotary_link(
            frame=link3_frame,
            parent=link2_frame,
            origin=geometry.Position(link2_frame, 0, 0.3, 0),
            orientation=geometry.Orientation(link2_frame, 0, 0, 0),
            axis=geometry.Direction(link2_frame, 0, 1, 0),
        )

        tree.add_rotary_link(
            frame=link4_frame,
            parent=link3_frame,
            origin=geometry.Position(link3_frame, 0, 0.25, 0),
            orientation=geometry.Orientation(link3_frame, 0, 0, 0),
            axis=geometry.Direction(link3_frame, 1, 0, 0),
        )

        tree.add_rotary_link(
            frame=link5_frame,
            parent=link4_frame,
            origin=geometry.Position(link4_frame, 0, 0.1, 0),
            orientation=geometry.Orientation(link4_frame, 0, 0, 0),
            axis=geometry.Direction(link4_frame, 0, 1, 0),
        )

        return tree

    @staticmethod
    def gen_6dof_arm() -> kinematic_tree.KinematicTree:
        """
        Create a 6DOF arm with revolute joints, similar to industrial robots.

        Returns:
            A KinematicTree representing a 6DOF arm
        """
        return KinematicTreeFixtureFactory.industrial_arm()
