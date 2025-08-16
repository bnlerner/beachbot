import os
import tempfile
from typing import Generator

import geometry
import pytest

from kinematics import primitives, urdf_parser
from kinematics.urdf_parser import _map_joint_type


@pytest.fixture
def temp_urdf_file() -> Generator[str, None, None]:
    """Create a temporary URDF file for testing."""
    temp_urdf = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
    temp_urdf.write(
        b"""
    <robot name="test_robot">
        <link name="base">
            <visual>
                <geometry>
                    <cylinder radius="0.1" length="0.05"/>
                </geometry>
            </visual>
        </link>

        <link name="arm">
            <visual>
                <geometry>
                    <box size="0.1 0.1 0.1"/>
                </geometry>
            </visual>
        </link>

        <link name="forearm">
            <visual>
                <geometry>
                    <box size="0.1 0.1 0.1"/>
                </geometry>
            </visual>
        </link>

        <joint name="arm" type="revolute">
            <parent link="base"/>
            <child link="arm"/>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
            <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
        </joint>

        <joint name="joint2" type="revolute">
            <parent link="arm"/>
            <child link="forearm"/>
            <origin xyz="0.2 0 0" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
            <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
        </joint>
    </robot>
    """
    )
    temp_urdf.close()

    yield temp_urdf.name

    # Clean up
    os.unlink(temp_urdf.name)


def test_parse_urdf_file(temp_urdf_file: str) -> None:
    """Test parsing a URDF file."""
    tree = urdf_parser.parse_kinematic_tree(temp_urdf_file)

    # Check that frames were created
    assert geometry.BASE in tree.frames
    assert geometry.ARM in tree.frames
    assert geometry.FOREARM in tree.frames

    # Check parent-child relationships
    assert tree.parent_frame(geometry.ARM) == geometry.BASE
    assert tree.parent_frame(geometry.FOREARM) == geometry.ARM


def test_map_joint_type() -> None:
    """Test mapping of URDF joint types to internal joint types."""

    assert _map_joint_type("revolute") == primitives.JointType.ROTARY
    assert _map_joint_type("continuous") is None
    assert _map_joint_type("fixed") == primitives.JointType.FIXED
    assert _map_joint_type("floating") is None
    assert _map_joint_type("planar") is None
