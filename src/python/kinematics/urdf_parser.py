"""
URDF Parser Module

This module provides functions for parsing URDF (Unified Robot Description Format)
files and creating frame trees from them. It handles extracting link and joint
information from the XML structure and populating the frame tree accordingly.
"""
from typing import List, Optional, Tuple
from xml.etree import ElementTree

import geometry

from kinematics import kinematic_tree, primitives


def parse_kinematic_tree(urdf_path: str) -> kinematic_tree.KinematicTree:
    """Parse a URDF file and create a kinematic tree. This is a wrapper around
    parse_urdf that handles file operations.
    """
    root = _extract_root(urdf_path)
    joints = root.findall("joint")
    root_frame = _find_root_frame(joints)
    tree = kinematic_tree.KinematicTree(root_frame)

    for joint_elem in joints:
        joint_type_str = joint_elem.get("type", "fixed")
        joint_type = _map_joint_type(joint_type_str)

        limits = _get_limits(joint_elem)

        # Add link to kinematic tree with joint information
        if (parent_frame := _get_parent_frame(joint_elem)) is None:
            continue

        frame = _get_frame(joint_elem)

        # Convert tuples to geometry objects
        origin_pos = _get_origin(parent_frame, joint_elem)
        origin_ori = _get_origin_ori(parent_frame, joint_elem)

        axis = _get_axis(parent_frame, joint_elem)

        if joint_type == primitives.JointType.FIXED:
            tree.add_stationary_link(
                frame=frame,
                parent=parent_frame,
                origin=origin_pos,
                orientation=origin_ori,
            )
        elif joint_type == primitives.JointType.ROTARY:
            tree.add_rotary_link(
                frame=frame,
                parent=parent_frame,
                origin=origin_pos,
                orientation=origin_ori,
                axis=axis,
                limits=limits,
            )
        else:
            raise ValueError(f"Unsupported joint type: {joint_type}")

    return tree


def _find_root_frame(joints: List[ElementTree.Element]) -> geometry.ReferenceFrame:
    """Find the root link of the robot by identifying which link has no parent."""
    # Ensure child elements exist before getting 'link'
    child_link_elements = [elem.find("child") for elem in joints]
    children = [
        child.get("link")
        for child in child_link_elements
        if child is not None and child.get("link")
    ]
    for elem in joints:
        # Ensure parent element exists before getting 'link'
        parent_elem = elem.find("parent")
        if (
            parent_elem is not None
            and (parent := parent_elem.get("link"))
            and (parent not in children)
        ):
            return geometry.ReferenceFrame(parent.upper())

    raise ValueError("No Root Link Found")


def _map_joint_type(type_str: str) -> Optional[primitives.JointType]:
    """Map a URDF joint type string to a JointType enum."""
    type_map = {
        "revolute": primitives.JointType.ROTARY,
        "fixed": primitives.JointType.FIXED,
    }
    return type_map.get(type_str.lower())


def _extract_root(urdf_path: str) -> ElementTree.Element:
    """Extract the tree from a URDF file."""
    try:
        tree = ElementTree.parse(urdf_path)
        root = tree.getroot()
        return root
    except FileNotFoundError:
        raise FileNotFoundError(f"URDF file '{urdf_path}' not found")
    except ElementTree.ParseError as e:
        raise ValueError(f"Invalid URDF file: {str(e)}")


def _get_axis(
    frame: geometry.ReferenceFrame, joint_elem: ElementTree.Element
) -> geometry.Direction:
    """Get the axis of a joint from the URDF element."""
    if axis_elem := joint_elem.find("axis"):
        axis_str = axis_elem.get("xyz")
        # Ensure axis_str is not None before splitting
        if axis_str:
            return geometry.Direction(frame, *[float(val) for val in axis_str.split()])
    # Default axis if not found or empty
    return geometry.Direction.unit_z(frame)


def _get_limits(joint_elem: ElementTree.Element) -> Tuple[float, float]:
    """Get the limits of a joint from the URDF element."""
    if limit_elem := joint_elem.find("limit"):
        return (
            float(limit_elem.get("lower", -float("inf"))),
            float(limit_elem.get("upper", float("inf"))),
        )
    else:
        return (-float("inf"), float("inf"))


def _get_origin(
    frame: geometry.ReferenceFrame, joint_elem: ElementTree.Element
) -> geometry.Position:
    """Get the origin of a joint from the URDF element."""
    if origin_elem := joint_elem.find("origin"):
        origin_str = origin_elem.get("xyz")
        # Ensure origin_str is not None before splitting
        if origin_str:
            return geometry.Position(frame, *[float(val) for val in origin_str.split()])
    # Default position if not found or empty
    return geometry.Position.zero(frame)


def _get_origin_ori(
    frame: geometry.ReferenceFrame, joint_elem: ElementTree.Element
) -> geometry.Orientation:
    """Get the origin of a joint from the URDF element."""
    if (elem := joint_elem.find("origin")) and (ori_str := elem.get("rpy")):
        return geometry.Orientation(frame, *[float(val) for val in ori_str.split()])
    else:
        return geometry.Orientation.zero(frame)


def _get_parent_frame(
    joint_elem: ElementTree.Element,
) -> Optional[geometry.ReferenceFrame]:
    """Get the parent frame of a joint from the URDF element."""
    if (elem := joint_elem.find("parent")) and (link := elem.get("link")):
        return geometry.ReferenceFrame(link.upper())
    else:
        return None


def _get_frame(joint_elem: ElementTree.Element) -> geometry.ReferenceFrame:
    """Get the frame of a joint from the URDF element."""
    if (elem := joint_elem.find("child")) and (link := elem.get("link")):
        return geometry.ReferenceFrame(link.upper())
    else:
        raise ValueError("Joint has no frame")
