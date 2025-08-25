"""
Kinematic Tree Implementation

This module provides a tree-based representation of a robot's kinematic structure,
with frames representing rigid bodies and connections representing joints. The tree
structure enables efficient transformations between frames and calculation of forward
and inverse kinematics.
"""

from typing import Dict, List, Optional, Tuple, Union, overload

import geometry
import numpy as np
from typing_helpers import req

from kinematics import joint_nodes


class KinematicTree:
    """A tree structure representing the kinematic relationships between rigid body
    frames.

    This class provides methods for creating and manipulating a tree of frames,
    transforming positions, orientations and poses between frames, and performing
    forward kinematics operations.

    The kinematic tree maintains the topology of the robot's kinematic structure, with
    frames representing rigid bodies and connections representing joints. The tree
    structure enables efficient traversal and transformation operations between
    any two frames in the tree.

    The coordinate frames follow standard robotics conventions, with transformations
    represented as 4x4 homogeneous transformation matrices. When transforming between
    frames, the full chain of transformations from source to target frame is computed
    automatically.

    Typical usage:
    1. Create a kinematic tree with a root frame.
    2. Add frames and connections to build the robot structure.
    3. Use transformation methods to convert positions, orientations, or poses
       between different frames.
    4. Calculate forward kinematics to find end effector pose for given joint values.
    5. Use the IKSolver class for inverse kinematics to find joint values for desired
       poses.
    """

    def __init__(self, root_frame: geometry.ReferenceFrame = geometry.BASE):
        """Initialize a kinematic tree with the specified root frame."""
        self._root_frame = root_frame
        self._links: Dict[geometry.ReferenceFrame, joint_nodes.BaseFrameNode] = {}

        # Joint values for non-fixed joints (name -> value)
        self._joint_values: Dict[str, float] = {}

        # TODO: Add Caching for transformation matrices to improve performance

        # Add the root frame which is always fixed and at the origin
        self._links[root_frame] = joint_nodes.FixedFrameNode(
            frame=root_frame,
            origin=geometry.Position.zero(root_frame),
            orientation=geometry.Orientation.zero(root_frame),
        )

    @property
    def frames(self) -> List[geometry.ReferenceFrame]:
        """Get all frames in the tree."""
        return list(self._links.keys())

    def add_stationary_link(
        self,
        frame: geometry.ReferenceFrame,
        origin: geometry.Position,
        orientation: geometry.Orientation,
    ) -> None:
        """Adds a stationary link to the tree in the reference frame of the origin and
        orientation provided.
        """
        self._raise_if_link_exists(frame)
        parent_node = self._links[origin.frame] if origin else None
        node = joint_nodes.FixedFrameNode(
            frame=frame, parent=parent_node, origin=origin, orientation=orientation
        )
        if parent_node:
            parent_node.add_child(node)

        self._links[frame] = node

    def add_rotary_link(
        self,
        frame: geometry.ReferenceFrame,
        origin: geometry.Position,
        orientation: geometry.Orientation,
        axis: geometry.Direction,
        limits: Tuple[float, float] = (-float("inf"), float("inf")),
    ) -> None:
        """Adds a rotary link to the tree in the reference frame of the origin and
        orientation provided.
        """
        self._raise_if_link_exists(frame)
        self._raise_if_no_parent_exists(origin.frame)
        parent_node = self.get_node(origin.frame)
        node = joint_nodes.RotaryFrameNode(
            frame=frame,
            origin=origin,
            orientation=orientation,
            rotary_axis=axis,
            velocity=geometry.Velocity.zero(frame),
            acceleration=geometry.Acceleration.zero(frame),
            limits=limits,
            parent=parent_node,
        )
        self._links[origin.frame].add_child(node)
        self._links[frame] = node

    def parent_frame(
        self, frame: geometry.ReferenceFrame
    ) -> Optional[geometry.ReferenceFrame]:
        """Get the parent frame of a frame."""
        self._raise_if_no_link_exists(frame)

        parent_node = self._links[frame].parent
        return parent_node.frame if parent_node else None

    def frame_lineage(
        self, frame: geometry.ReferenceFrame
    ) -> List[geometry.ReferenceFrame]:
        """Get the lineage of a frame to the root frame."""
        self._raise_if_no_link_exists(frame)

        return self._path_to_root(frame)

    def get_node(self, frame: geometry.ReferenceFrame) -> joint_nodes.BaseFrameNode:
        """Get the node for a frame."""
        self._raise_if_no_link_exists(frame)

        return self._links[frame]

    def get_rotary_nodes(self) -> List[joint_nodes.RotaryFrameNode]:
        """Get all rotary nodes in the tree."""
        return [
            node
            for node in self._links.values()
            if isinstance(node, joint_nodes.RotaryFrameNode)
        ]

    @overload
    def transform(
        self, obj: geometry.Position, target: geometry.ReferenceFrame
    ) -> geometry.Position:
        ...

    @overload
    def transform(
        self, obj: geometry.Orientation, target: geometry.ReferenceFrame
    ) -> geometry.Orientation:
        ...

    @overload
    def transform(
        self, obj: geometry.Pose, target: geometry.ReferenceFrame
    ) -> geometry.Pose:
        ...

    @overload
    def transform(
        self, obj: geometry.Velocity, target: geometry.ReferenceFrame
    ) -> geometry.Velocity:
        ...

    @overload
    def transform(
        self, obj: geometry.Direction, target: geometry.ReferenceFrame
    ) -> geometry.Direction:
        ...

    @overload
    def transform(
        self, obj: geometry.Acceleration, target: geometry.ReferenceFrame
    ) -> geometry.Acceleration:
        ...

    @overload
    def transform(
        self, obj: geometry.AngularVelocity, target: geometry.ReferenceFrame
    ) -> geometry.AngularVelocity:
        ...

    @overload
    def transform(
        self, obj: geometry.AngularAcceleration, target: geometry.ReferenceFrame
    ) -> geometry.AngularAcceleration:
        ...

    @overload
    def transform(
        self, obj: geometry.Twist, target: geometry.ReferenceFrame
    ) -> geometry.Twist:
        ...

    def transform(
        self,
        obj: Union[
            geometry.Position,
            geometry.Orientation,
            geometry.Twist,
            geometry.Pose,
            geometry.Direction,
            geometry.Velocity,
            geometry.Acceleration,
            geometry.AngularVelocity,
            geometry.AngularAcceleration,
        ],
        target: geometry.ReferenceFrame,
    ) -> Union[
        geometry.Position,
        geometry.Orientation,
        geometry.Twist,
        geometry.Pose,
        geometry.Direction,
        geometry.Velocity,
        geometry.Acceleration,
        geometry.AngularVelocity,
        geometry.AngularAcceleration,
    ]:
        """Transform an object into the target frame."""
        # Transform based on object type
        if isinstance(obj, geometry.Position):
            return self._transform_position(obj, target)
        elif isinstance(obj, geometry.Orientation):
            return self._transform_orientation(obj, target)
        elif isinstance(obj, geometry.Pose):
            return self._transform_pose(obj, target)
        elif isinstance(obj, geometry.Direction):
            return self._transform_direction(obj, target)
        elif isinstance(obj, geometry.Velocity):
            return self._transform_velocity(obj, target)
        elif isinstance(obj, geometry.Acceleration):
            return self._transform_acceleration(obj, target)
        elif isinstance(obj, geometry.AngularVelocity):
            return self._transform_angular_velocity(obj, target)
        elif isinstance(obj, geometry.AngularAcceleration):
            return self._transform_angular_acceleration(obj, target)
        elif isinstance(obj, geometry.Twist):
            return self._transform_twist(obj, target)
        else:
            raise TypeError("Input must be a Position, Orientation, or Pose object")

    def update_node_state(
        self,
        frame: geometry.ReferenceFrame,
        *,
        orientation: Optional[geometry.Orientation] = None,
        velocity: Optional[geometry.Velocity] = None,
        acceleration: Optional[geometry.Acceleration] = None,
        angular_velocity: Optional[geometry.AngularVelocity] = None,
        angular_acceleration: Optional[geometry.AngularAcceleration] = None,
    ) -> None:
        """Update the kinematic state of a node.

        This method updates the orientation, velocity, acceleration, angular
        velocity, and/or angular acceleration of a node in the frame tree. The updates
        are applied in a way that respects the joint constraints.

        An acceleration or velocity of a node is represented in its own frame.
        Orientation is represented relative to the parent frame.

        Once the position of a node is set relative to its parent, it cannot be updated.
        This may change in the future if a prismatic joint is added.

        # TODO: Look into locking this down further to prevent invalid updates via bad
        # orientations or velocities
        """
        if frame not in self._links:
            raise ValueError(f"Frame '{frame}' does not exist in the tree")

        node = self.get_node(frame)

        # Update each component if provided
        if orientation is not None:
            if node.joint_type == joint_nodes.JointType.FIXED:
                raise ValueError(f"Cannot update orientation of fixed joint '{frame}'")
            node.orientation = orientation

        if velocity is not None:
            node.velocity = velocity

        if acceleration is not None:
            node.acceleration = acceleration

        if angular_velocity is not None:
            node.angular_velocity = angular_velocity

        if angular_acceleration is not None:
            node.angular_acceleration = angular_acceleration

    def _get_transformation_matrix(
        self, source: geometry.ReferenceFrame, target: geometry.ReferenceFrame
    ) -> np.ndarray:
        """Get the 4x4 transformation matrix from source frame to target frame."""
        # Check if frames exist
        if source not in self._links:
            raise ValueError(f"Source frame '{source}' does not exist")
        if target not in self._links:
            raise ValueError(f"Target frame '{target}' does not exist")

        # Check if the target is above the source which indicates an upward transform or
        # from parent to child.
        is_upward = target in self._path_to_root(source)
        # TODO: Check the cache if no joint values are specified

        # If source and target are the same, return identity
        transform = np.eye(4)
        if source == target:
            return transform

        # Find the path from source to target
        if not (path := self._find_path(source, target)):
            raise ValueError(f"No path from '{source}' to '{target}'")

        # Calculate the transformation by traversing the path
        for i, _ in enumerate(path[1:], 1):
            prev_frame = path[i - 1]
            # We always traverse edge (prev_frame -> frame). Each node stores the
            # transform T_child_parent (child to parent). For mapping coordinates:
            # - Going upward (child->parent): multiply by T_child_parent
            # - Going downward (parent->child): multiply by inverse(T_child_parent)
            if is_upward:
                transform = transform.dot(
                    self._links[prev_frame].get_inverse_transformation_matrix()
                )
            else:
                transform = transform.dot(
                    self._links[prev_frame].get_transformation_matrix()
                )
        # TODO: add to cache

        return transform

    def _find_path(
        self, source: geometry.ReferenceFrame, target: geometry.ReferenceFrame
    ) -> List[geometry.ReferenceFrame]:
        """Find the path from source to target frame. List of frames forming the path
        from source to target
        """
        # If the source or target is the root, the path is simpler
        if source == self._root_frame:
            return self._path_to_root(target)[::-1]
        if target == self._root_frame:
            return self._path_to_root(source)

        # Get paths to root
        source_to_root = self._path_to_root(source)
        target_to_root = self._path_to_root(target)

        # Find the common ancestor
        common_ancestor = None
        for s in source_to_root:
            if s in target_to_root:
                common_ancestor = s
                break

        if common_ancestor is None:
            # No common ancestor, should never happen in a tree
            return []

        # Build the path: source → common ancestor → target
        source_to_common = []
        for frame in source_to_root:
            source_to_common.append(frame)
            if frame == common_ancestor:
                break

        common_to_target = []
        for frame in target_to_root:
            if frame == common_ancestor:
                break
            common_to_target.append(frame)

        return source_to_common + common_to_target[::-1]

    def _path_to_root(
        self, frame: geometry.ReferenceFrame
    ) -> List[geometry.ReferenceFrame]:
        """Get the path as a list of frames from a frame to the root. Includes passed
        and root frame.
        """
        path = [frame]
        current = self._links[frame]
        while True:
            if current.parent:
                current = current.parent
                path.append(current.frame)
            else:
                break

        return path

    def _rotary_nodes(self) -> List[joint_nodes.RotaryFrameNode]:
        """Get all rotary nodes in the tree."""
        return [
            node
            for node in self._links.values()
            if isinstance(node, joint_nodes.RotaryFrameNode)
        ]

    def _transform_position(
        self, position: geometry.Position, target_frame: geometry.ReferenceFrame
    ) -> geometry.Position:
        """Transform a position from its frame to the target frame."""
        transform = self._get_transformation_matrix(position.frame, target_frame)

        trans_array = transform.dot(position.as_array(homogeneous=True))
        return geometry.Position.from_array(target_frame, trans_array[:3])

    def _transform_orientation(
        self, orientation: geometry.Orientation, target_frame: geometry.ReferenceFrame
    ) -> geometry.Orientation:
        """Transform an orientation from its frame to the target frame."""
        # Get the transformation matrix
        transform = self._get_transformation_matrix(orientation.frame, target_frame)

        # Extract the rotation matrix from the transform
        rot_matrix = transform[:3, :3]
        rotation = geometry.Rotation.from_matrix(target_frame, rot_matrix)

        orientation.frame = target_frame
        trans_ori = orientation.rotated(rotation, intrinsic=False)
        return trans_ori

    def _transform_direction(
        self, direction: geometry.Direction, target_frame: geometry.ReferenceFrame
    ) -> geometry.Direction:
        """Transform a direction from its frame to the target frame."""
        # Get the transformation matrix
        transform = self._get_transformation_matrix(direction.frame, target_frame)

        # Extract the rotation matrix from the transform
        rot_matrix = transform[:3, :3]
        rotation = geometry.Rotation.from_matrix(target_frame, rot_matrix)

        direction.frame = target_frame
        trans_dir = direction.rotated(rotation, intrinsic=False)
        return trans_dir

    def _transform_pose(
        self, pose: geometry.Pose, target_frame: geometry.ReferenceFrame
    ) -> geometry.Pose:
        """Transform a pose from its frame to the target frame."""
        transformed_position = self._transform_position(pose.position, target_frame)
        transformed_orientation = self._transform_orientation(
            pose.orientation, target_frame
        )

        return geometry.Pose(
            position=transformed_position, orientation=transformed_orientation
        )

    def _transform_twist(
        self, twist: geometry.Twist, target_frame: geometry.ReferenceFrame
    ) -> geometry.Twist:
        """Transform a twist from its frame to the target frame."""
        transformed_velocity = self._transform_velocity(twist.velocity, target_frame)
        transformed_spin = self._transform_angular_velocity(twist.spin, target_frame)

        return geometry.Twist(velocity=transformed_velocity, spin=transformed_spin)

    def _transform_velocity(
        self,
        velocity: geometry.Velocity,
        target_frame: geometry.ReferenceFrame,
        position: Optional[geometry.Position] = None,
    ) -> geometry.Velocity:
        """Transform a velocity from its frame to the target frame.

        This accounts for both the rotational difference between frames and
        any angular velocity that might affect the linear velocity.

        Optional position at which the velocity is measured. If None, assumes the
        velocity is at the origin of its frame.
        """
        source_frame = velocity.frame
        # If frames are the same, just return the original velocity
        if source_frame == target_frame:
            return velocity

        # Get the position at which velocity is measured (if not provided) and ensure in
        # the source frame.
        position = position or geometry.Position.zero(source_frame)
        # Ensure position is transformed to the source frame before use
        if position.frame != source_frame:
            position = self._transform_position(position, source_frame)

        # Get the transformation matrix
        transform = self._get_transformation_matrix(source_frame, target_frame)

        # Extract rotation matrix
        rotation = transform[:3, :3]

        # If there's angular velocity, we need to account for the cross-product term
        if angular_velocity := self._links[source_frame].angular_velocity:
            # Transform angular velocity to target frame
            transformed_angular = self._transform_angular_velocity(
                angular_velocity, target_frame
            )

            # Position vector (for cross product) - Check if position is None
            if position is None:
                raise ValueError(
                    "Position must be provided when angular velocity is present for velocity transform"
                )
            position_vector = np.array([position.x, position.y, position.z])

            # Orbital velocity component: ω × r
            angular_vector = np.array(
                [transformed_angular.x, transformed_angular.y, transformed_angular.z]
            )

            orbital_velocity = np.cross(angular_vector, position_vector)

            # The total velocity is the transformed linear velocity plus the orbital component
            velocity_vector = np.array([velocity.x, velocity.y, velocity.z])
            transformed_vector = np.dot(rotation, velocity_vector) + orbital_velocity
        else:
            # Simple transformation without angular effects
            velocity_vector = np.array([velocity.x, velocity.y, velocity.z])
            transformed_vector = np.dot(rotation, velocity_vector)

        # Return transformed velocity
        return geometry.Velocity(
            frame=target_frame,
            x=transformed_vector[0],
            y=transformed_vector[1],
            z=transformed_vector[2],
        )

    def _transform_angular_velocity(
        self,
        angular_velocity: geometry.AngularVelocity,
        target_frame: geometry.ReferenceFrame,
    ) -> geometry.AngularVelocity:
        """Transform an angular velocity from its frame to the target frame."""
        source_frame = angular_velocity.frame
        # If frames are the same, just return the original angular velocity
        if source_frame == target_frame:
            return angular_velocity

        # Get the transformation matrix
        transform = self._get_transformation_matrix(source_frame, target_frame)

        # Extract rotation matrix
        rotation = transform[:3, :3]

        # Angular velocity transforms like a vector under rotation
        angular_vector = np.array(
            [angular_velocity.x, angular_velocity.y, angular_velocity.z]
        )
        transformed_vector = np.dot(rotation, angular_vector)

        # Return transformed angular velocity
        return geometry.AngularVelocity(
            frame=target_frame,
            x=transformed_vector[0],
            y=transformed_vector[1],
            z=transformed_vector[2],
        )

    def _transform_acceleration(
        self,
        acceleration: geometry.Acceleration,
        target_frame: geometry.ReferenceFrame,
        position: Optional[geometry.Position] = None,
    ) -> geometry.Acceleration:
        """Transform an acceleration from its frame to the target frame.

        This accounts for both the rotational difference between frames and
        any angular velocity/acceleration that might affect the linear acceleration.

        Optional position at which the acceleration is measured. If None, assumes the
        acceleration is at the origin of its frame.
        """
        source_frame = acceleration.frame

        # If frames are the same, just return the original acceleration
        if source_frame == target_frame:
            return acceleration

        # Get the transformation matrix
        transform = self._get_transformation_matrix(source_frame, target_frame)

        # Extract rotation matrix
        rotation = transform[:3, :3]

        # Get the position at which acceleration is measured (if not provided) and
        # ensure position is in the source frame
        position = position or geometry.Position(source_frame, 0.0, 0.0, 0.0)
        # Ensure position is transformed to the source frame before use
        if position.frame != source_frame:
            position = self._transform_position(position, source_frame)

        # Get angular velocity and acceleration of source frame relative to target frame (if any)
        source_node = self._links[source_frame]

        # The transformed acceleration is complex with angular effects
        # a' = R(a + α × r + ω × (ω × r))
        # where a is linear acceleration, α is angular acceleration, ω is angular velocity,
        # r is position vector, and R is rotation matrix

        # Base acceleration vector
        accel_vector = np.array([acceleration.x, acceleration.y, acceleration.z])
        # Check if position is None
        if position is None:
            raise ValueError(
                "Position must be provided for acceleration transform with angular effects"
            )
        position_vector = np.array([position.x, position.y, position.z])

        # Add angular effects if present
        # Transform angular quantities to target frame
        if source_node.angular_velocity:
            transformed_angular_vel = self._transform_angular_velocity(
                source_node.angular_velocity, target_frame
            )
            omega = np.array(
                [
                    transformed_angular_vel.x,
                    transformed_angular_vel.y,
                    transformed_angular_vel.z,
                ]
            )

            # Centripetal acceleration: ω × (ω × r)
            centripetal = np.cross(omega, np.cross(omega, position_vector))
            accel_vector += centripetal

        if source_node.angular_acceleration:
            transformed_angular_accel = self._transform_angular_acceleration(
                source_node.angular_acceleration, target_frame
            )
            alpha = np.array(
                [
                    transformed_angular_accel.x,
                    transformed_angular_accel.y,
                    transformed_angular_accel.z,
                ]
            )

            # Tangential acceleration: α × r
            tangential = np.cross(alpha, position_vector)
            accel_vector += tangential

        # Apply rotation
        transformed_vector = np.dot(rotation, accel_vector)

        # Return transformed acceleration
        return geometry.Acceleration(
            frame=target_frame,
            x=transformed_vector[0],
            y=transformed_vector[1],
            z=transformed_vector[2],
        )

    def _transform_angular_acceleration(
        self,
        angular_acceleration: geometry.AngularAcceleration,
        target_frame: geometry.ReferenceFrame,
    ) -> geometry.AngularAcceleration:
        """Transform an angular acceleration from its frame to the target frame."""
        source_frame = angular_acceleration.frame

        # If frames are the same, just return the original angular acceleration
        if source_frame == target_frame:
            return angular_acceleration

        # Get the transformation matrix
        transform = self._get_transformation_matrix(source_frame, target_frame)

        # Extract rotation matrix
        rotation = transform[:3, :3]

        # Angular acceleration transforms like a vector under rotation
        angular_vector = np.array(
            [angular_acceleration.x, angular_acceleration.y, angular_acceleration.z]
        )
        transformed_vector = np.dot(rotation, angular_vector)

        # Return transformed angular acceleration
        return geometry.AngularAcceleration(
            frame=target_frame,
            x=transformed_vector[0],
            y=transformed_vector[1],
            z=transformed_vector[2],
        )

    def forward_kinematics(self, position: geometry.Position) -> geometry.Pose:
        """
        Compute forward kinematics to find the world pose at a specific position.

        Args:
            position: The position for which to calculate forward kinematics

        Returns:
            Pose of the specified position in the world frame

        Raises:
            ValueError: If the position's frame is not in the tree
        """
        # Check if the frame exists in the tree
        frame = position.frame
        if frame not in self._links:
            raise ValueError(f"Frame '{frame}' does not exist in the tree")

        # Calculate the transformation matrix from the root to the specified frame
        transform = self._get_transformation_matrix(self._root_frame, frame)

        # Extract position from the transformation matrix
        world_position = transform[:3, 3]

        # Add local offsets if the position has non-zero coordinates
        if position.x != 0 or position.y != 0 or position.z != 0:
            local_offset = np.array([position.x, position.y, position.z, 1.0])
            local_transform = np.eye(4)
            local_transform[:3, 3] = local_offset[:3]
            transform = np.dot(transform, local_transform)
            world_position = transform[:3, 3]

        # Extract orientation (rotation matrix) from the transformation matrix
        world_rotation = transform[:3, :3]
        quaternion = geometry.euler_to_quaternion(world_rotation)

        # Create and return the pose
        return geometry.Pose(
            position=geometry.Position(
                x=world_position[0],
                y=world_position[1],
                z=world_position[2],
                frame=self._root_frame,
            ),
            orientation=geometry.Orientation.from_quaternion(
                w=quaternion[0],
                x=quaternion[1],
                y=quaternion[2],
                z=quaternion[3],
                frame=self._root_frame,
            ),
        )

    def _compute_jacobian(self, position: geometry.Position) -> np.ndarray:
        """
        Compute the Jacobian matrix for a given position.

        The Jacobian relates joint velocities to end-effector velocities (both linear and angular).
        For rotary joints, the columns are [J_v; J_w] where:
            J_v = z_i x (p - p_i)   (cross product of joint axis with distance to position)
            J_w = z_i                (joint axis)
        For prismatic joints, the columns are [J_v; J_w] where:
            J_v = z_i                (joint axis)
            J_w = 0                  (no angular velocity contribution)

        Args:
            position: The position for which to compute the Jacobian

        Returns:
            A 6xN matrix where N is the number of movable joints. The first 3 rows
            represent linear velocity components, the last 3 rows represent angular
            velocity components.

        Raises:
            ValueError: If the position's frame is not in the tree
        """
        # Check if the frame exists in the tree
        frame = position.frame
        if frame not in self._links:
            raise ValueError(f"Frame '{frame}' does not exist in the tree")

        # Get all movable joints
        movable_joints = self._rotary_nodes()
        jacobian = np.zeros((6, len(movable_joints)))

        # Calculate forward kinematics to get the world position of the target
        pose = self.forward_kinematics(position)
        end_position = np.array([pose.position.x, pose.position.y, pose.position.z])

        # For each movable joint, calculate its contribution to the Jacobian
        for col_idx, node in enumerate(movable_joints):
            frame = node.frame  # Get the frame from the node
            # Get parent frame
            # Ensure parent node exists before accessing frame
            parent_frame = self.parent_frame(frame)
            if node.parent is None:
                # Should not happen for non-root movable joints, but good practice
                continue

            # Get transformation from root to joint parent
            parent_transform = self._get_transformation_matrix(
                self._root_frame, req(parent_frame)
            )

            # Joint position in world coordinates
            joint_position = parent_transform[:3, 3]

            # Joint axis in world coordinates (apply rotation from parent transform)
            axis = node.rotary_axis.as_array()
            world_axis = np.dot(parent_transform[:3, :3], axis)

            # Linear velocity component is axis × (end_position - joint_position)
            position_diff = end_position - joint_position
            jacobian[:3, col_idx] = np.cross(world_axis, position_diff)

            # Angular velocity component is axis
            jacobian[3:, col_idx] = world_axis

        return jacobian

    def _raise_if_link_exists(self, frame: geometry.ReferenceFrame) -> None:
        if frame in self._links:
            raise ValueError(f"Frame '{frame}' already exists")

    def _raise_if_no_parent_exists(self, parent: geometry.ReferenceFrame) -> None:
        if parent not in self._links:
            raise ValueError(f"Parent frame '{parent}' does not exist")

    def _raise_if_no_link_exists(self, frame: geometry.ReferenceFrame) -> None:
        if frame not in self._links:
            raise ValueError(f"Frame '{frame}' does not exist")
