import geometry
import pytest

from kinematics import joint_nodes, kinematic_tree


@pytest.fixture
def tree() -> kinematic_tree.KinematicTree:
    t = kinematic_tree.KinematicTree()
    t.add_rotary_link(
        geometry.SHOULDER,
        geometry.Position(geometry.BASE, 0, 0, 0.3),
        geometry.Orientation(geometry.BASE, 0, 0, 30),
        geometry.Direction.unit_z(geometry.SHOULDER),
    )
    t.add_rotary_link(
        geometry.ARM,
        geometry.Position(geometry.SHOULDER, 0.2, 0, 0),
        geometry.Orientation(geometry.SHOULDER, 0, -40, 0),
        geometry.Direction.unit_x(geometry.ARM),
    )
    t.add_rotary_link(
        geometry.FOREARM,
        geometry.Position(geometry.ARM, 0.7, 0, 0),
        geometry.Orientation(geometry.ARM, 0, 50, 0),
        geometry.Direction.unit_x(geometry.FOREARM),
    )

    return t


def test_add_stationary_link() -> None:
    """Test adding a stationary link to the tree with default parameters."""
    tree = kinematic_tree.KinematicTree()
    assert _base_tree_is_valid(tree)
    tree.add_stationary_link(
        geometry.ARM,
        geometry.Position.zero(geometry.BASE),
        geometry.Orientation.zero(geometry.BASE),
    )

    # Check that the link was added correctly
    assert geometry.ARM in tree.frames
    arm_node = tree.get_node(geometry.ARM)
    assert arm_node.joint_type == joint_nodes.JointType.FIXED
    assert arm_node.frame == geometry.ARM
    assert arm_node.parent and arm_node.parent.frame == geometry.BASE


def test_add_rotary_link() -> None:
    """Test adding a rotary link to the tree with default parameters."""
    tree = kinematic_tree.KinematicTree()
    assert _base_tree_is_valid(tree)
    tree.add_rotary_link(
        geometry.ARM,
        geometry.Position.zero(geometry.BASE),
        geometry.Orientation.zero(geometry.BASE),
        geometry.Direction.unit_x(geometry.ARM),
    )

    assert geometry.ARM in tree.frames
    arm_node = tree.get_node(geometry.ARM)
    assert arm_node.joint_type == joint_nodes.JointType.ROTARY
    assert arm_node.frame == geometry.ARM
    assert arm_node.parent and arm_node.parent.frame == geometry.BASE


def test_frame_lineage(tree: kinematic_tree.KinematicTree) -> None:
    """Test that the frame lineage is returned correctly."""
    lineage = tree.frame_lineage(geometry.ARM)
    assert geometry.BASE in lineage
    assert geometry.ARM in lineage
    assert geometry.SHOULDER in lineage
    assert len(lineage) == 3


def test_update_node_state(tree: kinematic_tree.KinematicTree) -> None:
    """Test setting multiple joint values at once."""
    target_ori = geometry.Orientation(geometry.ARM, 1, 2, 3)
    target_vel = geometry.Velocity(geometry.FOREARM, 1, 2, 3)
    target_accel = geometry.Acceleration(geometry.FOREARM, 4, 5, 6)
    target_ang_vel = geometry.AngularVelocity(geometry.FOREARM, 7, 8, 9)
    target_ang_accel = geometry.AngularAcceleration(geometry.FOREARM, 10, 11, 12)
    tree.update_node_state(
        geometry.FOREARM,
        orientation=target_ori,
        velocity=target_vel,
        acceleration=target_accel,
        angular_velocity=target_ang_vel,
        angular_acceleration=target_ang_accel,
    )

    # Check that values were set correctly
    arm_node = tree.get_node(geometry.FOREARM)
    assert arm_node.orientation == target_ori
    assert arm_node.velocity == target_vel
    assert arm_node.acceleration == target_accel
    assert arm_node.angular_velocity == target_ang_vel
    assert arm_node.angular_acceleration == target_ang_accel


def test_downward_transform(tree: kinematic_tree.KinematicTree) -> None:
    """Tests a simple version of a forward kinematics transform."""
    p_base = geometry.Position(geometry.BASE, 1.0, 0, 0)
    shoulder_ori = geometry.Orientation(geometry.SHOULDER, 0, 0, 30)
    p_shoulder = tree.transform(p_base, geometry.SHOULDER)

    assert p_shoulder.frame == geometry.SHOULDER
    assert p_shoulder.x == p_base.x * shoulder_ori.cos("yaw")
    assert p_shoulder.y == -p_base.x * shoulder_ori.sin("yaw")
    assert p_shoulder.z == 0.3


def test_upward_transform(tree: kinematic_tree.KinematicTree) -> None:
    """Tests a simple version of a forward kinematics transform."""
    p_shoulder = geometry.Position(geometry.SHOULDER, 1.0, 0, 0)
    shoulder_ori = geometry.Orientation(geometry.SHOULDER, 0, 0, 30)
    p_base = tree.transform(p_shoulder, geometry.BASE)

    assert p_base.frame == geometry.BASE
    assert p_base.x == p_shoulder.x * shoulder_ori.cos("yaw")
    assert p_base.y == p_shoulder.x * shoulder_ori.sin("yaw")
    assert p_base.z == 0.3


def _base_tree_is_valid(base_tree: kinematic_tree.KinematicTree) -> bool:
    """Returns True if the base tree is valid, False otherwise."""
    base_frame_available = geometry.BASE in base_tree.frames
    base_node = base_tree.get_node(geometry.BASE)
    base_frame_is_fixed = base_node.joint_type == joint_nodes.JointType.FIXED
    return base_frame_available and base_frame_is_fixed
