# URDF Configuration Files

This directory contains Universal Robot Description Format (URDF) files that define the kinematic structure of various robot arms used for testing and demonstration purposes in the Beachbot project.

## Available Robot Arm Configurations

### 3-DOF Robot Arm (`3dof_arm.urdf`)
A simple 3-degree-of-freedom robot arm with:
- A rotating base (shoulder)
- An elbow joint
- A wrist joint

### 4-DOF Robot Arm (`4dof_arm.urdf`)
A 4-degree-of-freedom robot arm with:
- A rotating base (shoulder)
- An elbow joint
- A wrist joint
- A hand rotation joint

### 5-DOF Robot Arm (`5dof_arm.urdf`)
A 5-degree-of-freedom robot arm with:
- A rotating base (shoulder)
- An upper arm joint
- A forearm (elbow) joint
- A wrist roll joint
- A hand rotation joint

## Usage

These URDF files are automatically loaded by the `FrameTreeFixtureFactory` class in the kinematics module to create frame tree fixtures for testing and demonstration purposes.

### How to Access in Code

```python
from kinematics.frame_tree_fixtures import FrameTreeFixtureFactory, RobotArmType

# Create a 3-DOF robot arm frame tree
frame_tree_3dof = FrameTreeFixtureFactory.create_3dof_arm()

# Create a 4-DOF robot arm frame tree
frame_tree_4dof = FrameTreeFixtureFactory.create_4dof_arm()

# Create a 5-DOF robot arm frame tree
frame_tree_5dof = FrameTreeFixtureFactory.create_5dof_arm()
```

### Running the Example

You can see these robot arm configurations in action using the example script:

```bash
# Create and demonstrate a 3-DOF robot arm
python src/python/examples/urdf_frame_tree_example.py --dof 3

# Create and demonstrate a 4-DOF robot arm (default)
python src/python/examples/urdf_frame_tree_example.py

# Create and demonstrate a 5-DOF robot arm
python src/python/examples/urdf_frame_tree_example.py --dof 5
```

## Adding New Configurations

To add a new robot arm configuration:

1. Create a new URDF file in this directory with a descriptive name (e.g., `6dof_arm.urdf`)
2. Update the `RobotArmType` enum in the `FrameTreeFixtureFactory` class to include the new type
3. Add a helper method to the factory for creating the new type

The URDF file should follow the standard format with links and joints properly defined with appropriate visual elements, origins, axes, and limits. 
