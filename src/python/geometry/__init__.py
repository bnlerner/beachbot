from geometry import cartesian_objects, frames, math_helpers

ReferenceFrame = frames.ReferenceFrame
# Frames for beachbot robot
BODY = frames.ReferenceFrame.BODY
FRONT_CAMERA = frames.ReferenceFrame.FRONT_CAMERA
REAR_CAMERA = frames.ReferenceFrame.REAR_CAMERA
UTM = frames.ReferenceFrame.UTM
VEHICLE = frames.ReferenceFrame.VEHICLE
# Frames for robot arm
HAND = frames.ReferenceFrame.HAND
FOREARM = frames.ReferenceFrame.FOREARM
ARM = frames.ReferenceFrame.ARM
BASE = frames.ReferenceFrame.BASE

BaseAngleType = cartesian_objects.BaseAngleType
BaseVectorType = cartesian_objects.BaseVectorType

Position = cartesian_objects.Position
Orientation = cartesian_objects.Orientation
Direction = cartesian_objects.Direction
Rotation = cartesian_objects.Rotation
Velocity = cartesian_objects.Velocity
Pose = cartesian_objects.Pose
Polygon = cartesian_objects.Polygon
Twist = cartesian_objects.Twist
AngularVelocity = cartesian_objects.AngularVelocity
AngularAcceleration = cartesian_objects.AngularAcceleration
Acceleration = cartesian_objects.Acceleration

wrap_degrees = math_helpers.wrap_degrees
sign = math_helpers.sign
quaternion_to_euler = math_helpers.quaternion_to_euler
euler_to_quaternion = math_helpers.euler_to_quaternion
linear_ramp = math_helpers.linear_ramp
clip = math_helpers.clip
mean = math_helpers.mean


# Some common conversions.
VEH_TO_BODY_ROT = Rotation(UTM, 180, 0, 0)
