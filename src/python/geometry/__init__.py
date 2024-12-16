from geometry import cartesian_objects, frames, math_helpers

UTM = frames.ReferenceFrame.UTM
VEHICLE = frames.ReferenceFrame.VEHICLE
BODY = frames.ReferenceFrame.BODY

BaseAngleType = cartesian_objects.BaseAngleType
BaseVectorType = cartesian_objects.BaseVectorType

Position = cartesian_objects.Position
Orientation = cartesian_objects.Orientation
Direction = cartesian_objects.Direction
Rotation = cartesian_objects.Rotation
Velocity = cartesian_objects.Velocity
Pose = cartesian_objects.Pose
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
