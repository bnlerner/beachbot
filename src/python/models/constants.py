# The wheels need to spin slightly faster than a strict calculation of the wheel speed
# relative to tangential speed during turning. This is required due to wheel slippage
# and friction with the ground.
WHEEL_RESISTANCE_FACTOR = 0.5
# Max speed of the robot in m/s. 5 m/s is roughly 11 mph. Technically we can go faster
# than this but is limited for safety.
MAX_LINEAR_SPEED = 5.0
# Max angular speed the robot can turn in deg/s.
MAX_ANGULAR_SPEED = 90.0
# The max wheel torque (Nm) to apply to counterrotate the robot when otherwise
# stationary. When the robot is moving forward or backward, this is unnecessary. Without
# this compensation, we have to wait for integral windup to take affect.
TURNING_STATIC_FRICTION_TORQUE = 3.0
