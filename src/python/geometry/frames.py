import enum


class ReferenceFrame(enum.Enum):
    """Reference Frames that the beachbot robot uses. Note that the vehicle frame is
    lifted from the Ublox Automotive vehicle frame definition and is defined as follows.

    28.6.4.2 Vehicle-Frame
    The vehicle-frame is a right-handed 3D Cartesian frame rigidly connected with the
    vehicle and is used to determine the attitude of the vehicle with respect to the
    local-level frame. It has the following features:
    • The origin (O) is the VRP in protocol versions less than 19.2, otherwise, is the
        origin of the IMU instrumental frame;
    • The x-axis points towards the front of the vehicle;
    • the y-axis points towards the right of the vehicle;
    • the z-axis completes the right-handed reference system by pointing down.

                                      UTM
                                       |
                                    VEHICLE
                                       |
                                      BODY
                                       |
                                -------+-------
                                |             |
                           FRONT_CAMERA   REAR_CAMERA

    Adds a robot arm frame coordinate system which is used describe the frames in each
    joint.

                                      BASE
                                       |
                                      ARM
                                       |
                                    FOREARM
                                       |
                                     HAND
    """

    # A world frame defined in UTM.
    UTM = "UTM"
    # The vehicle frame is the frame all incoming sensor data from the Ublox module is
    # defined in. It follows a Forward, Right and Down model similar to NED (global).
    # x-axis points toward the front of the robot and y-axis toward the right side of
    # the robot.
    VEHICLE = "VEHICLE"
    # The body frame is a convenience frame which defines the z-axis upward and the
    # x-axis pointed toward the front. Its origin is in the center of robot coplanar
    # with the surface the cooler rests on. Frame is rotated 180 degrees about the
    # VEHICLE frame x-axis (roll)
    BODY = "BODY"

    FRONT_CAMERA = "FRONT_CAMERA"
    REAR_CAMERA = "REAR_CAMERA"

    HAND = "HAND"
    FOREARM = "FOREARM"
    ARM = "ARM"
    BASE = "BASE"
    SHOULDER = "SHOULDER"

    def __repr__(self) -> str:
        return str(self)
