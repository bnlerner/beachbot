import enum


class GPSQuality(enum.Enum):
    """GPS position quality. Typically position accuracy can be improved by using
    carrier phase range solutions. Improved quality can reduce GPS positioning error
    from meters to centimeter in accuracy.

    Autonomous indicates no carrier phase range solution is being used and is the
    typical type of position estimate used by GPS devices without a separate base
    station.

    RTK (Real time kinematics) float solution indicates that there are floating
    ambiguities in the number of whole wavelengths emitted from the satellite to the
    receiver.

    RTK Fix float solution indicates the number of whole wavelengths is known and this
    is the most accurate GPS quality indicator with precision down to the single
    centimeter.
    """

    AUTONOMOUS = 0
    RTK_FLOAT = 1
    RTK_FIX = 2


class SensorFusionMode(enum.Enum):
    """Modes the Ublox sensor fusion can be in.

    0: Initialization mode: receiver is initializing some unknown values required for
        doing sensor fusion
    1: Fusion mode: GNSS and sensor data are used for navigation solution computation
    2: Suspended fusion mode: sensor fusion is temporarily disabled due to e.g. invalid
        sensor data or detected ferry
    3: Disabled fusion mode: sensor fusion is permanently disabled until receiver reset
        due e.g. to sensor error.
    """

    INITIALIZING = 0
    FUSION = 1
    SUSPENDED = 2
    DISABLED = 3


class SensorStatus(enum.Enum):
    """Initialization status of the sensor."""

    OFF = 0
    INITIALIZING = 1
    INITIALIZED = 2
    IMU_MOUNT_INITIALIZED = 3


class FixType(enum.Enum):
    """Fix of the Ublox GPS sensor."""

    NO_FIX = 0
    DEAD_RECKONING = 1
    FIX_2D = 2
    FIX_3D = 3
    GNSS_W_DEAD_RECKONING = 4
    TIME_ONLY = 5
