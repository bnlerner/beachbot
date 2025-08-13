#include "registry.h"

ChannelSpec Channels::BODY_KINEMATICS{"vehicle_dynamics", 1024};
ChannelSpec Channels::FRONT_CAMERA_IMAGE{"front_camera_image", 750000};
ChannelSpec Channels::FRONT_OBSTACLES{"front_obstacles", 10000};
ChannelSpec Channels::IMU{"imu", 1024};
ChannelSpec Channels::GNSS{"gnss", 1024};
ChannelSpec Channels::MOTOR_CMD_FRONT_LEFT{"front_left_motor_command", 1024};
ChannelSpec Channels::MOTOR_CMD_FRONT_RIGHT{"front_right_motor_command", 1024};
ChannelSpec Channels::MOTOR_CMD_REAR_LEFT{"rear_left_motor_command", 1024};
ChannelSpec Channels::MOTOR_CMD_REAR_RIGHT{"rear_right_motor_command", 1024};
ChannelSpec Channels::MOTOR_VELOCITY_FRONT_LEFT{"front_left_motor_velocity", 1024};
ChannelSpec Channels::MOTOR_VELOCITY_FRONT_RIGHT{"front_right_motor_velocity", 1024};
ChannelSpec Channels::MOTOR_VELOCITY_REAR_LEFT{"rear_left_motor_velocity", 1024};
ChannelSpec Channels::MOTOR_VELOCITY_REAR_RIGHT{"rear_right_motor_velocity", 1024};
ChannelSpec Channels::REAR_OBSTACLES{"rear_obstacles", 10000};
ChannelSpec Channels::STOP_MOTORS{"stop_motors", 1024};
ChannelSpec Channels::TEST{"test", 1024};

RequestSpec Requests::NAVIGATE{"navigate"};

NodeID NodeIDs::CAMERA{"camera"};
NodeID NodeIDs::GNSS{"gnss"};
NodeID NodeIDs::IMU{"imu"};
NodeID NodeIDs::LOCALIZER{"localizer"};
NodeID NodeIDs::MOTOR_CONTROL{"motor_control"};
NodeID NodeIDs::NAVIGATION{"navigation"};
NodeID NodeIDs::RC{"rc"};
NodeID NodeIDs::TEST0{"test0"};
NodeID NodeIDs::TEST1{"test1"};
NodeID NodeIDs::UI{"ui"};

ChannelSpec motor_command_channel(const Motor& motor) {
    switch (motor.location) {
        case MotorLocation::FRONT_LEFT: return Channels::MOTOR_CMD_FRONT_LEFT;
        case MotorLocation::FRONT_RIGHT: return Channels::MOTOR_CMD_FRONT_RIGHT;
        case MotorLocation::REAR_LEFT: return Channels::MOTOR_CMD_REAR_LEFT;
        case MotorLocation::REAR_RIGHT: return Channels::MOTOR_CMD_REAR_RIGHT;
        default: return Channels::TEST;
    }
}

ChannelSpec motor_velocity_channel(const Motor& motor) {
    switch (motor.location) {
        case MotorLocation::FRONT_LEFT: return Channels::MOTOR_VELOCITY_FRONT_LEFT;
        case MotorLocation::FRONT_RIGHT: return Channels::MOTOR_VELOCITY_FRONT_RIGHT;
        case MotorLocation::REAR_LEFT: return Channels::MOTOR_VELOCITY_REAR_LEFT;
        case MotorLocation::REAR_RIGHT: return Channels::MOTOR_VELOCITY_REAR_RIGHT;
        default: return Channels::TEST;
    }
}

