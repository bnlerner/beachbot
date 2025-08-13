#ifndef IPC_REGISTRY_H
#define IPC_REGISTRY_H

#include "core.h"
#include "messages.h"

// Dummy MotorLocation enum (adjust as needed)
enum class MotorLocation { FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT };
struct Motor { MotorLocation location; };

struct Channels {
    static ChannelSpec BODY_KINEMATICS;
    static ChannelSpec FRONT_CAMERA_IMAGE;
    static ChannelSpec FRONT_OBSTACLES;
    static ChannelSpec IMU;
    static ChannelSpec GNSS;
    static ChannelSpec MOTOR_CMD_FRONT_LEFT;
    static ChannelSpec MOTOR_CMD_FRONT_RIGHT;
    static ChannelSpec MOTOR_CMD_REAR_LEFT;
    static ChannelSpec MOTOR_CMD_REAR_RIGHT;
    static ChannelSpec MOTOR_VELOCITY_FRONT_LEFT;
    static ChannelSpec MOTOR_VELOCITY_FRONT_RIGHT;
    static ChannelSpec MOTOR_VELOCITY_REAR_LEFT;
    static ChannelSpec MOTOR_VELOCITY_REAR_RIGHT;
    static ChannelSpec REAR_OBSTACLES;
    static ChannelSpec STOP_MOTORS;
    static ChannelSpec TEST;
};

struct Requests {
    static RequestSpec NAVIGATE;
};

struct NodeIDs {
    static NodeID CAMERA;
    static NodeID GNSS;
    static NodeID IMU;
    static NodeID LOCALIZER;
    static NodeID MOTOR_CONTROL;
    static NodeID NAVIGATION;
    static NodeID RC;
    static NodeID TEST0;
    static NodeID TEST1;
    static NodeID UI;
};

ChannelSpec motor_command_channel(const Motor& motor);
ChannelSpec motor_velocity_channel(const Motor& motor);

#endif // IPC_REGISTRY_H
