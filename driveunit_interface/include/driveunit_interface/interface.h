#pragma once

#include "driveunit_interface/message.h"

#include <Eigen/Dense>

namespace pisar::driveunit_interface {

struct DefaultResponse { bool ack; };

// Heartbeat
struct HeartbeatRequest {};

struct HeartbeatResponse {
    uint32_t time_alive_ms;
};

// Commands

struct CommandIdle {};

struct CommandFollowTrajectory
{
    float reference_time;
    std::vector<Eigen::Vector2f> trajectory;
};

struct CommandRotate
{
    float rotation_deg; // CCW positive
};

using CommandRequest = MessageSet<CommandIdle, CommandFollowTrajectory, CommandRotate>;
using CommandResponse = DefaultResponse;


using Request = MessageSet<HeartbeatRequest, CommandRequest>;
using Response = MessageSet<HeartbeatResponse, CommandResponse>;

};