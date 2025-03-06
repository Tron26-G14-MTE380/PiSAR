#pragma once

#include "pisar/driveunit_interface/message.h"

#include <Eigen/Dense>

#include <chrono>

namespace pisar::driveunit_interface {


constexpr size_t kSpiSpeed = 10'000'000;

struct DefaultResponse { bool ack; };

// Heartbeat
struct HeartbeatRequest {};

struct HeartbeatResponse
{
    std::chrono::duration<uint32_t, std::milli> time_alive;
};

// Commands

struct CommandIdle {};

struct CommandFollowTrajectory
{
    std::chrono::duration<float> reference_time;
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