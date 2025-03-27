#pragma once

#include "pisar/driveunit_interface/message.h"
#include "pisar/driveunit_interface/codec.h"
#include "pisar/driveunit_interface/zpp_support.h"

#include <Eigen/Dense>
#include <zpp_bits/zpp_bits.h>

#include <chrono>

namespace pisar::driveunit_interface {

constexpr size_t kUartSpeed = 1'500'000;;
constexpr size_t kMaxRequestPacketSize = 256;
constexpr size_t kMaxResponsePacketSize = 64;

struct DefaultResponse
{
    using serialize = zpp::bits::members<1>;

    bool ack;
};

// Heartbeat
struct HeartbeatRequest
{
    using serialize = zpp::bits::members<0>;
};

struct HeartbeatResponse
{
    using serialize = zpp::bits::members<1>;

    std::chrono::duration<uint32_t, std::milli> time_alive;
};

// Command Status

struct CommandStatusRequest
{
    using serialize = zpp::bits::members<0>;
};

struct CommandStatusResponse
{
    using serialize = zpp::bits::members<1>;
    bool busy;
};

// Commands

struct CommandIdle
{
    using serialize = zpp::bits::members<0>;
};

struct CommandHardStop
{
    using serialize = zpp::bits::members<0>;
};


struct CommandFollowTrajectory
{
    using serialize = zpp::bits::members<2>;
    using ReferenceTimeT = std::chrono::microseconds;

    uint32_t reference_time; // Microseconds
    std::vector<Eigen::Vector2f> trajectory;
};

struct CommandGoToTarget
{
    using serialize = zpp::bits::members<2>;
    using ReferenceTimeT = std::chrono::microseconds;

    uint32_t reference_time; // Microseconds
    Eigen::Vector2f target_position;
};

struct CommandRotate
{
    using serialize = zpp::bits::members<1>;

    float rotation_deg; // CCW positive
};

struct CommandSetGripper
{
    using serialize = zpp::bits::members<1>;

    bool open;
};

using CommandRequest = MessageSet<CommandIdle, CommandHardStop, CommandFollowTrajectory, CommandGoToTarget, CommandRotate, CommandSetGripper>;
using CommandResponse = DefaultResponse;


using Request = MessageSet<HeartbeatRequest, CommandStatusRequest, CommandRequest>;
using Response = MessageSet<HeartbeatResponse, CommandStatusResponse, CommandResponse>;

using RequestEncoder = PacketEncoder<Request, kMaxRequestPacketSize>;

template<size_t tkPacketQueueSize>
using RequestDecoder = PacketDecoder<Request, kMaxRequestPacketSize, tkPacketQueueSize>;


using ResponseEncoder = PacketEncoder<Response, kMaxResponsePacketSize>;
template<size_t tkPacketQueueSize>
using ResponseDecoder = PacketDecoder<Response, kMaxResponsePacketSize, tkPacketQueueSize>;


};