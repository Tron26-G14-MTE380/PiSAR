#pragma once

#include "driveunit_interface/message.h"

namespace pisar::driveunit_interface {

struct HeartbeatRequest {};

struct HeartbeatResponse {
    uint32_t time_alive_ms;
};

using Request = Message<HeartbeatRequest>;
using Response = Message<HeartbeatResponse>;

};