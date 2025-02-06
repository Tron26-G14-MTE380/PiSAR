#pragma once

#include "firmware_interface/message.h"

namespace pisar::interface {

struct HeartbeatRequest {};

struct HeartbeatResponse {
    uint32_t time_alive;
};

using Request = Message<HeartbeatRequest>;
using Response = Message<HeartbeatResponse>;

};