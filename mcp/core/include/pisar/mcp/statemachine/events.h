#pragma once

#include <variant>

namespace pisar::mcp {

struct EventStartSearchAndRescue {};
struct EventTargetFound {};
struct EventTargetLost {};
struct EventReachedTarget {};
struct EventPickedUpTarget {};
struct EventFoundLineWithTarget {};
struct EventReachedHome {};
struct EventDroppedTarget {};

using RobotEvent = std::variant<
    EventStartSearchAndRescue,
    EventTargetFound,
    EventTargetLost,
    EventReachedTarget,
    EventPickedUpTarget,
    EventFoundLineWithTarget,
    EventReachedHome,
    EventDroppedTarget
>;

}