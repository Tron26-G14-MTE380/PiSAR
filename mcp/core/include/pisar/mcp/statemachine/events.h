#pragma once

#include <variant>

namespace pisar::mcp {

struct EventTargetFound {};
struct EventTargetLost {};
struct EventReachedTarget {};
struct EventFoundLineWithTarget {};
struct EventReachedHome {};


using RobotEvents = std::variant<
    EventTargetFound,
    EventTargetLost,
    EventReachedTarget,
    EventFoundLineWithTarget,
    EventReachedHome
>;

}