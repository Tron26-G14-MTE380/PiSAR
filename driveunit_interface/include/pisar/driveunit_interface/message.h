#pragma once

#include <variant>

namespace pisar::driveunit_interface {

/**
 * @brief Represents a packet sent between the driveunit and main raspberry pi board. 
 * Implemented as a type safe union (variant) of possible commands/requests or responses.
 * 
 * @tparam Ts The set of possible commands that the message can be.
 */
template<typename... Ts>
using MessageSet = std::variant<Ts...>;

}