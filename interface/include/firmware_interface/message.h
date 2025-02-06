#pragma once

#include "mpark/variant.hpp"

namespace pisar::interface {

/**
 * @brief Represents a packet sent between the MCU and main raspberry pi board. 
 * Implemented as a type safe union (variant) of possible commands/requests or responses.
 * 
 * @tparam Ts The set of possible commands that the message can be. Must be POD structs.
 */
template<typename... Ts>
class Message : public mpark::variant<Ts...> {
    using mpark::variant<Ts...>::variant;
};

}