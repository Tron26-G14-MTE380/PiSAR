#pragma once

#include "mpark/variant.hpp"

namespace pisar::interface {

template<typename... Ts>
class Message : public mpark::variant<Ts...> {
    using mpark::variant<Ts...>::variant;
};

}