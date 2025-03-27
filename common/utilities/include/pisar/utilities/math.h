#pragma once

#include <type_traits>
#include <numbers>

namespace pisar {

template<typename T>
inline constexpr T degToRad(T deg)
{
    return deg * std::numbers::pi_v<T> / static_cast<T>(180);
}

template<typename T>
inline constexpr T radToDeg(T deg)
{
    return deg * static_cast<T>(180) / std::numbers::pi_v<T>;
}

}