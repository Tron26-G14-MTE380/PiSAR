#pragma once

#include <type_traits>
#include <numbers>

namespace pisar::mcp {

template<typename T>
inline constexpr T deg_to_rad(T deg) 
{
    return deg * std::numbers::pi_v<T> / static_cast<T>(180);
} 

template<typename T>
inline constexpr T rad_to_deg(T deg) 
{
    return deg * static_cast<T>(180) / std::numbers::pi_v<T>;
}
 
}