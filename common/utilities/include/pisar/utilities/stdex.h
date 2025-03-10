#pragma once

namespace pisar {

template <typename... Ts>
struct Overloaded : Ts...
{
    using Ts::operator()...;
};
template <typename... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;

template <typename T>
concept CIsChronoDuration = std::is_base_of_v<std::chrono::duration<typename T::rep, typename T::period>, T>;

}