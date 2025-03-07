#pragma once

#include <array>
#include <string_view>

namespace pisar::utilities {

namespace detail {

template <typename T>
constexpr auto typeNameImpl() {
#if defined(__clang__) || defined(__GNUC__)
    constexpr std::string_view function = __PRETTY_FUNCTION__;
    constexpr std::string_view prefix = "constexpr auto pisar::utilities::detail::typeNameImpl() [with T = ";
    constexpr std::string_view suffix = "]";
#else
#error Unsupported compiler
#endif

    constexpr size_t start = prefix.size();
    constexpr size_t end = function.size() - suffix.size();
    constexpr size_t length = end - start;

    // Store result in a static buffer to ensure it doesn't go out of scope
    std::array<char, length+1> name = {}; // +1 for null terminator

    // Fill buffer at compile-time
    for (size_t i = 0; i < length; ++i)
    {
        name[i] = function[start + i];
    }

    name[length] = '\0'; // Ensure null-termination
    return name; // Return pointer to static storage
}

template<typename T>
struct TypeNameImpl
{
    static constexpr auto value = typeNameImpl<T>();
};

}

template<typename T>
struct TypeName
{
    static constexpr std::string_view value = std::string_view(detail::TypeNameImpl<T>::value.data(), detail::TypeNameImpl<T>::value.size());
};

}