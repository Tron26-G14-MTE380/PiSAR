#pragma once

#include <array>
#include <string_view>

namespace pisar {

namespace detail {

template <typename T>
constexpr auto typeNameImpl() {
#if defined(__clang__) || defined(__GNUC__)
    constexpr std::string_view function = __PRETTY_FUNCTION__;
    constexpr std::string_view prefix = "constexpr auto pisar::detail::typeNameImpl() [with T = ";
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

template <typename T>
constexpr auto simpleTypeNameImpl()
{
#if defined(__clang__) || defined(__GNUC__)
    constexpr std::string_view function = __PRETTY_FUNCTION__;
    constexpr std::string_view prefix = "constexpr auto pisar::detail::simpleTypeNameImpl() [with T = ";
    constexpr std::string_view suffix = "]";
#else
#error Unsupported compiler
#endif

    constexpr size_t start = prefix.size();
    constexpr size_t end = function.size() - suffix.size();
    constexpr std::string_view full_type = function.substr(start, end - start);

    // Cut at first '<' if present
    constexpr size_t angle_pos = full_type.find('<');
    constexpr std::string_view stripped = (angle_pos != std::string_view::npos)
                                            ? full_type.substr(0, angle_pos)
                                            : full_type;

    // Copy to array so it can be used as a compile-time literal
    std::array<char, stripped.size() + 1> result = {};
    for (size_t i = 0; i < stripped.size(); ++i)
        result[i] = stripped[i];
    result[stripped.size()] = '\0';

    return result;
}

template<typename T>
struct TypeNameImpl
{
    static constexpr auto value = typeNameImpl<T>();
};


template<typename T>
struct SimpleTypeNameImpl
{
    static constexpr auto value = simpleTypeNameImpl<T>();
};

}

template<typename T>
struct TypeName
{
    static constexpr std::string_view value = std::string_view(detail::TypeNameImpl<T>::value.data(), detail::TypeNameImpl<T>::value.size());
};


template<typename T>
struct SimpleTypeName
{
    static constexpr std::string_view value = std::string_view(detail::SimpleTypeNameImpl<T>::value.data(), detail::SimpleTypeNameImpl<T>::value.size());
};

}