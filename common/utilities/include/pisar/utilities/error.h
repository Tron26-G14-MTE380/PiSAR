#pragma once

#include <system_error>
#include <array>
#include <string_view>

namespace pisar {

/**
 * @brief Compile-time lookup for error messages using a linear search.
 * @tparam TError Enum type representing error codes.
 * @tparam kSize Number of error codes.
 */
template <typename TError, size_t kSize>
class StaticErrorCategory : public std::error_category
{
public:
    using ErrorPair = std::pair<TError, std::string_view>;

private:
    std::string_view m_name;
    std::array<ErrorPair, kSize> m_error_messages;

public:
    explicit constexpr StaticErrorCategory(std::string_view name, std::array<ErrorPair, kSize> error_messages)
        : m_name(name), m_error_messages(error_messages) {}

    [[nodiscard]] const char* name() const noexcept override
    {
        return m_name.data();
    }

    [[nodiscard]] std::string message(int condition) const override
    {
        for (const auto& [code, msg] : m_error_messages)
        {
            if (static_cast<int>(code) == condition)
            {
                return std::string(msg);
            }
        }
        return "Unknown error";
    }
};

}

#define PISAR_DEFINE_ERROR_CATEGORY(EnumType, CategoryName, ...) \
    namespace detail { \
        constexpr std::array error_messages_##CategoryName = { __VA_ARGS__ }; \
    } \
    inline const auto& get_##CategoryName() \
    { \
        static constexpr StaticErrorCategory<EnumType, detail::error_messages_##CategoryName.size()> instance( \
            #EnumType, detail::error_messages_##CategoryName); \
        return instance; \
    } \
    inline std::error_code make_error_code(EnumType e) \
    { \
        return {static_cast<int>(e), get_##CategoryName()}; \
    }


#define PISAR_REGISTER_ERROR(EnumType) \
namespace std { \
    template <> struct is_error_code_enum<::EnumType> : true_type {}; \
}

