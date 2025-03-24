#pragma once

#include <tuple>
#include <cstdint>
#include <cstdio>
#include <array>
#include <span>
#include <functional>
#include <string_view>
#include <source_location>

#include "pisar/driveunit/sync.h"

#ifndef PISAR_DRIVEUNIT_LOGGING_ENABLED
#define PISAR_DRIVEUNIT_LOGGING_ENABLED 1
#endif

// maximum length of formatted log message
#ifndef PISAR_DRIVEUNIT_LOGGING_MAX_MESSAGE_LENGTH
#define PISAR_DRIVEUNIT_LOGGING_MAX_MESSAGE_LENGTH 512
#endif

namespace pisar::driveunit {

/**
 * @enum LogLevel
 * @brief Defines different log levels for filtering log messages.
 */
enum class LogLevel : uint8_t {
    kDebug = 0,
    kInfo,
    kWarn,
    kError
};

inline constexpr std::string_view logLevelName(LogLevel level)
{
    switch (level)
    {
        case LogLevel::kDebug: return "DEBUG";
        case LogLevel::kInfo: return "INFO";
        case LogLevel::kWarn: return "WARN";
        case LogLevel::kError: return "ERROR";
        default: return "UNKNOWN";
    }
}

namespace detail {
    inline constexpr std::string_view extractFilename(std::string_view path)
    {
        size_t pos = path.find_last_of("/\"");
        return (pos == std::string_view::npos) ? path : path.substr(pos + 1);
    }

    inline constexpr std::string_view extractFunctionName(std::string_view func_name)
    {
        // Find first '(' to remove parameters and return type
        size_t paren_pos = func_name.find('(');
        if (paren_pos != std::string_view::npos)
        {
            func_name = func_name.substr(0, paren_pos);
        }

        // Find last "::" to get only the function name
        size_t last_colon = func_name.rfind("::");
        if (last_colon != std::string_view::npos)
        {
            return func_name.substr(last_colon + 2); // Extract only function name
        }

        return func_name;
    }
}

/**
 * @brief Compile-time wrapper around std::source_location to store only the filename.
 */
struct SourceLocation {
    std::string_view file_name;
    std::string_view function_name;
    uint32_t line;
    uint32_t column;

    inline constexpr SourceLocation(std::source_location loc = std::source_location::current())
        : file_name(detail::extractFilename(loc.file_name())),
          function_name(detail::extractFunctionName(loc.function_name())),
          line(loc.line()),
          column(loc.column()) {}
};

/**
 * @brief Helper struct to capture log message and context information.
 */
template <typename... TArgs>
struct LogMessage {
    std::string_view format;
    std::tuple<std::decay_t<TArgs>...> args;
    SourceLocation loc;

    /**
     * @brief Constructs a log message that contains message and source information.
     * @param fmt The log message (format string).
     * @param as Arguments for formatting.
     * @param l The source location of the log.
     */
    explicit inline constexpr LogMessage(std::string_view fmt, TArgs&&... as, SourceLocation l = {})
        : format(fmt), args(std::make_tuple(static_cast<std::decay_t<TArgs>>(as)...)), loc(l) {}
};

/**
 * @brief Deduction guide for LogMessage.
 */
template <typename... TArgs>
LogMessage(std::string_view, TArgs&&...) -> LogMessage<TArgs...>;

/**
 * @brief Helper struct to capture log message and context information.
 */
template <typename... TArgs>
struct LogMessageWithLevel {
    LogLevel level;
    LogMessage<TArgs...> msg;

    /**
     * @brief Constructs a log message that contains message, source and level information.
     * @param level The log level of the message.
     * @param fmt The log message (format string).
     * @param as Arguments for formatting.
     * @param l The source location of the log.
     */
    explicit inline constexpr LogMessageWithLevel(LogLevel lvl, std::string_view fmt, TArgs&&... as,
        const SourceLocation l = std::source_location::current())
        : level(lvl), msg(fmt, std::forward<TArgs>(as)..., l) {}

    inline constexpr LogMessageWithLevel(LogLevel lvl, LogMessage<TArgs...>&& m)
        : level(lvl), msg(m) {}

    inline constexpr LogMessageWithLevel(LogLevel lvl, const LogMessage<TArgs...>& m)
        : level(lvl), msg(m) {}
};

/**
 * @brief Deduction guide for LogMessageWithLevel.
 */
template <typename... TArgs>
LogMessageWithLevel(LogLevel, std::string_view, TArgs&&...) -> LogMessageWithLevel<TArgs...>;


/**
 * @class Logger
 * @brief Lightweight logging system for embedded environments.
 */
class Logger {
public:
    using OutputFunction = void(*)(std::string_view);

private:
    LogLevel m_log_level;
    OutputFunction m_output_fn;
    Mutex m_mutex;


public:

    explicit inline Logger() : Logger(LogLevel::kInfo, nullptr) {}

    explicit inline Logger(LogLevel level, OutputFunction output_fn) :
        m_log_level(level), m_output_fn(output_fn) {}

    /**
     * @brief Sets the logging level to filter messages.
     * @param level The minimum log level to be displayed.
     */
    inline void setLogLevel(LogLevel level)
    {
        Lock<Mutex> lock(m_mutex);

        m_log_level = level;
    }

    /**
     * @brief Sets the output function for log messages.
     * @param output_fn Function taking a std::string_view for logging output.
     */
    inline void setOutput(OutputFunction output_fn)
    {
        Lock<Mutex> lock(m_mutex);

        m_output_fn = output_fn;
    }

    /**
     * @brief Logs a message with the given log level and contextual information.
     * @param level The log level of the message.
     * @param file Source file name.
     * @param line Line number.
     * @param func Function name.
     * @param msg The log message (format string).
     * @param args Arguments for formatting.
     */
    template <typename... TArgs>
    constexpr void log(const LogMessageWithLevel<TArgs...>& log_msg)
    {
        Lock<Mutex> lock(m_mutex);

        if (!m_output_fn || log_msg.level < m_log_level)
        {
            return;
        }

        // Fixed-size buffer to avoid dynamic allocation
        std::array<char, PISAR_DRIVEUNIT_LOGGING_MAX_MESSAGE_LENGTH> buffer{};
        int written = std::apply(
            [&](auto&&... args) {
                return std::snprintf(buffer.data(), buffer.size(), log_msg.msg.format.data(), args...);
            },
            log_msg.msg.args);

        if (written > 0 && static_cast<size_t>(written) < buffer.size())
        {
            std::array<char, PISAR_DRIVEUNIT_LOGGING_MAX_MESSAGE_LENGTH> log_buffer{};
            std::snprintf(
                log_buffer.data(), log_buffer.size(),
                "[%.*s:%.*s:%u:%u][%s]: %s",
                static_cast<int>(log_msg.msg.loc.file_name.size()), log_msg.msg.loc.file_name.data(),
                static_cast<int>(log_msg.msg.loc.function_name.size()), log_msg.msg.loc.function_name.data(),
                log_msg.msg.loc.line, log_msg.msg.loc.column,
                logLevelName(log_msg.level).data(), buffer.data()
            );

            m_output_fn(std::string_view(log_buffer.data()));
        }
        else
        {
            m_output_fn("LOGGING ERROR: Formatting failed\n"); // Debugging output
        }
    }

    template <typename... TArgs>
    inline constexpr void debug(const LogMessage<TArgs...>& log_msg)
    {
        log(LogMessageWithLevel(LogLevel::kDebug, log_msg));
    }

    template <typename... TArgs>
    inline constexpr void info(const LogMessage<TArgs...>& log_msg)
    {
        log(LogMessageWithLevel(LogLevel::kInfo, log_msg));
    }

    template <typename... TArgs>
    inline constexpr void warn(const LogMessage<TArgs...>& log_msg)
    {
        log(LogMessageWithLevel(LogLevel::kWarn, log_msg));
    }

    template <typename... TArgs>
    inline constexpr void error(const LogMessage<TArgs...>& log_msg)
    {
        log(LogMessageWithLevel(LogLevel::kError, log_msg));
    }
};


#if PISAR_DRIVEUNIT_LOGGING_ENABLED == 1

extern Logger g_uart_logger;

void waitForSerialConnection();
void initLogging(unsigned int baud, LogLevel level, bool wait_for_connection);

#else

// Inlined empty functions
inline void waitForSerialConnection() {}
inline void initLogging(unsigned int baud, LogLevel level, bool wait_for_connection) {}

#endif

}

#if PISAR_DRIVEUNIT_LOGGING_ENABLED == 1

#define PISAR_LOG(level, msg, ...)      pisar::driveunit::g_uart_logger.log(pisar::driveunit::LogMessageWithLevel(level, msg, ##__VA_ARGS__))
#define PISAR_LOG_DEBUG(msg, ...)       pisar::driveunit::g_uart_logger.debug(pisar::driveunit::LogMessage(msg, ##__VA_ARGS__))
#define PISAR_LOG_INFO(msg, ...)        pisar::driveunit::g_uart_logger.info(pisar::driveunit::LogMessage(msg, ##__VA_ARGS__))
#define PISAR_LOG_WARN(msg, ...)        pisar::driveunit::g_uart_logger.warn(pisar::driveunit::LogMessage(msg, ##__VA_ARGS__))
#define PISAR_LOG_ERROR(msg, ...)       pisar::driveunit::g_uart_logger.error(pisar::driveunit::LogMessage(msg, ##__VA_ARGS__))

#else

#define __PISAR_LOG_VOID do {} while(0)
#define PISAR_LOG(level, msg, ...)      __PISAR_LOG_VOID
#define PISAR_LOG_DEBUG(msg, ...)       __PISAR_LOG_VOID
#define PISAR_LOG_INFO(msg, ...)        __PISAR_LOG_VOID
#define PISAR_LOG_WARN(msg, ...)        __PISAR_LOG_VOID
#define PISAR_LOG_ERROR(msg, ...)       __PISAR_LOG_VOID

#endif