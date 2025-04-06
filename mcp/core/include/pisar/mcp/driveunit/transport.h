#pragma once

#include "pisar/driveunit_interface/codec.h"
#include "pisar/driveunit_interface/interface.h"

#include "pisar/mcp/driveunit/serial_uart.h"

#include "pisar/utilities/expected.h"
#include "pisar/utilities/error.h"

#include <Eigen/Dense>

#include <vector>
#include <mutex>
#include <optional>
#include <iostream>
#include <chrono>

namespace pisar::mcp {

/**
 * @brief Handles UART communication between the Raspberry Pi (master) and driveunit (slave).
 */
class DriveunitTransport {
private:
    SerialUart m_uart;           ///< Handles uart comms.
    std::mutex m_mutex;          ///< Mutex for thread-safe UART transactions.

public:
    enum class Error : uint8_t {
        kNone,              ///< No error, operation successful.
        kUartNotOpen,       ///< UART connection is closed.
        kEncodingError,     ///< Failed to encode the request.
        kWriteError,        ///< UART write operation failed.
        kReadTimeout,       ///< No response received within the timeout.
        kDecodingError,     ///< Response was received but failed decoding.
        kInvalidResponse    ///< Response type not expected.
    };

    /**
     * @brief Constructs the DriveunitTransport interface.
     * @param uart_device The UART device file (e.g., "/dev/serial0").
     * @param baud_rate Baud rate for UART communication.
     */
    inline explicit DriveunitTransport(const std::string_view uart_device = "/dev/ttyAMA0") : m_uart(uart_device) {}

    inline ~DriveunitTransport()
    {
        close();
    }

    /// @brief Returns the uart device name.
    [[nodiscard]] inline std::string_view device() { return m_uart.device(); }

    /**
     * @brief Opens the transport port (uart fd) for comms.
     *
     * @param baud_rate The baud rate to communicate at.
     * @return true if successful otherwise false.
     */
    inline bool open(int baud_rate = driveunit_interface::kUartSpeed)
    {
        return m_uart.open(baud_rate);
    }

    /// @brief Closes the transport (uart fd).
    inline void close()
    {
        m_uart.close();
    }

    /**
     * @brief Sends a request to the driveunit and waits for a response.
     *        Avoids conversion if `driveunit_interface::Request` is used directly.
     * @tparam TRequest Type of request.
     * @tparam TResponse Expected response type.
     * @param request The request data.
     * @return `rd::expected<TResponse, std::error_code>` Response or error.
     */
    template<typename TRequest, typename TResponse>
    [[nodiscard]]
    std::enable_if_t<!std::is_same_v<TRequest, driveunit_interface::Request>, rd::expected<TResponse, std::error_code>>
    sendRequest(const TRequest& request);

    /**
     * @brief Overload for sending a raw `driveunit_interface::Request` directly.
     * @param request The already encoded request.
     * @return `rd::expected<driveunit_interface::Response, std::error_code>` Response or error.
     */
    [[nodiscard]]
    inline rd::expected<driveunit_interface::Response, std::error_code>
    sendRequest(const driveunit_interface::Request& request)
    {
        return sendRequestInternal(request);
    }

    /**
     * @brief Sends a request to the driveunit with retry logic.
     * @tparam TRequest Type of request.
     * @tparam TResponse Expected response type.
     * @param request The request data.
     * @param retries Number of times to retry on failure.
     * @return `rd::expected<TResponse, std::error_code>` Response or error.
     */
    template<typename TRequest, typename TResponse>
    [[nodiscard]]
    std::enable_if_t<!std::is_same_v<TRequest, driveunit_interface::Request>, rd::expected<TResponse, std::error_code>>
    sendRequest(const TRequest& request, const unsigned int retries);

    /**
     * @brief Sends a raw request with retry logic.
     * @param request The already encoded request.
     * @param retries Number of times to retry on failure.
     * @return `rd::expected<driveunit_interface::Response, std::error_code>` Response or error.
     */
    [[nodiscard]]
    rd::expected<driveunit_interface::Response, std::error_code>
    sendRequest(const driveunit_interface::Request& request, const unsigned int retries);

private:

    /**
     * @brief Sends a command to the driveunit and waits for a response.
     * @tparam TRequest Type of request.
     * @tparam TResponse Expected response type.
     * @param request The command request to send.
     * @return std::optional<TResponse> The decoded response if successful, otherwise std::nullopt.
     */
    [[nodiscard]]
    rd::expected<driveunit_interface::Response, std::error_code>
    sendRequestInternal(const driveunit_interface::Request& request);

    /**
     * @brief Receives a response from the driveunit.
     * @tparam TResponse Expected response type.
     * @return std::optional<TResponse> The decoded response if successful.
     */
    [[nodiscard]]
    rd::expected<driveunit_interface::Response, std::error_code>
    receiveResponse(std::chrono::milliseconds timeout = std::chrono::milliseconds(5));
};

PISAR_DEFINE_ERROR_CATEGORY(DriveunitTransport::Error, TransportErrorCategory,
    std::make_pair(DriveunitTransport::Error::kNone, std::string_view("No error")),
    std::make_pair(DriveunitTransport::Error::kUartNotOpen, std::string_view("UART is not open")),
    std::make_pair(DriveunitTransport::Error::kEncodingError, std::string_view("Encoding failed")),
    std::make_pair(DriveunitTransport::Error::kWriteError, std::string_view("UART write failed")),
    std::make_pair(DriveunitTransport::Error::kReadTimeout, std::string_view("Read timeout")),
    std::make_pair(DriveunitTransport::Error::kDecodingError, std::string_view("Decoding failed")),
    std::make_pair(DriveunitTransport::Error::kInvalidResponse, std::string_view("Invalid Response"))
);

template<typename TRequest, typename TResponse>
[[nodiscard]]
std::enable_if_t<!std::is_same_v<TRequest, driveunit_interface::Request>, rd::expected<TResponse, std::error_code>>
DriveunitTransport::sendRequest(const TRequest& request)
{
    const auto result = sendRequestInternal(driveunit_interface::Request(request));
    if (result.has_value() == false)
    {
        return rd::unexpected(result.error());
    }

    if (auto response = std::get_if<TResponse>(&result.value()))
    {
        return rd::expected<TResponse, std::error_code>(*response);
    }

    return rd::unexpected(make_error_code(Error::kInvalidResponse));
}

template<typename TRequest, typename TResponse>
[[nodiscard]]
std::enable_if_t<!std::is_same_v<TRequest, driveunit_interface::Request>, rd::expected<TResponse, std::error_code>>
DriveunitTransport::sendRequest(const TRequest& request, const unsigned int retries)
{
    rd::expected<TResponse, std::error_code> last_error = rd::unexpected(make_error_code(Error::kNone));

    int64_t retries_left = retries;
    while (retries_left-- >= 0)
    {
        auto result = sendRequest<TRequest, TResponse>(request);
        if (result.has_value())
            return result;

        last_error = rd::unexpected(result.error());
    }

    return last_error;
}

inline rd::expected<driveunit_interface::Response, std::error_code>
DriveunitTransport::sendRequest(const driveunit_interface::Request& request, const unsigned int retries)
{
    rd::expected<driveunit_interface::Response, std::error_code> last_error = rd::unexpected(make_error_code(Error::kNone));

    int64_t retries_left = retries;
    while (retries_left-- >= 0)
    {
        auto result = sendRequest(request);
        if (result.has_value())
            return result;

        last_error = rd::unexpected(result.error());
    }

    return last_error;
}

} // namespace pisar::mcp

PISAR_REGISTER_ERROR(pisar::mcp::DriveunitTransport::Error);