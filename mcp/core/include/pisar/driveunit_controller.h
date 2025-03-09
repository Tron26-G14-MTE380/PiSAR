#pragma once

#include "pisar/driveunit_interface/codec.h"
#include "pisar/driveunit_interface/interface.h"

#include <wiringSerial.h>
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
class DriveunitController {
private:
    int m_uart_fd;               ///< UART file descriptor.
    std::mutex m_mutex;          ///< Mutex for thread-safe UART transactions.

public:
    /**
     * @brief Constructs the DriveunitController interface.
     * @param uart_device The UART device file (e.g., "/dev/serial0").
     * @param baud_rate Baud rate for UART communication.
     */
    inline explicit DriveunitController() {}

    bool open(const std::string_view uart_device)
    {
        m_uart_fd = serialOpen(uart_device.data(), driveunit_interface::kUartSpeed);
        if (m_uart_fd < 0)
        {
            return false;
        }

        return true;
    }

    /**
     * @brief Sends a command to the driveunit and waits for a response.
     * @tparam TRequest Type of request.
     * @tparam TResponse Expected response type.
     * @param request The command request to send.
     * @return std::optional<TResponse> The decoded response if successful, otherwise std::nullopt.
     */
    template<typename TRequest, typename TResponse>
    std::optional<TResponse> sendRequest(const TRequest& request)
    {
        if (m_uart_fd < 0)
        {
            std::cerr << "UART not initialized!" << std::endl;
            return std::nullopt;
        }

        std::scoped_lock lock(m_mutex);

        // Encode the request directly into a buffer
        std::array<std::byte, driveunit_interface::RequestEncoder::kMaxEncodedPacketSize> send_buffer{};

        driveunit_interface::RequestEncoder encoder;
        auto encoded = encoder.encode(driveunit_interface::Request(request), std::span(send_buffer));

        if (!encoded)
        {
            std::cerr << "Failed to encode request" << std::endl;
            return std::nullopt;
        }

        // Write request to UART
        write(m_uart_fd, reinterpret_cast<const char*>(encoded->data()), encoded->size());

        // Read response from UART
        return receiveResponse<TResponse>();
    }

    /**
     * @brief Receives a response from the driveunit.
     * @tparam TResponse Expected response type.
     * @return std::optional<TResponse> The decoded response if successful.
     */
    template<typename TResponse>
    std::optional<TResponse> receiveResponse()
    {
        driveunit_interface::ResponseDecoder<1> decoder;
        auto start_time = std::chrono::steady_clock::now();

        while (true)
        {
            // Check for timeout
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - start_time)
                                    .count();

            if (elapsed_time > 5)
            {
                std::cerr << "UART receive timeout!" << std::endl;
                return std::nullopt;
            }

            // Read available data
            while (serialDataAvail(m_uart_fd) > 0)
            {
                uint8_t byte = serialGetchar(m_uart_fd);
                decoder.submit(std::span(reinterpret_cast<std::byte*>(&byte), 1));

                // Reset timeout on new data
                start_time = std::chrono::steady_clock::now();
            }

            // Try to extract a packet
            auto response = decoder.query();
            if (response)
            {
                if (auto result = std::get_if<TResponse>(&(*response)))
                {
                    return *result;
                }
                else
                {
                    std::cerr << "Invalid response type!" << std::endl;
                    return std::nullopt;
                }
            }
        }
    }

    /**
     * @brief Sends a heartbeat request.
     * @return std::optional<HeartbeatResponse> The heartbeat response if successful, otherwise std::nullopt.
     */
    inline std::optional<driveunit_interface::HeartbeatResponse> sendHeartbeat()
    {
        return sendRequest<driveunit_interface::HeartbeatRequest, driveunit_interface::HeartbeatResponse>(driveunit_interface::HeartbeatRequest{});
    }

    /**
     * @brief Sends a command to idle.
     * @return true if the command was acknowledged, false otherwise.
     */
    inline bool sendIdleCommand()
    {
        auto response = sendRequest<driveunit_interface::CommandIdle, driveunit_interface::CommandResponse>(driveunit_interface::CommandIdle{});
        return response && response->ack;
    }

    /**
     * @brief Sends a trajectory to the driveunit.
     * @param reference_time The time reference for trajectory execution.
     * @param trajectory The list of waypoints (Eigen::Vector2f).
     * @return true if the command was acknowledged, false otherwise.
     */
    inline bool sendTrajectoryCommand(std::chrono::duration<float> reference_time, const std::vector<Eigen::Vector2f>& trajectory)
    {
        driveunit_interface::CommandFollowTrajectory command{reference_time, trajectory};
        auto response = sendRequest<driveunit_interface::CommandFollowTrajectory, driveunit_interface::CommandResponse>(command);
        return response && response->ack;
    }

    /**
     * @brief Sends a rotation command.
     * @param degrees The rotation in degrees (CCW positive).
     * @return true if the command was acknowledged, false otherwise.
     */
    inline bool sendRotateCommand(float degrees)
    {
        driveunit_interface::CommandRotate command{degrees};
        auto response = sendRequest<driveunit_interface::CommandRotate, driveunit_interface::CommandResponse>(command);
        return response && response->ack;
    }
};

} // namespace pisar::mcp
