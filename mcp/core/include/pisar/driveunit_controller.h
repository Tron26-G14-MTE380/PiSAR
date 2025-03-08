#pragma once

#include "pisar/driveunit_interface/codec.h"
#include "pisar/driveunit_interface/interface.h"

#include <wiringPiSPI.h>

#include <Eigen/Dense>

#include <vector>
#include <mutex>
#include <optional>

namespace pisar::mcp {

/**
 * @brief Handles SPI communication between the Raspberry Pi (master) and driveunit (slave).
 */
class DriveunitController {
private:
    int m_spi_channel;          ///< SPI channel (0 or 1).
    std::mutex m_mutex;         ///< Mutex for thread-safe SPI transactions.

public:
    /**
     * @brief Constructs the DriveunitController interface.
     * @param spi_channel The SPI channel (0 or 1).
     */
    inline explicit DriveunitController(int spi_channel)
        : m_spi_channel(spi_channel)
    {
        wiringPiSPISetupMode(spi_channel, driveunit_interface::kSpiSpeed, 1);
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
        std::scoped_lock lock(m_mutex);

        // Encode the request directly into a buffer
        std::array<std::byte, driveunit_interface::RequestEncoder::kMaxEncodedPacketSize> send_buffer{};

        driveunit_interface::RequestEncoder encoder;
        auto encoded = encoder.encode(driveunit_interface::Request(request), std::span(send_buffer));

        if (!encoded)
        {
            return std::nullopt;
        }

        // Send the encoded request over SPI
        std::vector<uint8_t> tx_data(reinterpret_cast<uint8_t*>(encoded->data()),
                                     reinterpret_cast<uint8_t*>(encoded->data()) + encoded->size());

        int ret = wiringPiSPIDataRW(m_spi_channel, tx_data.data(), tx_data.size());
        if (ret < 0)
        {
            std::cout << "Error transmitting SPI data: " << errno << std::endl;
            return std::nullopt;
        }

        if (ret != tx_data.size())
        {
            std::cout << "Could not transmit all data: " << ret << std::endl;
            return std::nullopt;
        }

        // Step 1: Read response size (1 byte)
        uint8_t response_size = 0;
        wiringPiSPIDataRW(m_spi_channel, &response_size, 1);

        if (response_size == 0) // Sanity check
        {
            return std::nullopt;
        }

        // Step 2: Read the full response
        std::array<std::byte, driveunit_interface::ResponseEncoder::kMaxEncodedPacketSize> response_buffer;
        wiringPiSPIDataRW(m_spi_channel, reinterpret_cast<uint8_t*>(response_buffer.data()), response_buffer.size());

        // Decode response
        driveunit_interface::ResponseDecoder<1> decoder;
        decoder.submit(std::span(response_buffer.data(), response_size));

        const auto response = decoder.query();

        if (!response)
        {
            return std::nullopt;
        }

        // Extract and return expected response type
        if (auto result = std::get_if<TResponse>(&(*response)))
        {
            return *result;
        }

        return std::nullopt;
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
        auto response = sendRequest<driveunit_interface::CommandIdle, driveunit_interface::DefaultResponse>(driveunit_interface::CommandIdle{});
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
        auto response = sendRequest<driveunit_interface::CommandFollowTrajectory, driveunit_interface::DefaultResponse>(command);
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
        auto response = sendRequest<driveunit_interface::CommandRotate, driveunit_interface::DefaultResponse>(command);
        return response && response->ack;
    }
};

} // namespace pisar::mcp
