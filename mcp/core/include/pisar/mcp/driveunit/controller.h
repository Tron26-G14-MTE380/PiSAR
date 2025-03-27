#pragma once

#include "pisar/mcp/driveunit/transport.h"

namespace pisar::mcp {

/**
 * @brief Provides an interface to control the driveunit synchronously.
 *
 */
class DriveunitController {
private:
    std::reference_wrapper<DriveunitTransport> m_transport;

public:
    /**
     * @brief Construct a new Driveunit Controller object
     *
     * @param transport The driveunit interface transport used for comms.
     */
    inline DriveunitController(DriveunitTransport& transport) : m_transport(transport) {}

    /**
     * @brief Sends a command to the driveunit and waits for a response.
     * @tparam TRequest Type of request.
     * @tparam TResponse Expected response type.
     * @param request The command request to send.
     * @return `rd::expected<TResponse, std::error_code>` Response or error.
     */
    template<typename TRequest, typename TResponse>
    inline rd::expected<TResponse, std::error_code> sendRequest(const TRequest& request)
    {
        return m_transport.get().template sendRequest<TRequest, TResponse>(request);
    }

    /**
     * @brief Sends a command to the driveunit and waits for a response.
     * @param request The command request to send.
     * @return `rd::expected<driveunit_interface::Response, std::error_code>` Response or error.
     */
    [[nodiscard]] inline rd::expected<driveunit_interface::Response, std::error_code>
    sendRequest(const driveunit_interface::Request& request)
    {
        return m_transport.get().sendRequest(request);
    }

    /**
     * @brief Sends a heartbeat request.
     * @return std::optional<HeartbeatResponse> The heartbeat response if successful, otherwise std::nullopt.
     */
    inline rd::expected<driveunit_interface::HeartbeatResponse, std::error_code> sendHeartbeat()
    {
        return sendRequest<driveunit_interface::HeartbeatRequest, driveunit_interface::HeartbeatResponse>(driveunit_interface::HeartbeatRequest{});
    }

    /**
     * @brief Sends a command to idle.
     * @return true if the command was acknowledged, false otherwise.
     */
    inline rd::expected<bool, std::error_code> sendIdleCommand()
    {
        auto response = sendRequest<driveunit_interface::CommandIdle, driveunit_interface::CommandResponse>(driveunit_interface::CommandIdle{});
        return response.transform([](driveunit_interface::CommandResponse response) { return response.ack; });
    }

    /**
     * @brief Sends a trajectory to the driveunit.
     * @param reference_time The time reference for trajectory execution.
     * @param trajectory The list of waypoints (Eigen::Vector2f).
     * @return true if the command was acknowledged, false otherwise.
     */
    inline rd::expected<bool, std::error_code> sendTrajectoryCommand(
        driveunit_interface::CommandFollowTrajectory::ReferenceTimeT reference_time,
        const std::span<const Eigen::Vector2f>& trajectory
    )
    {
        driveunit_interface::CommandFollowTrajectory command{reference_time.count(), std::vector<Eigen::Vector2f>(trajectory.begin(), trajectory.end())};
        auto response = sendRequest<driveunit_interface::CommandFollowTrajectory, driveunit_interface::CommandResponse>(command);
        return response.transform([](driveunit_interface::CommandResponse response) { return response.ack; });
    }

    /**
     * @brief Sends a rotation command.
     * @param degrees The rotation in degrees (CCW positive).
     * @return true if the command was acknowledged, false otherwise.
     */
    inline rd::expected<bool, std::error_code> sendRotateCommand(float degrees)
    {
        driveunit_interface::CommandRotate command{degrees};
        auto response = sendRequest<driveunit_interface::CommandRotate, driveunit_interface::CommandResponse>(command);
        return response.transform([](driveunit_interface::CommandResponse response) { return response.ack; });
    }
};

}