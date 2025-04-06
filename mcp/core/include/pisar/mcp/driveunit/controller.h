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
     * @param retries Number of times to retry on failure.
     * @return `rd::expected<TResponse, std::error_code>` Response or error.
     */
    template<typename TRequest, typename TResponse>
    inline rd::expected<TResponse, std::error_code> sendRequest(const TRequest& request, unsigned int retries = 0)
    {
        return m_transport.get().template sendRequest<TRequest, TResponse>(request, retries);
    }

    /**
     * @brief Sends a command to the driveunit and waits for a response.
     * @param request The command request to send.
     * @param retries Number of times to retry on failure.
     * @return `rd::expected<driveunit_interface::Response, std::error_code>` Response or error.
     */
    [[nodiscard]] inline rd::expected<driveunit_interface::Response, std::error_code>
    sendRequest(const driveunit_interface::Request& request, unsigned int retries = 0)
    {
        return m_transport.get().sendRequest(request, retries);
    }

    /**
     * @brief Sends a heartbeat request.
     * @param retries Number of times to retry on failure.
     * @return The heartbeat response if successful, otherwise std::nullopt.
     */
    inline rd::expected<driveunit_interface::HeartbeatResponse, std::error_code> sendHeartbeatRequest(const unsigned int retries = 0)
    {
        return sendRequest<driveunit_interface::HeartbeatRequest, driveunit_interface::HeartbeatResponse>(driveunit_interface::HeartbeatRequest{}, retries);
    }

    /**
     * @brief Sends a command status request. Returns whether the driveunit is still busy with last command.
     * @param retries Number of times to retry on failure.
     * @return Whether the driveunit is busy with a command if successful, otherwise error code.
     */
    inline rd::expected<bool, std::error_code> sendCommandStatusRequest(const unsigned int retries = 0)
    {
        auto response = sendRequest<driveunit_interface::CommandStatusRequest, driveunit_interface::CommandStatusResponse>(driveunit_interface::CommandStatusRequest{}, retries);
        return response.transform([](driveunit_interface::CommandStatusResponse response) { return response.busy; });
    }

    /**
     * @brief Sends a command to idle.
     * @param retries Number of times to retry on failure.
     * @return true if the command was acknowledged, false otherwise.
     */
    inline rd::expected<bool, std::error_code> sendIdleCommand(const unsigned int retries = 0)
    {
        auto response = sendRequest<driveunit_interface::CommandIdle, driveunit_interface::CommandResponse>(driveunit_interface::CommandIdle{}, retries);
        return response.transform([](driveunit_interface::CommandResponse response) { return response.ack; });
    }

    /**
     * @brief Sends a command to hard stop.
     * @param retries Number of times to retry on failure.
     * @return true if the command was acknowledged, false otherwise.
     */
    inline rd::expected<bool, std::error_code> sendHardStopCommand(const unsigned int retries = 0)
    {
        auto response = sendRequest<driveunit_interface::CommandHardStop, driveunit_interface::CommandResponse>(driveunit_interface::CommandHardStop{}, retries);
        return response.transform([](driveunit_interface::CommandResponse response) { return response.ack; });
    }

    /**
     * @brief Sends a trajectory to the driveunit.
     * @param reference_time The time reference for trajectory execution.
     * @param trajectory The list of waypoints (Eigen::Vector2f).
     * @param retries Number of times to retry on failure.
     * @return true if the command was acknowledged, false otherwise.
     */
    inline rd::expected<bool, std::error_code> sendTrajectoryCommand(
        driveunit_interface::CommandFollowTrajectory::ReferenceTimeT reference_time,
        const std::span<const Eigen::Vector2f>& trajectory,
        const unsigned int retries = 0
    )
    {
        driveunit_interface::CommandFollowTrajectory command{reference_time.count(), std::vector<Eigen::Vector2f>(trajectory.begin(), trajectory.end())};
        auto response = sendRequest<driveunit_interface::CommandFollowTrajectory, driveunit_interface::CommandResponse>(command, retries);
        return response.transform([](driveunit_interface::CommandResponse response) { return response.ack; });
    }

    /**
     * @brief Tell the driveunit to travel to target position.
     *
     * @param reference_time The reference time when position data was taken.
     * @param target_position The target position.
     * @param retries Number of times to retry on failure.
     * @return true if the command was acknowledged, false otherwise.
     */
    inline rd::expected<bool, std::error_code> sendGoToTargetCommand(
        driveunit_interface::CommandGoToTarget::ReferenceTimeT reference_time,
        const Eigen::Vector2f& target_position,
        const unsigned int retries = 0
    )
    {
        driveunit_interface::CommandGoToTarget command{reference_time.count(), target_position};
        auto response = sendRequest<driveunit_interface::CommandGoToTarget, driveunit_interface::CommandResponse>(command, retries);
        return response.transform([](driveunit_interface::CommandResponse response) { return response.ack; });
    }

    /**
     * @brief Sends a rotation command.
     * @param degrees The rotation in degrees (CCW positive).
     * @param retries Number of times to retry on failure.
     * @return true if the command was acknowledged, false otherwise.
     */
    inline rd::expected<bool, std::error_code> sendRotateCommand(float degrees, const unsigned int retries = 0)
    {
        driveunit_interface::CommandRotate command{degrees};
        auto response = sendRequest<driveunit_interface::CommandRotate, driveunit_interface::CommandResponse>(command, retries);
        return response.transform([](driveunit_interface::CommandResponse response) { return response.ack; });
    }

    /**
     * @brief Sends a gripper command.
     * @param open True to open the gripper, false to close it.
     * @param retries Number of times to retry on failure.
     * @return true if the command was acknowledged, false otherwise.
     */
    inline rd::expected<bool, std::error_code> sendGripperCommand(bool open, const unsigned int retries = 0)
    {
        driveunit_interface::CommandSetGripper command{open};
        auto response = sendRequest<driveunit_interface::CommandSetGripper, driveunit_interface::CommandResponse>(command, retries);
        return response.transform([](driveunit_interface::CommandResponse response) { return response.ack; });
    }
};

}