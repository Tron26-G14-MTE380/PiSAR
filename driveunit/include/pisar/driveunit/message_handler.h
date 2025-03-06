#pragma once

#include "pisar/driveunit/operating_mode_manager.h"
#include "pisar/driveunit/operating_mode.h"
#include "pisar/driveunit/facility.h"

#include "pisar/utilities/stdex.h"

#include <span>

namespace pisar::driveunit {

/**
 * @brief Handles SPI messages and executes robot commands.
 */
template<class TOperatingModeManager>
class MessageHandler {
private:
    RobotFacility& m_robot_facility;
    TOperatingModeManager& m_operating_mode_manager;

public:
    explicit MessageHandler(RobotFacility& robot, TOperatingModeManager& operating_mode_manager) :
        m_robot_facility(robot), m_operating_mode_manager(operating_mode_manager) {}

    inline size_t handleMessageImpl(
        const driveunit_interface::Request& request, const std::span<std::byte>& response_buffer)
    {
        return std::visit(
            utilities::Overloaded{
                [this, response_buffer](const driveunit_interface::CommandIdle& command) -> size_t
                {
                    m_operating_mode_manager.template switchMode<OperatingModeIdle>(m_robot_facility);
                    return encodeDefaultResponse(response_buffer);
                },

                [this, response_buffer](const driveunit_interface::CommandFollowTrajectory& command) -> size_t
                {
                    m_operating_mode_manager.template switchMode<OperatingModeFollowTrajectory>(
                        m_robot_facility, std::span(command.trajectory), command.reference_time
                    );
                    return encodeDefaultResponse(response_buffer);
                },

                [this, response_buffer](const driveunit_interface::CommandRotate& command) -> size_t
                {
                    m_operating_mode_manager.template switchMode<OperatingModeRotate>(m_robot_facility, command.rotation_deg);
                    return encodeDefaultResponse(response_buffer);
                },

                [this, response_buffer](const driveunit_interface::HeartbeatRequest& command) -> size_t
                {
                    return encodeResponse(driveunit_interface::HeartbeatResponse {
                            static_cast<decltype(driveunit_interface::HeartbeatResponse::time_alive)>(millis())
                        },
                        response_buffer
                    );
                }},

            request);
    }

private:
    inline size_t encodeDefaultResponse(const std::span<std::byte>& buffer)
    {
        driveunit_interface::DefaultResponse response{true};
        return encodeResponse(response, buffer);
    }

    template<typename T>
    inline size_t encodeResponse(const T& response, const std::span<std::byte>& buffer)
    {
        driveunit_interface::PacketEncoder<T> encoder;
        auto encoded = encoder.encode(response, buffer);
        return encoded ? encoded->size_bytes() : 0;
    }

    template<typename T>
    inline size_t encodeResponse(T&& response, const std::span<std::byte>& buffer)
    {
        driveunit_interface::PacketEncoder<T> encoder;
        auto encoded = encoder.encode(std::forward<T>(response), buffer);
        return encoded ? encoded->size_bytes() : 0;
    }
};

} // namespace pisar::driveunit
