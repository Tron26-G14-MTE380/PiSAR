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

    inline std::optional<driveunit_interface::Response> handleMessage(
        const driveunit_interface::Request& request)
    {
        return std::visit(
            Overloaded{
                [this](const driveunit_interface::CommandRequest& command) -> driveunit_interface::Response
                {
                    return std::visit(
                        Overloaded{
                            [this](const driveunit_interface::CommandIdle& command) -> driveunit_interface::Response
                            {
                                PISAR_LOG_DEBUG("Got command 'Idle'");
                                m_operating_mode_manager.template switchMode(OperatingModeIdle(m_robot_facility));
                                return driveunit_interface::DefaultResponse {.ack = true};
                            },

                            [this](const driveunit_interface::CommandFollowTrajectory& command) -> driveunit_interface::Response
                            {
                                PISAR_LOG_DEBUG("Got command 'Follow Trajectory'");
                                m_operating_mode_manager.template switchMode(OperatingModeFollowTrajectory(
                                   m_robot_facility, std::span(command.trajectory), command.reference_time
                                ));
                                return driveunit_interface::DefaultResponse {.ack = true};
                            },

                            [this](const driveunit_interface::CommandRotate& command) -> driveunit_interface::Response
                            {
                                PISAR_LOG_DEBUG("Got command 'Rotate'");
                                m_operating_mode_manager.template switchMode(OperatingModeRotate(m_robot_facility, command.rotation_deg));
                                return driveunit_interface::DefaultResponse {.ack = true};
                            },
                        }, command
                    );
                },
                [this](const driveunit_interface::HeartbeatRequest& command) -> driveunit_interface::Response
                {
                    PISAR_LOG_DEBUG("Got Request 'Heartbeat'");
                    return driveunit_interface::HeartbeatResponse {
                        static_cast<decltype(driveunit_interface::HeartbeatResponse::time_alive)>(millis())
                    };
                }
            }, request
        );
    }
};

} // namespace pisar::driveunit
