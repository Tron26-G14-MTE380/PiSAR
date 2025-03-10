#include "pisar/driveunit/operating_mode.h"
#include "pisar/driveunit/logging.h"

namespace pisar::driveunit {

///////////////////////////////////////////// OperatingModeFollowTrajectory ////////////////////////////////////////////

OperatingModeFollowTrajectory::OperatingModeFollowTrajectory(
    RobotFacility& facility,
    const std::span<const Eigen::Vector2f> trajectory,
    const std::chrono::duration<float> reference_time
) : m_facility(facility), m_trajectory(trajectory.begin(), trajectory.end()), m_ref_time(reference_time) {}

void OperatingModeFollowTrajectory::onEnterImpl()
{
    PISAR_LOG_INFO("Entering Follow Trajectory mode with %zu waypoints", m_trajectory.size());

    if (m_trajectory.empty())
    {
        PISAR_LOG_WARN("Trajectory is empty, stopping immediately.");
        m_facility.get().driveStop();
        return;
    }

     // Move towards the first waypoint
     Eigen::Vector2f target = m_trajectory.front();

     float forward_movement = target.x();  // Forward distance
     float sideways_movement = target.y(); // Sideways movement

     // Convert sideways movement into turning (assumption: positive = right, negative = left)
     float base_speed = 0.3f;
     float turn_speed = std::clamp(sideways_movement * 0.5f, -1.0f, 1.0f);

     float left_speed = base_speed - turn_speed;
     float right_speed = base_speed + turn_speed;

     // Clamp speeds
     left_speed = std::clamp(left_speed, -1.0f, 1.0f);
     right_speed = std::clamp(right_speed, -1.0f, 1.0f);

     // Set speed and move forward (very basic)
     m_facility.get().tankDrive(left_speed, right_speed);
}

[[nodiscard]] bool OperatingModeFollowTrajectory::updateImpl()
{
    return false;
}

void OperatingModeFollowTrajectory::onExitImpl()
{
    PISAR_LOG_INFO("Exiting Follow Trajectory mode");
    m_facility.get().driveStop();
}

////////////////////////////////////////////////// OperatingModeRotate /////////////////////////////////////////////////

OperatingModeRotate::OperatingModeRotate(RobotFacility& facility, float rotation_deg) :
    m_facility(facility), m_rotation_deg(rotation_deg) {}

void OperatingModeRotate::onEnterImpl() { m_facility.get().driveStop(); }
[[nodiscard]] bool OperatingModeRotate::updateImpl() { return true; }
void OperatingModeRotate::onExitImpl() { m_facility.get().driveStop(); }

} // namespace pisar::driveunit
