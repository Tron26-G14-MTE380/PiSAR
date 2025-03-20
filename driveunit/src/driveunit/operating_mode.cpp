#include "pisar/driveunit/operating_mode.h"
#include "pisar/driveunit/logging.h"

namespace pisar::driveunit {

///////////////////////////////////////////// OperatingModeFollowTrajectory ////////////////////////////////////////////

OperatingModeFollowTrajectory::OperatingModeFollowTrajectory(
    RobotFacility& facility,
    const std::span<const Eigen::Vector2f> trajectory,
    const std::chrono::duration<float> reference_time
) : m_facility(facility), m_trajectory(trajectory.begin(), trajectory.end()), m_ref_time(reference_time), m_start_time(0) {}

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

     m_start_time = std::chrono::milliseconds(millis());
}

[[nodiscard]] bool OperatingModeFollowTrajectory::updateImpl()
{
    if (millis() - m_start_time.count() > 500)
    {
        return true;
    }

    return false;
}

void OperatingModeFollowTrajectory::onExitImpl()
{
    PISAR_LOG_INFO("Exiting Follow Trajectory mode");
    m_facility.get().driveStop();
}

////////////////////////////////////////////////// OperatingModeRotate /////////////////////////////////////////////////

OperatingModeRotate::OperatingModeRotate(RobotFacility& facility, const float rotation_deg) :
    m_facility(facility), m_rotation_deg(rotation_deg), m_last_update_time{0} {}

void OperatingModeRotate::onEnterImpl() 
{ 
    m_integral = 0.0f;
    m_last_error = 0.0f;
}
[[nodiscard]] bool OperatingModeRotate::updateImpl() 
{ 

    float current_angle = m_facility.get().getOrientation();

    float error = m_rotation_deg - current_angle;

    // Normalize error to [-180, 180] for shortest rotation path
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    auto current_time = std::chrono::milliseconds(millis());
    std::chrono::duration<float> elapsed = current_time - m_last_update_time;
    float dt = elapsed.count();

    // Compute PID terms
    m_integral += error * dt;
    float derivative = (error - m_last_error) / dt;

    // Compute PID output
    float pid_output = (m_kp * error) + (m_ki * m_integral) + (m_kd * derivative);

    // negative PID means left, positive means right
    float left_speed = -pid_output;
    float right_speed = pid_output;

    // Clamp speeds to [-1.0, 1.0]
    left_speed = std::clamp(left_speed, -1.0f, 1.0f);
    right_speed = std::clamp(right_speed, -1.0f, 1.0f);

    m_facility.get().tankDrive(left_speed, right_speed);

    m_last_error = error;
    m_last_update_time = std::chrono::milliseconds(millis());

    if (std::abs(error) < 1.0f)
    {
        m_facility.get().driveStop();
        return true;
    }

    return false;
}
void OperatingModeRotate::onExitImpl() 
{   
    PISAR_LOG_INFO("Exiting Rotate mode");
    m_facility.get().driveStop(); 
}

} // namespace pisar::driveunit
