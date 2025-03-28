#include "pisar/driveunit/operating_mode.h"

namespace pisar::driveunit {

OperatingModeRotate::OperatingModeRotate(RobotFacility& facility, const float rotation_deg) :
    m_facility(facility),
    m_rotation_deg(rotation_deg),
    m_pid(kPidkp, kPidki, kPidkd, -1.0f, 1.0f),
    m_on_target_timestamp(std::nullopt)
    {}

void OperatingModeRotate::onEnterImpl()
{
    m_on_target_timestamp = std::nullopt;

    const float pid_adj_scale = 1.0f / m_facility.get().getDriveController().getSpeedRange();
    m_pid.setGains(kPidkp * pid_adj_scale, kPidki * pid_adj_scale, kPidkd * pid_adj_scale);

    m_facility.get().getKinematicTracker().reset(false);
}

[[nodiscard]] bool OperatingModeRotate::updateImpl()
{
    float current_angle = m_facility.get().getKinematicTracker().getOrientation();

    float error = m_rotation_deg - current_angle;

    // Normalize error to [-180, 180] for shortest rotation path
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    auto current_time = std::chrono::milliseconds(millis());
    const auto delta_time = updateDeltaTime();

    if (std::abs(error) > kTolerance)
    {
        m_on_target_timestamp = std::nullopt;

        const float pid_output = m_pid.update(error, delta_time);

        // negative PID means left, positive means right
        float left_speed = -pid_output;
        float right_speed = pid_output;

        // Clamp speeds to [-1.0, 1.0]
        left_speed = std::clamp(left_speed, -1.0f, 1.0f);
        right_speed = std::clamp(right_speed, -1.0f, 1.0f);

        // ROTATEEEEEE!!! :D
        m_facility.get().getDriveController().tankDrive(left_speed, right_speed);
    }
    else
    {
        if (!m_on_target_timestamp.has_value())
        {
            m_on_target_timestamp = current_time;
            m_facility.get().getDriveController().hardStop();
        }
        else if (current_time - m_on_target_timestamp.value() >= kOnTargetDurationTolerance)
        {
            m_facility.get().getDriveController().hardStop();
            return true;
        }
    }

    return false;
}

void OperatingModeRotate::onExitImpl() {}

}