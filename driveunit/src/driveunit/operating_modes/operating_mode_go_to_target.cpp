#include "pisar/driveunit/operating_mode.h"

namespace pisar::driveunit {

OperatingModeGoToTarget::OperatingModeGoToTarget(RobotFacility& facility, const Eigen::Vector2f target) :
    m_facility(facility),
    m_target(target),
    m_pid_rotation(kPidkpRotation, kPidkiRotation, kPidkdRotation, -1.0f, 1.0f),
    m_pid_travel(kPidkpTravel, kPidkiTravel, kPidkdTravel, 0.0f, 1.0f)
{
    const float pid_adj_scale = 1.0f / m_facility.get().getDriveController().getSpeedRange();
    m_pid_travel.setGains(kPidkpTravel * pid_adj_scale, kPidkiTravel * pid_adj_scale, kPidkdTravel * pid_adj_scale);

    // ELEPHANT do a lot of bs with ash about distance travelled after image vs actual travel attempt
    m_facility.get().getKinematicTracker().reset(false);

}

void OperatingModeGoToTarget::onEnterImpl()
{
    m_facility.get().getDriveController().hardStop();
}

[[nodiscard]] bool OperatingModeGoToTarget::updateImpl()
{
    auto current_pose = m_facility.get().getKinematicTracker().getPose();
    auto current_position = current_pose.position;
    auto current_heading = current_pose.orientation;

    // Test Data Updating ELEPHANT
    // PISAR_LOG_INFO("First Trajectory Point: %f, %f", target_position.x(), target_position.y());

    Eigen::Vector2f error_vec = m_target - current_position;
    float distance_error = error_vec.norm();

    // PISAR_LOG_INFO("Distance Error: %f", distance_error);

    // ELEPHANT this is lowkey butchered with IMU coonfig rn
    float target_angle = toDegrees(std::atan2(error_vec.y(), error_vec.x()));
    float angle_error = target_angle - current_heading;

    // Angle testing ELEPHANT

    // PISAR_LOG_INFO("Angle: %f", angle_error);

    // Normalize angle error to [-180, 180]
    while (angle_error > 180.0f) angle_error -= 360.0f;
    while (angle_error < -180.0f) angle_error += 360.0f;

    auto current_time = std::chrono::milliseconds(millis());

    std::chrono::duration<float> elapsed = current_time - m_last_update_time;
    float dt = elapsed.count();

    // PISAR_LOG_INFO("Time Delta: %f", dt);

    if (std::abs(angle_error) > kThetaTolerance || std::abs(distance_error) > kDistTolerance)
    {
        m_on_target_timestamp = std::nullopt;

        // Compute PID terms
        const auto delta_time = updateDeltaTime();
        const float angular_output = m_pid_rotation.update(angle_error, delta_time);
        const float forward_output = m_pid_travel.update(distance_error, delta_time);

        // Arc forward blend
        const float left_speed_raw = abs(forward_output - angular_output);
        const float right_speed_raw = abs(forward_output + angular_output);

        const float max_speed = std::max(left_speed_raw, right_speed_raw);

        float left_speed = std::clamp(left_speed_raw / max_speed, 0.0f, 1.0f);
        float right_speed = std::clamp(right_speed_raw / max_speed, 0.0f, 1.0f);

        m_facility.get().getDriveController().tankDrive(left_speed_raw, right_speed_raw);
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

void OperatingModeGoToTarget::onExitImpl()
{
    m_facility.get().getDriveController().hardStop();
}

}