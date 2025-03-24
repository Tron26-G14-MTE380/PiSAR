#include "pisar/driveunit/operating_mode.h"
#include "pisar/driveunit/logging.h"

namespace pisar::driveunit {

///////////////////////////////////////////// OperatingModeFollowTrajectory ////////////////////////////////////////////

OperatingModeFollowTrajectory::OperatingModeFollowTrajectory(RobotFacility& facility, const std::span<const Eigen::Vector2f> trajectory) :
    m_facility(facility),
    m_trajectory(trajectory.begin(), trajectory.end()),
    m_adj_kp(0.0f),
    m_adj_ki(0.0f),
    m_adj_kd(0.0f),
    m_target_index(0),
    m_distance_to_target(0.0f),
    m_target_heading(0.0f),
    m_integral_angle(0.0f),
    m_last_angle_error(0.0f),
    m_last_update_time{0},
    m_on_target_timestamp(std::nullopt)
    {}

void OperatingModeFollowTrajectory::onEnterImpl()
{
    m_facility.get().getDriveController().stop();

    const float pid_adj_scale = 1.0f / m_facility.get().getDriveController().getSpeedRange();
    m_adj_kp = kPidkp * pid_adj_scale;
    m_adj_ki = kPidki * pid_adj_scale;
    m_adj_kd = kPidkd * pid_adj_scale;

    // Set target
    m_target_index = 0;
    m_distance_to_target = getCurrentTarget().norm();
    m_target_heading = std::atan2(getCurrentTarget().x(), getCurrentTarget().y()) * 180.0f / M_PI; // might have to adjust due to imu axis directions

    m_integral_angle = 0.0f;
    m_last_angle_error = m_target_heading;
    m_on_target_timestamp = std::nullopt;
    // ELEPHANT do a lot of bs with ash about distance travelled after image vs actual travel attempt
    m_facility.get().getKinematicTracker().reset(false);

}

[[nodiscard]] bool OperatingModeFollowTrajectory::updateImpl()
{
    auto current_pose = m_facility.get().getKinematicTracker().getPose();
    auto current_position = current_pose.position;
    auto current_heading = current_pose.orientation;

    Eigen::Vector2f error_vec = getCurrentTarget() - current_position;
    float distance_error = error_vec.norm();
    float angle_error = m_target_heading - current_heading;

    // Normalize angle error to [-180, 180]
    while (angle_error > 180.0f) angle_error -= 360.0f;
    while (angle_error < -180.0f) angle_error += 360.0f;

    auto current_time = std::chrono::milliseconds(millis());
    std::chrono::duration<float> elapsed = current_time - m_last_update_time;
    float dt = elapsed.count();

    if (std::abs(angle_error) > thetaTolerance || std::abs(distance_error) > distTolerance)
    {
        m_on_target_timestamp = std::nullopt;

        // Compute PID terms
        m_integral_angle += angle_error * dt;
        float derivative_angle = (angle_error - m_last_angle_error) / dt;

        // Sharing Kp for now, this could change to be individual and most likely will
        // PID outputs
        float angular_output = (m_adj_kp * angle_error) + (m_adj_kd * derivative_angle) + (m_adj_ki * m_integral_angle);

        // Forward speed (only proportional for now)
        float forward_output = m_adj_kp * distance_error;

        // Clamp speeds
        angular_output = std::clamp(angular_output, -1.0f, 1.0f);
        forward_output = std::clamp(forward_output, 0.0f, 1.0f); // no reversing

        // Blended drive — arc forward
        float left_speed = forward_output - angular_output;
        float right_speed = forward_output + angular_output;

        left_speed = std::clamp(left_speed, -1.0f, 1.0f);
        right_speed = std::clamp(right_speed, -1.0f, 1.0f);

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

    m_last_angle_error = angle_error;
    m_last_update_time = current_time;
    return false;
}

void OperatingModeFollowTrajectory::onExitImpl()
{
    m_facility.get().getDriveController().stop();
}

////////////////////////////////////////////////// OperatingModeRotate /////////////////////////////////////////////////

OperatingModeRotate::OperatingModeRotate(RobotFacility& facility, const float rotation_deg) :
    m_facility(facility),
    m_rotation_deg(rotation_deg),
    m_adj_kp(0.0f),
    m_adj_ki(0.0f),
    m_adj_kd(0.0f),
    m_integral(0.0f),
    m_last_error(0.0f),
    m_last_update_time{0},
    m_on_target_timestamp(std::nullopt)
    {}

void OperatingModeRotate::onEnterImpl()
{
    m_facility.get().getDriveController().stop();

    const float pid_adj_scale = 1.0f / m_facility.get().getDriveController().getSpeedRange();
    m_adj_kp = kPidkp * pid_adj_scale;
    m_adj_ki = kPidki * pid_adj_scale;
    m_adj_kd = kPidkd * pid_adj_scale;

    m_integral = 0.0f;
    m_last_error = m_rotation_deg;
    m_on_target_timestamp = std::nullopt;
    m_facility.get().getKinematicTracker().reset();
}

[[nodiscard]] bool OperatingModeRotate::updateImpl()
{
    float current_angle = m_facility.get().getKinematicTracker().getOrientation();

    float error = m_rotation_deg - current_angle;

    // Normalize error to [-180, 180] for shortest rotation path
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    auto current_time = std::chrono::milliseconds(millis());
    std::chrono::duration<float> elapsed = current_time - m_last_update_time;
    float dt = elapsed.count();


    if (std::abs(error) > kTolerance)
    {
        m_on_target_timestamp = std::nullopt;

        // Compute PID terms
        m_integral += error * dt;
        float derivative = (error - m_last_error) / dt;

        // Compute PID output
        float pid_output = (m_adj_kp * error) + (m_adj_ki * m_integral) + (m_adj_kd * derivative);

        // negative PID means left, positive means right
        float left_speed = -pid_output;
        float right_speed = pid_output;

        // Clamp speeds to [-1.0, 1.0]
        left_speed = std::clamp(left_speed, -1.0f, 1.0f);
        right_speed = std::clamp(right_speed, -1.0f, 1.0f);

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

    m_last_error = error;
    m_last_update_time = current_time;
    return false;
}

void OperatingModeRotate::onExitImpl()
{
    m_facility.get().getDriveController().stop();
}

} // namespace pisar::driveunit
