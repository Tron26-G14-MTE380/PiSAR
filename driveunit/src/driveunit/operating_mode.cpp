#include "pisar/driveunit/operating_mode.h"
#include "pisar/driveunit/logging.h"

namespace pisar::driveunit {

template <typename T>
[[nodiscard]] constexpr T toDegrees(T radians)
{
    static_assert(std::is_arithmetic_v<T>, "toDegrees<T>: T must be an arithmetic type");
    return radians * static_cast<T>(180.0) / static_cast<T>(M_PI);
}

///////////////////////////////////////////// OperatingModeFollowTrajectory ////////////////////////////////////////////

OperatingModeFollowTrajectory::OperatingModeFollowTrajectory(RobotFacility& facility, const std::span<const Eigen::Vector2f> trajectory) :
    m_facility(facility),
    m_trajectory(trajectory.begin(), trajectory.end()),
    m_trajectry_points(trajectory.size()),
    m_adj_kp_rotation(0.0f),
    m_adj_ki_rotation(0.0f),
    m_adj_kd_rotation(0.0f),
    m_adj_kp_travel(0.0f),
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
    m_adj_kp_rotation = kPidkpRotation * pid_adj_scale;
    m_adj_ki_rotation = kPidkiRotation * pid_adj_scale;
    m_adj_kd_rotation = kPidkdRotation * pid_adj_scale;
    m_adj_kp_travel = kPidkpTravel * pid_adj_scale;

    // Set target
    m_target_index = 0;
    m_distance_to_target = getCurrentTarget().norm();
    m_target_heading = toDegrees(std::atan2(getCurrentTarget().y(), getCurrentTarget().x())); // might have to adjust due to imu axis directions

    m_integral_angle = 0.0f;
    m_last_angle_error = m_target_heading;
    m_on_target_timestamp = std::nullopt;
    // ELEPHANT do a lot of bs with ash about distance travelled after image vs actual travel attempt
    m_facility.get().getKinematicTracker().reset(false);

}

[[nodiscard]] bool OperatingModeFollowTrajectory::updateImpl()
{
    auto current_pose = m_facility.get().getKinematicTracker().getPose();
    auto target_position = getCurrentTarget();
    // conditional loop of passed point logic goes here
    auto current_position = current_pose.position;
    auto current_heading = current_pose.orientation;

    float angle = toDegrees(std::acos(current_position.dot(target_position)/(current_position.norm() * target_position.norm())));

    // Angle testing ELEPHANT
    // Note that this always evaluates to inf if the current position is 0,0
    // PISAR_LOG_INFO("Angle: %f, Current: %.20f, Target: %f", angle, current_position.norm(), target_position.norm());

    // Test Data Updating ELEPHANT
    // PISAR_LOG_INFO("First Trajectory Point: %f, %f", target_position.x(), target_position.y());

    if(angle > kDotProductTolerance)
    {
        Eigen::Vector2f error_vec = target_position - current_position;
        float distance_error = error_vec.norm();

        // PISAR_LOG_INFO("Distance Error: %f", distance_error);

        float angle_error = m_target_heading - current_heading;

        // Normalize angle error to [-180, 180]
        while (angle_error > 180.0f) angle_error -= 360.0f;
        while (angle_error < -180.0f) angle_error += 360.0f;

        auto current_time = std::chrono::milliseconds(millis());
        std::chrono::duration<float> elapsed = current_time - m_last_update_time;
        float dt = elapsed.count();

        // Compute PID terms
        m_integral_angle += angle_error * dt;
        float derivative_angle = (angle_error - m_last_angle_error) / dt;

        // Sharing Kp for now, this could change to be individual and most likely will
        // PID outputs
        float angular_output = (m_adj_kp_rotation * angle_error) + (m_adj_kd_rotation * derivative_angle) + (m_adj_ki_rotation * m_integral_angle);

        // Forward speed (only proportional for now)
        float forward_output = m_adj_kp_travel * distance_error;

        // Clamp speeds
        angular_output = std::clamp(angular_output, -1.0f, 1.0f);
        forward_output = std::clamp(forward_output, 0.0f, 1.0f); // no reversing, technically angle check solves this

        // Arc forward blend
        float left_speed_raw = abs(forward_output - angular_output);
        float right_speed_raw = abs(forward_output + angular_output);

        PISAR_LOG_INFO("Angle Error: %f", angle_error);
        // PISAR_LOG_INFO("Left Speed Original Raw: %f, Right Speed Original Raw: %f", left_speed_raw, right_speed_raw);

        // Find the max speed, scale both down to keep ratio
        float max_speed = std::max(left_speed_raw, right_speed_raw);
        left_speed_raw /= max_speed;
        right_speed_raw /= max_speed;

        PISAR_LOG_INFO("Left Speed Raw: %f, Right Speed Raw: %f", left_speed_raw, right_speed_raw);

        // Clamp again after scaling
        float left_speed = std::clamp(left_speed_raw, 0.0f, 1.0f);
        float right_speed = std::clamp(right_speed_raw, 0.0f, 1.0f);

        // PISAR_LOG_INFO("Left Speed: %f, Right Speed: %f", left_speed, right_speed);

        m_facility.get().getDriveController().tankDrive(left_speed, right_speed);

        m_last_angle_error = angle_error;
        m_last_update_time = current_time;
        return false;

    }
    else if(m_trajectry_points - 1 != m_target_index)
    {
        m_target_index += 1;
        return false;
    }
    else if(m_trajectry_points - 1 == m_target_index)
    {
        m_facility.get().getDriveController().hardStop();
        return true;
    }
}

void OperatingModeFollowTrajectory::onExitImpl()
{
    m_facility.get().getDriveController().hardStop();
}

///////////////////////////////////////////// OperatingModeGoToTarget ////////////////////////////////////////////

OperatingModeGoToTarget::OperatingModeGoToTarget(RobotFacility& facility, const Eigen::Vector2f target) :
    m_facility(facility),
    m_target(target),
    m_adj_kp_rotation(0.0f),
    m_adj_ki_rotation(0.0f),
    m_adj_kd_rotation(0.0f),
    m_adj_kp_travel(0.0f),
    m_distance_to_target(0.0f),
    m_target_heading(0.0f),
    m_integral_angle(0.0f),
    m_last_angle_error(0.0f),
    m_last_update_time{0},
    m_on_target_timestamp(std::nullopt)
    {}

void OperatingModeGoToTarget::onEnterImpl()
{
    m_facility.get().getDriveController().stop();

    const float pid_adj_scale = 1.0f / m_facility.get().getDriveController().getSpeedRange();
    m_adj_kp_rotation = kPidkpRotation * pid_adj_scale;
    m_adj_ki_rotation = kPidkiRotation * pid_adj_scale;
    m_adj_kd_rotation = kPidkdRotation * pid_adj_scale;
    m_adj_kp_travel = kPidkpTravel * pid_adj_scale;

    // Set target
    m_distance_to_target = m_target.norm();
    m_target_heading = toDegrees(std::atan2(m_target.y(), m_target.x())); // might have to adjust due to imu axis directions

    m_integral_angle = 0.0f;
    m_last_angle_error = m_target_heading;
    m_on_target_timestamp = std::nullopt;
    // ELEPHANT do a lot of bs with ash about distance travelled after image vs actual travel attempt
    m_facility.get().getKinematicTracker().reset(false);

}

[[nodiscard]] bool OperatingModeGoToTarget::updateImpl()
{
    auto current_pose = m_facility.get().getKinematicTracker().getPose();
    auto current_position = current_pose.position;
    auto current_heading = current_pose.orientation;

    // Note that this always evaluates to inf if the current position is 0,0
    float angle = toDegrees(std::acos(current_position.dot(m_target)/(current_position.norm() * m_target.norm())));

    // Angle testing ELEPHANT

    // PISAR_LOG_INFO("Angle: %f, Current: %.20f, Target: %f", angle, current_position.norm(), target_position.norm());

    // Test Data Updating ELEPHANT
    // PISAR_LOG_INFO("First Trajectory Point: %f, %f", target_position.x(), target_position.y());

    Eigen::Vector2f error_vec = m_target - current_position;
    float distance_error = error_vec.norm();

    // PISAR_LOG_INFO("Distance Error: %f", distance_error);

    float angle_error = m_target_heading - current_heading;

    // Normalize angle error to [-180, 180]
    while (angle_error > 180.0f) angle_error -= 360.0f;
    while (angle_error < -180.0f) angle_error += 360.0f;

    auto current_time = std::chrono::milliseconds(millis());
    std::chrono::duration<float> elapsed = current_time - m_last_update_time;
    float dt = elapsed.count();

    if (std::abs(angle_error) > kThetaTolerance || std::abs(distance_error) > kDistTolerance)
    {
        m_on_target_timestamp = std::nullopt;

        // Compute PID terms
        m_integral_angle += angle_error * dt;
        float derivative_angle = (angle_error - m_last_angle_error) / dt;

        // Sharing Kp for now, this could change to be individual and most likely will
        // PID outputs
        float angular_output = (m_adj_kp_rotation * angle_error) + (m_adj_kd_rotation * derivative_angle) + (m_adj_ki_rotation * m_integral_angle);

        // Forward speed (only proportional for now)
        float forward_output = m_adj_kp_travel * distance_error;

        // Clamp speeds
        angular_output = std::clamp(angular_output, -1.0f, 1.0f);
        forward_output = std::clamp(forward_output, 0.0f, 1.0f); // no reversing, technically angle check solves this

        // Arc forward blend
        float left_speed_raw = abs(forward_output - angular_output);
        float right_speed_raw = abs(forward_output + angular_output);

        // PISAR_LOG_INFO("Angle Error: %f", angle_error);
        // PISAR_LOG_INFO("Left Speed Original Raw: %f, Right Speed Original Raw: %f", left_speed_raw, right_speed_raw);

        // Find the max speed, scale both down to keep ratio
        float max_speed = std::max(left_speed_raw, right_speed_raw);
        left_speed_raw /= max_speed;
        right_speed_raw /= max_speed;

        //PISAR_LOG_INFO("Left Speed Raw: %f, Right Speed Raw: %f", left_speed_raw, right_speed_raw);

        // PISAR_LOG_INFO("Left Speed: %f, Right Speed: %f", left_speed, right_speed);

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

    m_last_angle_error = angle_error;
    m_last_update_time = current_time;
    return false;
}

void OperatingModeGoToTarget::onExitImpl()
{
    m_facility.get().getDriveController().hardStop();
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
