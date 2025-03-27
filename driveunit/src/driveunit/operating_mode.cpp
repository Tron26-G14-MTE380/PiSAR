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

OperatingModeFollowTrajectory::OperatingModeFollowTrajectory(RobotFacility& facility, const std::span<const Eigen::Vector2f> trajectory, const std::chrono::microseconds reference_time) :
    m_facility(facility),
    m_trajectory(trajectory.begin(), trajectory.end()),
    m_reference_time(std::chrono::microseconds(micros()) - reference_time),
    m_pid_rotation(kPidkpRotation, kPidkiRotation, kPidkdRotation, 0.0f, 1.0f),
    m_pid_travel(kPidkpTravel, kPidkiTravel, kPidkdTravel, 0.0f, 1e6f),
    m_target_index(0),
    m_last_update_time{0}
    {}

void OperatingModeFollowTrajectory::onEnterImpl()
{
    //PISAR_LOG_INFO("Going to (%f, %f)", m_trajectory[0].x(), m_trajectory[0].y());
    const auto ref_pose = m_facility.get().getKinematicTracker().poseAtNearest(m_reference_time);

    const float pid_adj_scale = 1.0f / m_facility.get().getDriveController().getSpeedRange();
    m_pid_rotation.setGains(kPidkpRotation * pid_adj_scale, kPidkiRotation * pid_adj_scale, kPidkdRotation * pid_adj_scale);
    m_pid_travel.setGains(kPidkpTravel * pid_adj_scale, kPidkiTravel * pid_adj_scale, kPidkdTravel * pid_adj_scale);

    // Set target
    m_target_index = m_trajectory.size()-1;

    // ELEPHANT do a lot of bs with ash about distance travelled after image vs actual travel attempt
    m_facility.get().getKinematicTracker().reset(false);
    if (ref_pose)
    {
        m_facility.get().getKinematicTracker().setPoseReference(ref_pose.value());
    }
}

static float angleFromTrajectory(const Eigen::Vector2f& trajectory_vector, const Eigen::Vector2f target_vector)
{
    const float target_vector_norm = target_vector.norm();
    const float trajectory_vector_norm = trajectory_vector.norm();

    const bool target_at_ref = target_vector_norm < 1e-6;
    const bool trajectory_empty = trajectory_vector_norm < 1e-6;

    if (trajectory_empty)
    {
        PISAR_LOG_ERROR("Trajectory vector is 0, wtf");
        return 0;
    }

    // Robot is on trajectory start
    if (target_at_ref)
    {
        return 0;
    }

    return toDegrees(std::acos(target_vector.dot(trajectory_vector)/(target_vector_norm * trajectory_vector_norm)));
}

[[nodiscard]] bool OperatingModeFollowTrajectory::updateImpl()
{
    // if (m_trajectory.size() == 1)
    // {
    //     PISAR_LOG_INFO("Trajectory size is 1, use OperatingModeGoToTarget, stopping");
    //     m_facility.get().getDriveController().hardStop();
    //     return true;
    // }

    auto current_pose = m_facility.get().getKinematicTracker().getPose();

    const auto current_position = current_pose.position;
    const auto current_heading = current_pose.orientation;

    //PISAR_LOG_INFO("Current position: (%f, %f)", current_position.x(), current_position.y());

    // // Check if we passed the current target iteratively
    // while (true)
    // {
    //     Eigen::Vector2f trajectory_vector;
    //     if (m_target_index == m_trajectory.size() - 1)
    //     {
    //         trajectory_vector = m_trajectory[m_target_index] - m_trajectory[m_target_index-1];
    //     }
    //     else
    //     {
    //         trajectory_vector = m_trajectory[m_target_index+1] - m_trajectory[m_target_index];
    //     }

    //     const auto delta_pos = current_position - m_trajectory[m_target_index];

    //     // PISAR_LOG_INFO("Trajectory vector: (%f, %f)", trajectory_vector.x(), trajectory_vector.y());
    //     // PISAR_LOG_INFO("Delta pos: (%f, %f)", delta_pos.x(), delta_pos.y());

    //     const float angle_from_trajectory = angleFromTrajectory(trajectory_vector, delta_pos);

    //     //PISAR_LOG_INFO("angle_from_trajectory: %f", angle_from_trajectory);

    //     if (angle_from_trajectory < kDotProductTolerance)
    //     {
    //         if(m_target_index == m_trajectory.size() - 1)
    //         {
    //             PISAR_LOG_INFO("Reached end of trajectory");
    //             m_facility.get().getDriveController().hardStop();
    //             return true;
    //         }

    //         PISAR_LOG_INFO("Passed target %d", m_target_index);
    //         m_target_index ++;
    //     }
    //     else
    //     {
    //         break;
    //     }
    // }

    const auto target_position = getCurrentTarget();

    const Eigen::Vector2f error_vec = target_position - current_position;
    const float distance_error = error_vec.norm();

    const float target_heading = -toDegrees(std::atan2(error_vec.x(), error_vec.y()));

    //PISAR_LOG_INFO("Heading: %f --> %f", current_heading, target_heading);
    float angle_error = target_heading - current_heading;

    // Normalize angle error to [-180, 180]
    while (angle_error > 180.0f) angle_error -= 360.0f;
    while (angle_error < -180.0f) angle_error += 360.0f;

    // Test Data Updating ELEPHANT
    // PISAR_LOG_INFO("Trajectory Point: %f, %f", target_position.x(), target_position.y());

    // PID update
    const auto delta_time = updateDeltaTime();
    float forward_output = 1.0f; //std::clamp(distance_error * 0.5f, 0.1f, 1.0f); // creep at low distance

    const float turn_ratio = m_pid_rotation.update(std::abs(angle_error), delta_time);

    // Find the max speed, scale both down to keep ratio
    // Clamp again after scaling
    float left_speed = 0.0f, right_speed = 0.0f;
    if (angle_error > 0.0f) {
        // Turn left = slow down left wheel
        left_speed = forward_output * (1.0f - turn_ratio);
        right_speed = forward_output;
    } else {
        // Turn right = slow down right wheel
        left_speed = forward_output;
        right_speed = forward_output * (1.0f - turn_ratio);
    }
    PISAR_LOG_INFO("(%f, %f) -> (%f, %f)", forward_output, turn_ratio, left_speed, right_speed);

    left_speed = std::clamp(left_speed, 0.0f, 1.0f);
    right_speed = std::clamp(right_speed, 0.0f, 1.0f);

    // DRIVEEEEE!!! :D
    m_facility.get().getDriveController().tankDrive(left_speed, right_speed);

    return false;
}

void OperatingModeFollowTrajectory::onExitImpl()
{
}

///////////////////////////////////////////// OperatingModeGoToTarget ////////////////////////////////////////////

OperatingModeGoToTarget::OperatingModeGoToTarget(RobotFacility& facility, const Eigen::Vector2f target) :
    m_facility(facility),
    m_target(target),
    m_pid_rotation(kPidkpRotation, kPidkiRotation, kPidkdRotation, -1.0f, 1.0f),
    m_pid_travel(kPidkpTravel, kPidkiTravel, kPidkdTravel, 0.0f, 1.0f),
{
    const float pid_adj_scale = 1.0f / m_facility.get().getDriveController().getSpeedRange();
    m_pid_travel.setGains(kPidkpTravel * pid_adj_scale, kPidkiTravel * pid_adj_scale, kPidkdTravel * pid_adj_scale);

    // ELEPHANT do a lot of bs with ash about distance travelled after image vs actual travel attempt
    m_facility.get().getKinematicTracker().reset(false);

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

////////////////////////////////////////////////// OperatingModeRotate /////////////////////////////////////////////////

OperatingModeRotate::OperatingModeRotate(RobotFacility& facility, const float rotation_deg) :
    m_facility(facility),
    m_rotation_deg(rotation_deg),
    m_pid(kPidkp, kPidki, kPidkd, -1.0f, 1.0f),
    m_on_target_timestamp(std::nullopt)
    {}

void OperatingModeRotate::onEnterImpl()
{
    m_on_target_timestamp = std::nullopt;
    m_facility.get().getKinematicTracker().reset();

    const float pid_adj_scale = 1.0f / m_facility.get().getDriveController().getSpeedRange();
    m_pid.setGains(kPidkp * pid_adj_scale, kPidki * pid_adj_scale, kPidkd * pid_adj_scale);

    // ELEPHANT do a lot of bs with ash about distance travelled after image vs actual travel attempt
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

void OperatingModeRotate::onExitImpl()
{
    m_facility.get().getDriveController().stop();
}

} // namespace pisar::driveunit
