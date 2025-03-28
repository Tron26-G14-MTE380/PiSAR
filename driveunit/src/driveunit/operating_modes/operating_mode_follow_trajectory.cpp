#include "pisar/driveunit/operating_mode.h"

#include "pisar/utilities/math.h"

namespace pisar::driveunit {

OperatingModeFollowTrajectory::OperatingModeFollowTrajectory(RobotFacility& facility, const std::span<const Eigen::Vector2f> trajectory, const std::chrono::microseconds reference_time) :
    m_facility(facility),
    m_trajectory(trajectory.begin(), trajectory.end()),
    m_reference_time(std::chrono::microseconds(micros()) - reference_time),
    m_pid_rotation(kPidkpRotation, kPidkiRotation, kPidkdRotation, 0.0f, 1.0f),
    m_pid_travel(kPidkpTravel, kPidkiTravel, kPidkdTravel, 0.0f, 1.0f),
    m_target_index(0),
    m_last_update_time{0}
    {}

void OperatingModeFollowTrajectory::onEnterImpl()
{
    const auto ref_pose = m_facility.get().getKinematicTracker().poseAtNearest(m_reference_time);

    const float pid_adj_scale = 1.0f / m_facility.get().getDriveController().getSpeedRange();
    m_pid_rotation.setGains(kPidkpRotation * pid_adj_scale, kPidkiRotation * pid_adj_scale, kPidkdRotation * pid_adj_scale);
    m_pid_travel.setGains(kPidkpTravel * pid_adj_scale, kPidkiTravel * pid_adj_scale, kPidkdTravel * pid_adj_scale);

    // Set target
    m_target_index = m_trajectory.size()-1;

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

    return radToDeg(std::acos(target_vector.dot(trajectory_vector)/(target_vector_norm * trajectory_vector_norm)));
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

    const float target_heading = -radToDeg(std::atan2(error_vec.x(), error_vec.y()));

    //PISAR_LOG_INFO("Heading: %f --> %f", current_heading, target_heading);
    float angle_error = target_heading - current_heading;

    // Normalize angle error to [-180, 180]
    while (angle_error > 180.0f) angle_error -= 360.0f;
    while (angle_error < -180.0f) angle_error += 360.0f;


    if (abs(angle_error) > kPastTargetAngleThreshold || distance_error < kOnTargetDistanceThresholds)
    {
        PISAR_LOG_INFO("Reached end of trajectory");
        m_facility.get().getDriveController().hardStop();
        return true;
    }

    // PID update
    const auto delta_time = updateDeltaTime();
    float forward_output = 1.0f; // m_pid_travel.update(std::abs(distance_error), delta_time)'

    const float turn_ratio = m_pid_rotation.update(std::abs(angle_error), delta_time);

    // Use turn ratio to offset forward output on one wheel.
    float left_speed = 0.0f, right_speed = 0.0f;
    if (angle_error > 0.0f)
    {
        // Turn left = slow down left wheel
        left_speed = forward_output * (1.0f - turn_ratio);
        right_speed = forward_output;
    }
    else
    {
        // Turn right = slow down right wheel
        left_speed = forward_output;
        right_speed = forward_output * (1.0f - turn_ratio);
    }

    //PISAR_LOG_INFO("(%f, %f) -> (%f, %f)", forward_output, turn_ratio, left_speed, right_speed);

    left_speed = std::clamp(left_speed, 0.0f, 1.0f);
    right_speed = std::clamp(right_speed, 0.0f, 1.0f);

    // DRIVEEEEE!!! :D
    m_facility.get().getDriveController().tankDrive(left_speed, right_speed);

    return false;
}

void OperatingModeFollowTrajectory::onExitImpl() {}

}