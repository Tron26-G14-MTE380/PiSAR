#include "pisar/driveunit/operating_mode.h"

#include "pisar/utilities/math.h"

namespace pisar::driveunit {

OperatingModeGoToTarget::OperatingModeGoToTarget(RobotFacility& facility, const Eigen::Vector2f target, const std::chrono::microseconds reference_time) :
    m_facility(facility),
    m_target_position(target),
    m_reference_time(std::chrono::microseconds(micros()) - reference_time),
    m_pid_rotation(kPidkpRotation, kPidkiRotation, kPidkdRotation, 0.0f, 1.0f),
    m_pid_travel(kPidkpTravel, kPidkiTravel, kPidkdTravel, 0.0f, 1.0f),
    m_last_update_time{0}
    {}

void OperatingModeGoToTarget::onEnterImpl()
{
    const auto ref_pose = m_facility.get().getKinematicTracker().poseAtNearest(m_reference_time);

    // const float pid_adj_scale = 1.0f / m_facility.get().getDriveController().getSpeedRange();
    // m_pid_rotation.setGains(kPidkpRotation * pid_adj_scale, kPidkiRotation * pid_adj_scale, kPidkdRotation * pid_adj_scale);
    // m_pid_travel.setGains(kPidkpTravel * pid_adj_scale, kPidkiTravel * pid_adj_scale, kPidkdTravel * pid_adj_scale);

    m_facility.get().getKinematicTracker().reset(false);
    if (ref_pose)
    {
        m_facility.get().getKinematicTracker().setPoseReference(ref_pose.value());
    }
}

[[nodiscard]] bool OperatingModeGoToTarget::updateImpl()
{
    auto current_pose = m_facility.get().getKinematicTracker().getPose();

    const auto current_position = current_pose.position;
    const auto current_heading = current_pose.orientation;

    const Eigen::Vector2f error_vec = m_target_position - current_position;
    const float distance_error = error_vec.norm();

    const float target_heading = -radToDeg(std::atan2(error_vec.x(), error_vec.y()));

    //PISAR_LOG_INFO("Heading: %f --> %f", current_heading, target_heading);
    float angle_error = target_heading - current_heading;

    // Normalize angle error to [-180, 180]
    while (angle_error > 180.0f) angle_error -= 360.0f;
    while (angle_error < -180.0f) angle_error += 360.0f;

    if (abs(angle_error) > kPastTargetAngleThreshold || distance_error < kOnTargetDistanceThresholds)
    {
        PISAR_LOG_INFO("Reached target");
        m_facility.get().getDriveController().hardStop();
        return true;
    }

    // PID update
    const auto delta_time = updateDeltaTime();
    float forward_output = m_pid_travel.update(std::abs(distance_error), delta_time);

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

void OperatingModeGoToTarget::onExitImpl() {}

}