#pragma once

#include "pisar/driveunit/imu.h"
#include "pisar/driveunit/pose_history.h"
#include "pisar/low_pass_filter.h"

#include <MadgwickAHRS.h>

#include <cstdint>

namespace pisar::driveunit {

struct KinematicState
{
    KinematicPose pose;

    Eigen::Vector2f velocity;
    float angular_velocity;

    Eigen::Vector2f acceleration;

};

/**
 * @brief Provides position and orientation tracking using an Imu.
 */
template<std::size_t tkPoseHistorySize>
class ImuPlanarKinematicTracker
{
private:
    float m_sample_time_us;
    uint64_t m_last_update_time_us;                                 ///< Timestamp of last update.
    KinematicState m_current_state;
    PoseHistory<tkPoseHistorySize> m_pose_history;                  ///< Tracks the pose history.

    LowPassFilter<typename Imu::AccelData::values> m_accel_filter;  ///< Accelerometer data digital low pass filter.
    LowPassFilter<typename Imu::GyroData::values> m_gyro_filter;    ///< Gyroscope data digital low pass filter.

    Madgwick m_madgwick_filter;                                     ///< Madgwick filter for orientation estimation.

public:
    /**
     * @brief Constructs a PoseEstimator.
     */
    ImuPlanarKinematicTracker(float sample_time_us) :
        m_sample_time_us(sample_time_us),
        m_last_update_time_us(0),
        m_current_state{
            0, {
                KinematicPose{Eigen::Vector2f::Zero(), 0.0f},
                Eigen::Vector2f::Zero(), 0.0f,
                Eigen::Vector2f::Zero()
            }
        },
        m_pose_history(),
        m_accel_filter(15, sample_time_us),
        m_gyro_filter(15, sample_time_us),
        m_madgwick_filter()
        {}

    /// @brief Destructor
    ~ImuPlanarKinematicTracker();

    /**
     * @brief Updates the internal kinematic state estimates using new IMU data.
     *
     * @param data The new accelerometer and gyroscope data reading.
     * @param timestamp_us The imu data timestamp.
     */
    void onImuReading(const Imu::AccelData& accel_data, const Imu::GyroData& gyro_data, uint64_t timestamp_us)
    {
        if (m_last_update_time_us == 0)
        {
            m_last_update_time_us = timestamp_us;
            m_madgwick_filter.begin(1.0f / m_sample_time_us);
            return; // Skip first update
        }

        // Compute time delta (dt)
        const float dt = static_cast<float>(timestamp_us - m_last_update_time_us) * 1e-6f;
        m_last_update_time_us = timestamp_us;

        // --- APPLY LOW-PASS FILTERS TO ACCEL & GYRO ---
        filtered_accel_data = m_accel_filter.update(accel_data.values);
        filtered_gyro_data = m_gyro_filter.update(gyro_data.values);

        m_current_state.angular_velocity = filtered_gyro_data.z();
        m_current_state.acceleration = filtered_accel_data;

        // --- UPDATE ORIENTATION USING MADGWICK FILTER ---
        filter.updateIMU(
            filtered_gyro_data.x(), filtered_gyro_data.y(), filtered_gyro_data.z(),
            filtered_accel_data.x(), filtered_accel_data.y(), filtered_accel_data.z()
        );
        m_current_state.pose.orientation = filter.getYawRadians(); // Extract yaw from quaternion

        // --- INTEGRATE VELOCITY ---
        m_current_state.velocity += linear_accel * dt;

        // --- INTEGRATE POSITION ---
        m_current_state.pose.position += m_current_state.velocity * dt;

        // Store updated state
        m_pose_history.addRecord(timestamp_us, m_current_state);
    }

    /**
     * @brief Sets the position and orientation estimate reference point to provided pose @p ref.
     *
     * @param ref The reference pose to set.
     */
    void setPoseReference(const KinematicPose& ref)
    {
        m_current_state.pose.position -= ref.position;
        m_current_state.pose.orientation -= ref.orientation;
        m_pose_history.setReference(ref);
    }

    /// @brief Sets the position and orientation estimate reference point to current pose. Same effect as resetting them.
    void setPoseReference()
    {
        m_pose_history.setReference(m_current_state.pose);
        m_current_state.pose.position = Eigen::Vector2f::Zero();
        m_current_state.pose.orientation = 0.0f;
    }

    /// @brief Gets the current kinematic state (all variables).
    [[nodiscard]] constexpr const KinematicState& getState() const noexcept
    {
        return m_current_state;
    }

    /// @brief Gets the current pose (position and orientation) with respect to the reference.
    [[nodiscard]] constexpr const KinematicPose& getPose() const noexcept
    {
        return m_current_state.pose;
    }

    /// @brief Gets the pose (position and orientation) history with respect to the reference.
    [[nodiscard]] constexpr const PoseHistory<tkPoseHistorySize>& getPoseHistory() const noexcept
    {
        return m_pose_history;
    }

    /// @brief Gets the current position estimate with respect to the reference.
    [[nodiscard]] constexpr Eigen::Vector2f getPosition() const noexcept
    {
        return m_current_state.pose.position;
    }

    /// @brief Gets the current orientation estimate (heading) in radians with respect to the reference.
    [[nodiscard]] constexpr float getOrientation() const noexcept
    {
        return m_current_state.pose.orientation;
    }

    /// @brief Gets the current velocity estimate.
    [[nodiscard]] constexpr Eigen::Vector2f getVelocity() const noexcept
    {
        return m_current_state.velocity;
    }

    /// @brief Gets the current angular velocity estimate in radians/sec.
    [[nodiscard]] constexpr float getAngularVelocity() const noexcept
    {
        return m_current_state.angular_velocity;
    }

    /// @brief Gets the current acceleration estimate.
    [[nodiscard]] constexpr Eigen::Vector2f getAcceleration() const noexcept
    {
        return m_current_state.acceleration;
    }

    /// @brief The timestamp of the last update.
    [[nodiscard]] constexpr uint64_t getLastUpdateTime() const noexcept
    {
        return m_last_update_time;
    }
};

}