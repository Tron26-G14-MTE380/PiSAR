#pragma once

#include "pisar/driveunit/imu.h"
#include "pisar/driveunit/pose_history.h"
#include "pisar/low_pass_filter.h"

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
public:
    using TimestampRepT = std::chrono::microseconds;
    using PoseHistoryT = PoseHistory<TimestampRepT, tkPoseHistorySize>;

private:
    std::chrono::duration<float> m_sample_time;                     ///< IMU sampling time.
    TimestampRepT m_last_update_time;                               ///< Timestamp of last update.
    KinematicState m_current_state;                                 ///< The current kinematic state of the robot.
    PoseHistoryT m_pose_history;                                    ///< Tracks the pose history.

    LowPassFilter<Eigen::Vector3f> m_accel_filter;                  ///< Accelerometer data digital low pass filter.
    LowPassFilter<Eigen::Vector3f> m_gyro_filter;                   ///< Gyroscope data digital low pass filter.

public:
    /**
     * @brief Constructs a ImuPlanarKinematicTracker.
     */
    constexpr ImuPlanarKinematicTracker(TimestampRepT sample_time) :
        m_sample_time(sample_time),
        m_last_update_time(0),
        m_current_state {
            .pose = KinematicPose{Eigen::Vector2f::Zero(), 0.0f},
            .velocity = Eigen::Vector2f::Zero(),
            .angular_velocity = 0.0f,
            .acceleration = Eigen::Vector2f::Zero()
        },
        m_pose_history(),
        m_accel_filter(15, sample_time),
        m_gyro_filter(15, sample_time)
        {}

    /// @brief Destructor
    ~ImuPlanarKinematicTracker();

    /**
     * @brief Updates the internal kinematic state estimates using new IMU data.
     *
     * @param data The new accelerometer and gyroscope data reading.
     * @param timestamp The imu data timestamp.
     */
    void onImuReading(const Imu::AccelData& accel_data, const Imu::GyroData& gyro_data, TimestampRepT timestamp)
    {
        if (m_last_update_time.count() == 0)
        {
            m_last_update_time = timestamp;
            return; // Skip first update
        }

        // --- APPLY LOW-PASS FILTERS TO ACCEL & GYRO ---
        const auto filtered_accel_data = m_accel_filter.update(accel_data.values);
        const auto filtered_gyro_data = m_gyro_filter.update(gyro_data.values);

        m_current_state.angular_velocity = filtered_gyro_data.z();
        m_current_state.acceleration = filtered_accel_data.head<2>(); // Take X and Y

        // --- INTEGRATE VELOCITY ---
        m_current_state.velocity += m_current_state.acceleration * m_sample_time.count();

        // --- INTEGRATE POSITION ---
        m_current_state.pose.position += m_current_state.velocity * m_sample_time.count();

        // -- INTEGRATE ANGULAR VELOCITY ---
        m_current_state.pose.orientation += m_current_state.angular_velocity * m_sample_time.count();

        // Store updated state
        m_pose_history.addRecord(timestamp, m_current_state.pose);
    }

    /**
     * @brief Sets the position and orientation estimate reference point to provided pose @p ref.
     *
     * @param ref The reference pose to set.
     */
    constexpr inline void setPoseReference(const KinematicPose& ref)
    {
        m_current_state.pose.position -= ref.position;
        m_current_state.pose.orientation -= ref.orientation;
        m_pose_history.setReference(ref);
    }

    /// @brief Sets the position and orientation estimate reference point to current pose. Same effect as resetting them.
    constexpr inline void setPoseReference()
    {
        m_pose_history.setReference(m_current_state.pose);
        m_current_state.pose.position = Eigen::Vector2f::Zero();
        m_current_state.pose.orientation = 0.0f;
    }

    /// @brief Gets the current kinematic state (all variables).
    [[nodiscard]] inline constexpr const KinematicState& getState() const noexcept
    {
        return m_current_state;
    }

    /// @brief Gets the current pose (position and orientation) with respect to the reference.
    [[nodiscard]] inline constexpr const KinematicPose& getPose() const noexcept
    {
        return m_current_state.pose;
    }

    /// @brief Gets the pose (position and orientation) history with respect to the reference.
    [[nodiscard]] inline constexpr const PoseHistoryT& getPoseHistory() const noexcept
    {
        return m_pose_history;
    }

    /// @brief Gets the current position estimate with respect to the reference.
    [[nodiscard]] inline constexpr Eigen::Vector2f getPosition() const noexcept
    {
        return m_current_state.pose.position;
    }

    /// @brief Gets the current orientation estimate (heading) in radians with respect to the reference.
    [[nodiscard]] inline constexpr float getOrientation() const noexcept
    {
        return m_current_state.pose.orientation;
    }

    /// @brief Gets the current velocity estimate.
    [[nodiscard]] inline constexpr Eigen::Vector2f getVelocity() const noexcept
    {
        return m_current_state.velocity;
    }

    /// @brief Gets the current angular velocity estimate in radians/sec.
    [[nodiscard]] inline constexpr float getAngularVelocity() const noexcept
    {
        return m_current_state.angular_velocity;
    }

    /// @brief Gets the current acceleration estimate.
    [[nodiscard]] inline constexpr Eigen::Vector2f getAcceleration() const noexcept
    {
        return m_current_state.acceleration;
    }

    /// @brief The timestamp of the last update.
    [[nodiscard]] inline constexpr TimestampRepT getLastUpdateTime() const noexcept
    {
        return m_last_update_time;
    }
};

}