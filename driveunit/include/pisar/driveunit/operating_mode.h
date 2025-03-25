#pragma once

#include "pisar/driveunit/facility.h"

#include <variant>
#include <span>
#include <chrono>

namespace pisar::driveunit {

/**
 * @brief Base class for all operating modes using CRTP.
 */
template<typename TDerived>
class OperatingMode
{
private:
    bool m_finished = false;

public:
    OperatingMode() = default;

    /**
     * @brief Called when entering this mode.
     */
    void onEnter()
    {
        m_finished = false; // Reset completion state
        static_cast<TDerived*>(this)->onEnterImpl();
    }

    /**
     * @brief Runs the mode's update step.
     * @return False if the mode is still running, true if it is finished.
     */
    [[nodiscard]] bool update()
    {
        if (m_finished)
        {
            return false; // Mode is finished, skip execution
        }

        m_finished = static_cast<TDerived*>(this)->updateImpl();
        return m_finished;
    }

    /**
     * @brief Called when exiting this mode.
     */
    void onExit()
    {
        static_cast<TDerived*>(this)->onExitImpl();
    }

    /**
     * @brief Checks if the mode has finished execution.
     * @return True if finished, false otherwise.
     */
    [[nodiscard]] bool isFinished() const { return m_finished; }
};

class OperatingModeIdle : public OperatingMode<OperatingModeIdle>
{
private:
    std::reference_wrapper<RobotFacility> m_facility;

public:
    OperatingModeIdle(RobotFacility& facility) : m_facility(facility) {}
    inline void onEnterImpl() { m_facility.get().getDriveController().hardStop(); }
    [[nodiscard]] inline bool updateImpl() { return false; }
    inline void onExitImpl() { m_facility.get().getDriveController().hardStop(); }
};

// Actual operating modes here

class OperatingModeFollowTrajectory : public OperatingMode<OperatingModeFollowTrajectory>
{
private:
    std::reference_wrapper<RobotFacility> m_facility;
    std::vector<Eigen::Vector2f> m_trajectory;

    static constexpr float kThetaTolerance = 5.0f;
    static constexpr float kDistTolerance = 0.0000001f;
    static constexpr auto kOnTargetDurationTolerance = std::chrono::milliseconds(1);
    static constexpr float kDotProductTolerance = 120.0f;

    static constexpr float kPidkpRotation = 0.000005f;
    static constexpr float kPidkiRotation = 0.0f;
    static constexpr float kPidkdRotation = 0.0001f;

    static constexpr float kPidkpTravel = 0.01f;

    float m_adj_kp_rotation;
    float m_adj_ki_rotation;
    float m_adj_kd_rotation;

    float m_adj_kp_travel;

    int m_target_index;
    float m_distance_to_target;
    float m_target_heading;
    int m_trajectry_points;

    float m_integral_angle;
    float m_last_angle_error;
    std::chrono::milliseconds m_last_update_time;
    std::optional<std::chrono::milliseconds> m_on_target_timestamp;

    [[nodiscard]] inline const Eigen::Vector2f& getCurrentTarget() const
    {
        return m_trajectory[m_target_index];
    }

public:
    OperatingModeFollowTrajectory(
        RobotFacility& facility,
        const std::span<const Eigen::Vector2f> trajectory
    );
    void onEnterImpl();
    [[nodiscard]] bool updateImpl();
    void onExitImpl();
};

class OperatingModeGoToTarget : public OperatingMode<OperatingModeGoToTarget>
{
private:
    std::reference_wrapper<RobotFacility> m_facility;
    Eigen::Vector2f m_target;

    static constexpr float kThetaTolerance = 5.0f;
    static constexpr float kDistTolerance = 0.0000001f;
    static constexpr auto kOnTargetDurationTolerance = std::chrono::milliseconds(1);
    static constexpr float kDotProductTolerance = 120.0f;

    static constexpr float kPidkpRotation = 0.000005f;
    static constexpr float kPidkiRotation = 0.0f;
    static constexpr float kPidkdRotation = 0.0001f;

    static constexpr float kPidkpTravel = 0.01f;

    float m_adj_kp_rotation;
    float m_adj_ki_rotation;
    float m_adj_kd_rotation;

    float m_adj_kp_travel;

    float m_distance_to_target;
    float m_target_heading;

    float m_integral_angle;
    float m_last_angle_error;
    std::chrono::milliseconds m_last_update_time;
    std::optional<std::chrono::milliseconds> m_on_target_timestamp;

public:
    OperatingModeGoToTarget(
        RobotFacility& facility,
        const Eigen::Vector2f target
    );
    void onEnterImpl();
    [[nodiscard]] bool updateImpl();
    void onExitImpl();
};

class OperatingModeRotate: public OperatingMode<OperatingModeRotate>
{
private:
    std::reference_wrapper<RobotFacility> m_facility;
    float m_rotation_deg;

    static constexpr float kPidkp = 0.0007f;
    static constexpr float kPidki = 0.0f;
    static constexpr float kPidkd = 0.0001f;

    static constexpr float kTolerance = 3.0f;
    static constexpr auto kOnTargetDurationTolerance = std::chrono::milliseconds(500);

    float m_adj_kp;
    float m_adj_ki;
    float m_adj_kd;

    float m_integral;
    float m_last_error;
    std::chrono::milliseconds m_last_update_time;
    std::optional<std::chrono::milliseconds> m_on_target_timestamp;

public:
    OperatingModeRotate(RobotFacility& facility, float rotation_deg);
    void onEnterImpl();
    [[nodiscard]] bool updateImpl();
    void onExitImpl();
};

}