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
    inline void onEnterImpl() { m_facility.get().driveStop(); }
    [[nodiscard]] inline bool updateImpl() { return false; }
    inline void onExitImpl() { m_facility.get().driveStop(); }
};


// Actual operating modes here

class OperatingModeFollowTrajectory : public OperatingMode<OperatingModeFollowTrajectory>
{
private:
    std::reference_wrapper<RobotFacility> m_facility;
    std::vector<Eigen::Vector2f> m_trajectory;
    std::chrono::duration<float> m_ref_time;

    std::chrono::milliseconds m_start_time;

public:
    OperatingModeFollowTrajectory(
        RobotFacility& facility,
        const std::span<const Eigen::Vector2f> trajectory,
        const std::chrono::duration<float> reference_time
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