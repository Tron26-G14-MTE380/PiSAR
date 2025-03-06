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
    RobotFacility& m_facility;

public:
    OperatingModeIdle(RobotFacility& facility);
    inline void onEnterImpl() {}
    [[nodiscard]] inline bool updateImpl() { return false; }
    inline void onExitImpl() {}
};


// Actual operating modes here

class OperatingModeFollowTrajectory : public OperatingMode<OperatingModeFollowTrajectory>
{
private:
    RobotFacility& m_facility;
    std::vector<Eigen::Vector2f> m_trajectory;

public:
    OperatingModeFollowTrajectory(
        RobotFacility& facility, std::span<Eigen::Vector2f> trajectory, std::chrono::duration<float> reference_time);
    void onEnterImpl();
    [[nodiscard]] bool updateImpl();
    void onExitImpl();
};

class OperatingModeRotate: public OperatingMode<OperatingModeFollowTrajectory>
{
private:
    RobotFacility& m_facility;
    float m_rotation_deg;

public:
    OperatingModeRotate(RobotFacility& facility, float rotation_deg);
    void onEnterImpl();
    [[nodiscard]] bool updateImpl();
    void onExitImpl();
};

}