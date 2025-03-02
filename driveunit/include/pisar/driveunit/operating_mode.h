#pragma once

#include "facility.h"

#include <variant>

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
public:
    inline void onEnterImpl() {}
    [[nodiscard]] inline bool updateImpl() { return false; }
    inline void onExitImpl() {}
};

/**
 * @brief Manages the active operating mode of the driveunit.
 */
template<typename... TOperatingModes>
class OperatingModeManager
{
private:
    std::variant<OperatingModeIdle, TOperatingModes...> m_current_mode;

public:
    OperatingModeManager() : m_current_mode(OperatingModeIdle{}) {}

    /**
     * @brief Switches to a new operating mode with custom parameters.
     * @tparam TMode The new operating mode type.
     * @tparam TArgs Types of arguments for mode construction.
     * @param args Arguments to construct the mode.
     */
    template<typename TMode, typename... TArgs>
    void switchMode(TArgs&&... args)
    {
        std::visit([](auto& mode) { mode.onExit(); }, m_current_mode);
        m_current_mode.template emplace<TMode>(std::forward<TArgs>(args)...);
        std::visit([](auto& mode) { mode.onEnter(); }, m_current_mode);
    }

    /**
     * @brief Runs the update loop for the current mode.
     *        If the mode is finished, it transitions back to Idle mode.
     */
    void update()
    {
        const bool finished = std::visit([](auto& mode) { return mode.update(); }, m_current_mode);

        if (finished)
        {
            switchMode<OperatingModeIdle>(); // Default to Idle when a mode finishes
        }
    }
};

// Actual operating modes here

class OperatingModeFollowTrajectory : public OperatingMode<OperatingModeFollowTrajectory>
{
private:
    RobotFacility& m_facility;

public:
    OperatingModeFollowTrajectory(RobotFacility& facility);
    void onEnterImpl();
    [[nodiscard]] bool updateImpl();
    void onExitImpl();
};

}