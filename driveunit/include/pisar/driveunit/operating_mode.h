#pragma once

#include "pisar/driveunit/facility.h"
#include "pisar/driveunit/pid.h"

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

class OperatingModeHardStop : public OperatingMode<OperatingModeHardStop>
{
private:
    std::reference_wrapper<RobotFacility> m_facility;

public:
    OperatingModeHardStop(RobotFacility& facility) : m_facility(facility) {}

    inline void onEnterImpl()
    {
        m_facility.get().getDriveController().hardStop();
    }

    [[nodiscard]] bool updateImpl() {}
    void onExitImpl() {}
};

class OperatingModeFollowTrajectory : public OperatingMode<OperatingModeFollowTrajectory>
{
private:
    static constexpr float kThetaTolerance = 5.0f;
    static constexpr float kDistTolerance = 0.0000001f;
    static constexpr float kDotProductTolerance = 120.0f;
    static constexpr float kAngleLockThreshold = 35.0f;

    static constexpr float kPidkpRotation = 0.0015f;
    static constexpr float kPidkiRotation = 0.0f;
    static constexpr float kPidkdRotation = 0.0004f;

    static constexpr float kPidkpTravel = 0.01f;
    static constexpr float kPidkiTravel = 0.0f;
    static constexpr float kPidkdTravel = 0.0f;

    std::reference_wrapper<RobotFacility> m_facility;
    std::vector<Eigen::Vector2f> m_trajectory;
    std::chrono::microseconds m_reference_time;

    PidController m_pid_rotation;
    PidController m_pid_travel;
    

    int m_target_index;

    std::chrono::milliseconds m_last_update_time;

    [[nodiscard]] inline const Eigen::Vector2f& getCurrentTarget() const
    {
        return m_trajectory[m_target_index];
    }

public:
    OperatingModeFollowTrajectory(
        RobotFacility& facility,
        const std::span<const Eigen::Vector2f> trajectory,
        const std::chrono::duration<float> reference_time
    );
    void onEnterImpl();
    [[nodiscard]] bool updateImpl();
    void onExitImpl();

    [[nodiscard]] inline std::chrono::duration<float> updateDeltaTime() 
    {
        const auto current_time = std::chrono::milliseconds(millis());
        std::chrono::duration<float> elapsed = current_time - m_last_update_time;
        m_last_update_time = current_time;
        return elapsed;
    }
};

class OperatingModeGoToTarget : public OperatingMode<OperatingModeGoToTarget>
{
private:

    static constexpr float kThetaTolerance = 3.0f;
    static constexpr float kDistTolerance = 0.0005f;
    static constexpr auto kOnTargetDurationTolerance = std::chrono::milliseconds(10);

    static constexpr float kPidkpRotation = 0.000005f;
    static constexpr float kPidkiRotation = 0.0f;
    static constexpr float kPidkdRotation = 0.0001f;

    static constexpr float kPidkpTravel = 0.01f;
    static constexpr float kPidkiTravel = 0.0f;
    static constexpr float kPidkdTravel = 0.0f;

    std::reference_wrapper<RobotFacility> m_facility;
    Eigen::Vector2f m_target;

    PidController m_pid_rotation;
    PidController m_pid_travel;

    float m_distance_to_target;

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

    [[nodiscard]] inline std::chrono::duration<float> updateDeltaTime() 
    {
        const auto current_time = std::chrono::milliseconds(millis());
        std::chrono::duration<float> elapsed = current_time - m_last_update_time;
        m_last_update_time = current_time;
        return elapsed;
    }
};

class OperatingModeRotate: public OperatingMode<OperatingModeRotate>
{
private:

    static constexpr float kPidkp = 0.0007f;
    static constexpr float kPidki = 0.0f;
    static constexpr float kPidkd = 0.0001f;

    static constexpr float kTolerance = 3.0f;
    static constexpr auto kOnTargetDurationTolerance = std::chrono::milliseconds(500);

    std::reference_wrapper<RobotFacility> m_facility;
    float m_rotation_deg;

    PidController m_pid;

    std::chrono::milliseconds m_last_update_time;
    std::optional<std::chrono::milliseconds> m_on_target_timestamp;

public:
    OperatingModeRotate(RobotFacility& facility, float rotation_deg);
    void onEnterImpl();
    [[nodiscard]] bool updateImpl();
    void onExitImpl();

    [[nodiscard]] inline std::chrono::duration<float> updateDeltaTime() 
    {
        const auto current_time = std::chrono::milliseconds(millis());
        std::chrono::duration<float> elapsed = current_time - m_last_update_time;
        m_last_update_time = current_time;
        return elapsed;
    }    
};

class OperatingModeSetGripper : public OperatingMode<OperatingModeSetGripper>
{
private:
    std::reference_wrapper<RobotFacility> m_facility;
    bool m_open;

public:
    OperatingModeSetGripper(RobotFacility& facility, bool open) : m_facility(facility), m_open(open) {}

    inline void onEnterImpl()
    {
        if (m_open)
        {
            m_facility.get().getGripperController().open();
        }
        else
        {
            m_facility.get().getGripperController().close();
        }
    }

    [[nodiscard]] bool updateImpl() {}
    void onExitImpl() {}
};

}