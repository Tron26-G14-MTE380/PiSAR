#pragma once

#include "pisar/driveunit/kinematic_tracker.h"
#include "pisar/driveunit/imu.h"
#include "pisar/driveunit/drive_controller.h"
#include "pisar/driveunit/gripper_controller.h"
#include "pisar/driveunit/sync.h"

namespace pisar::driveunit
{

/**
 * @brief Manages hardware interfaces for the driveunit, including motors, IMU, and pose estimation.
 */
class RobotFacility
{
private:
    std::reference_wrapper<DifferentialDriveController> m_drive_controller;
    std::reference_wrapper<ImuPlanarKinematicTracker> m_kinematic_tracker;
    std::reference_wrapper<GripperController> m_gripper_controller;

public:
    /**
     * @brief Constructs the RobotFacility object.
     * @param drive_controller Reference to the drive controller.]
     * @param kinematic_tracker Reference to the kinematic tracker.
     */
    inline RobotFacility(
        DifferentialDriveController& drive_controller,
        ImuPlanarKinematicTracker& kinematic_tracker,
        GripperController& gripper_controller
    )
        : m_drive_controller(drive_controller), m_kinematic_tracker(kinematic_tracker), m_gripper_controller(gripper_controller)
    {
    }

    /**
     * @brief Initializes all hardware components.
     *
     * @return True if initialization was successful, false otherwise.
     */
    inline bool initialize()
    {
        // Possible future shenanigans ELEPHANT
        return true;
    }

    /// @brief Gets a reference to the robot drive controller.
    inline DifferentialDriveController& getDriveController()
    {
        return m_drive_controller;
    }

    /// @brief Gets a reference to the robot kinematic tracker.
    inline ImuPlanarKinematicTracker& getKinematicTracker()
    {
        return m_kinematic_tracker;
    }

    /// @brief Gets a reference to the gripper controller.
    inline GripperController& getGripperController()
    {
        return m_gripper_controller;
    }

};

} // namespace pisar::driveunit
