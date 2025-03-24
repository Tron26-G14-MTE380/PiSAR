#pragma once

#include "pisar/driveunit/kinematic_tracker.h"
#include "pisar/driveunit/imu.h"
#include "pisar/driveunit/drive_controller.h"
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


public:
    /**
     * @brief Constructs the RobotFacility object.
     * @param spi The SPI bus instance for the IMU.
     * @param imu_cs_pin The chip select pin for the IMU.
     */
    inline RobotFacility(
        DifferentialDriveController& drive_controller,
        ImuPlanarKinematicTracker& kinematic_tracker
    )
        : m_drive_controller(drive_controller), m_kinematic_tracker(kinematic_tracker)
    {
    }

    /**
     * @brief Initializes all hardware components.
     *
     * @return True if initialization was successful, false otherwise.
     */
    inline bool initialize()
    {
        return true;
    }

    /// @brief Gets a reference to the robot drive controller.
    inline DifferentialDriveController& getDriveController()
    {
        return m_drive_controller.get();
    }

    /// @brief Gets a reference to the robot kinematic tracker.
    inline ImuPlanarKinematicTracker& getKinematicTracker()
    {
        return m_kinematic_tracker;
    }

};

} // namespace pisar::driveunit
