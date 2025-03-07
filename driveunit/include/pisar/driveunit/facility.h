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
    static constexpr size_t kPoseHistorySize = 128;

    DifferentialDriveController& m_drive_controller;
    Imu& m_imu;
    ImuPlanarKinematicTracker<kPoseHistorySize> m_kinematic_tracker;

    Mutex m_drive_mutex;
    Mutex m_imu_mutex;
    Mutex m_kinematic_tracker_mutex;

public:
    /**
     * @brief Constructs the RobotFacility object.
     * @param spi The SPI bus instance for the IMU.
     * @param imu_cs_pin The chip select pin for the IMU.
     */
    inline RobotFacility(
        DifferentialDriveController& drive_controller,
        Imu& imu
    )
        : m_drive_controller(drive_controller), m_imu(imu), m_kinematic_tracker(imu.getSampleTime())
    {
    }

    /// @brief Initializes all hardware components.
    inline void initialize()
    {
        m_drive_controller.initialize();
        m_imu.initialize();
    }

    /// @brief Gets a reference to the robot motor controller.
    inline DifferentialDriveController& getMotorController()
    {
        return m_drive_controller;
    }

    /// @brief Gets a refernce to the robot motor controller mutex primitive.
    inline Mutex& getMotorControllerMutex()
    {
        return m_drive_mutex;
    }

    /// @brief Gets a reference to the robot IMU sensor.
    inline Imu& getImu()
    {
        return m_imu;
    }

    /// @brief Gets a refernce to the robot IMU sensor mutex primitive.
    inline Mutex& getImuMutex()
    {
        return m_imu_mutex;
    }

    /// @brief Gets a reference to the robot kinematic tracker.
    inline ImuPlanarKinematicTracker<kPoseHistorySize>& getKinematicTracker()
    {
        return m_kinematic_tracker;
    }

    /// @brief Gets a refernce to the kinematic tracker mutex primitive.
    inline Mutex& getKinematicTrackerMutex()
    {
        return m_kinematic_tracker_mutex;
    }

    /// @brief Updates the drive controller.
    void updateDriveController()
    {
        Lock<Mutex> lock(m_drive_mutex);
        m_drive_controller.update();
    }

    /**
     * @brief Reads IMU data and updates IMU kinematic tracker in a thread-safe way.
     */
    void updateKinematicTracker();
};

} // namespace pisar::driveunit
