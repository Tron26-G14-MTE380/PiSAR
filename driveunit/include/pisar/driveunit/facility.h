#pragma once

#include "pisar/driveunit/drive_controller.h"
#include "pisar/driveunit/imu.h"
#include "pisar/driveunit/kinematic_tracker.h"

#include "CoreMutex.h"

namespace pisar::driveunit
{

/**
 * @brief Manages hardware interfaces for the driveunit, including motors, IMU, and pose estimation.
 */
template<std::size_t tkPoseHistorySize>
class RobotFacility
{
private:
    DifferentialDriveController& m_drive_controller;
    Imu& m_imu;
    ImuPlanarKinematicTracker<tkPoseHistorySize>& m_kinematic_tracker;

    mutex_t m_drive_mutex;
    mutex_t m_imu_mutex;
    mutex_t m_kinematic_tracker_mutex;

public:
    /**
     * @brief Constructs the RobotFacility object.
     * @param spi The SPI bus instance for the IMU.
     * @param imu_cs_pin The chip select pin for the IMU.
     */
    inline RobotFacility(
        DifferentialDriveController& drive_controller,
        Imu imu,
        ImuPlanarKinematicTracker<tkPoseHistorySize> kinematic_tracker
    )
        : m_drive_controller(drive_controller), m_imu(imu), m_kinematic_tracker(kinematic_tracker)
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
    inline mutex_t& getMotorControllerMutex()
    {
        return m_drive_mutex;
    }

    /// @brief Gets a reference to the robot IMU sensor.
    inline Imu& getImu()
    {
        return m_imu;
    }

    /// @brief Gets a refernce to the robot IMU sensor mutex primitive.
    inline mutex_t& getImuMutex()
    {
        return m_imu_mutex;
    }

    /// @brief Gets a reference to the robot kinematic tracker.
    inline ImuPlanarKinematicTracker<tkPoseHistorySize>& getKinematicTracker()
    {
        return m_kinematic_tracker;
    }

    /// @brief Gets a refernce to the kinematic tracker mutex primitive.
    inline mutex_t& getKinematicTrackerMutex()
    {
        return m_kinematic_tracker_mutex;
    }

    /// @brief Updates the drive controller.
    void updateDriveController()
    {
        CoreMutex lock(m_drive_mutex);
        m_drive_controller.update();
    }

    /**
     * @brief Reads IMU data and updates IMU kinematic tracker in a thread-safe way.
     */
    void updateKinematicTracker()
    {
        uint32_t sample_time_us = 0;
        uint64_t time_stamp_us = 0;
        std::array<Imu::Data, 128> imu_data;
        size_t data_samples = 0;

        CoreMutex lock(m_kinematic_tracker_mutex);

        {
            CoreMutex lock(m_imu_mutex);
            sample_time_us = m_imu.getSampleTimeUs();
            time_stamp_us = micros(); // Get current timestamp
            data_samples = m_imu.readFifo(std::span(imu_data));
        }

        for (int i = 0; i < data_samples; ++i)
        {
            m_kinematic_tracker.onImuReading(
                imu_data[i].accel_data,
                imu_data[i].gyro_data,
                time_stamp_us - (data_samples - 1 - i) * sample_time_us
            );
        }
    }
};

} // namespace pisar::driveunit
