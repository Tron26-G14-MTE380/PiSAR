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

    TaskHandle_t m_dc_update_task_handle;             ///< FreeRTOS task handle for drive controller update task.

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
        : m_drive_controller(drive_controller), m_imu(imu), m_kinematic_tracker(imu.getSampleTime()), m_dc_update_task_handle(nullptr)
    {
    }

    /**
     * @brief Initializes all hardware components.
     *
     * @param dc_update_task_priority Priority of the drive controller update thread.
     */
    inline void initialize(UBaseType_t dc_update_task_priority)
    {
        m_drive_controller.initialize();
        m_imu.initialize();

        if (dc_update_task_priority < 0 || dc_update_task_priority > configMAX_PRIORITIES)
        {
            PISAR_LOG_ERROR("Drive controller update task priority %u is out of range", dc_update_task_priority);
            return; // TODO ERROR CODE
        }

        // Spawn the processing task
        if (xTaskCreate(driveControllerUpdateTaskEntry, "drive_controller_update_task", 2048, this, dc_update_task_priority, &m_dc_update_task_handle) != pdPASS)
        {
            PISAR_LOG_ERROR("Failed to create drive controller update task");
            return;
        }

        m_drive_controller.enable();
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

    /**
     * @brief Drives the robot using tank-style controls.
     *
     * @param left_speed The left wheel speed (-1.0 to 1.0).
     * @param right_speed The right wheel speed (-1.0 to 1.0).
     */
    inline void tankDrive(const float left_speed, const float right_speed)
    {
        Lock<Mutex> lock(m_drive_mutex);
        m_drive_controller.tankDrive(left_speed, right_speed);
    }

    /// @brief Stops robot smoothly.
    inline void driveStop()
    {
        Lock<Mutex> lock(m_drive_mutex);
        m_drive_controller.stop();
    }

    /// @brief Stops robot smoothly.
    inline void driveHardStop()
    {
        Lock<Mutex> lock(m_drive_mutex);
        m_drive_controller.hardStop();
    }

private:
    /**
     * @brief Reads IMU data and updates IMU kinematic tracker in a thread-safe way.
     */
    void updateKinematicTracker();

    /**
     * @brief Main loop for the processing task.
     */
    static void driveControllerUpdateTaskEntry(void* param)
    {
        reinterpret_cast<RobotFacility*>(param)->driveControllerUpdateTaskLoop();
    }

    /**
     * @brief Main loop for the processing task.
     */
    void driveControllerUpdateTaskLoop()
    {
        while (true)
        {
            {
                Lock<Mutex> lock(m_drive_mutex);
                m_drive_controller.update();
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

};

} // namespace pisar::driveunit
