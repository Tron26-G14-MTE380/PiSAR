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
    static constexpr size_t kImuFifoBatchSize = 64;

    DifferentialDriveController& m_drive_controller;
    Imu& m_imu;
    ImuPlanarKinematicTracker<kPoseHistorySize> m_kinematic_tracker;

    Mutex m_drive_mutex;
    Mutex m_imu_mutex;
    Mutex m_kinematic_tracker_mutex;

    TaskHandle_t m_dc_update_task_handle;             ///< FreeRTOS task handle for drive controller update task.
    TaskHandle_t m_kt_update_task_handle;             ///< FreeRTOS task handle for kinematic tracker update task.

    BinarySemaphore m_imu_data_ready_sem;             ///< Semaphore to signal new IMU data is ready.

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
     * @param dc_update_task_priority Priority of the drive controller update task.
     * @param kt_update_task_priority Priority of the kinematic tracker update task.
     * @return True if initialization was successful, false otherwise.
     */
    inline bool initialize(UBaseType_t dc_update_task_priority, UBaseType_t kt_update_task_priority)
    {
        if (dc_update_task_priority < 0 || dc_update_task_priority > configMAX_PRIORITIES)
        {
            PISAR_LOG_ERROR("Drive controller update task priority %u is out of range", dc_update_task_priority);
            return false; // TODO ERROR CODE
        }

        if (kt_update_task_priority < 0 || kt_update_task_priority > configMAX_PRIORITIES)
        {
            PISAR_LOG_ERROR("Kinematic tracker update task priority %u is out of range", kt_update_task_priority);
            return false; // TODO ERROR CODE
        }

        m_drive_controller.initialize();
        
        if (!m_imu.initialize())
        {
            PISAR_LOG_ERROR("Failed to initialize IMU");
            return false;
        }

        if (!m_imu.setFifoWatermarkInterrupt(kImuFifoBatchSize, [this]() {
            BaseType_t higher_priority_task_woken = pdFALSE;
            m_imu_data_ready_sem.unlockIsr(&higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }))
        {
            PISAR_LOG_ERROR("Failed to set IMU FIFO watermark interrupt");
            return false;
        }

        // Spawn the processing task
        if (xTaskCreate(driveControllerUpdateTaskEntry, "dc_update_task", 2048, this, dc_update_task_priority, &m_dc_update_task_handle) != pdPASS)
        {
            PISAR_LOG_ERROR("Failed to create drive controller update task");
            return false;
        }

        if (xTaskCreate(kinematicTrackerUpdateTaskEntry, "kt_update_task", 16384, this, kt_update_task_priority, &m_kt_update_task_handle) != pdPASS)
        {
            PISAR_LOG_ERROR("Failed to create kinematic tracker update task");
            return false;
        }

        m_drive_controller.enable();
        return true;
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

    /// @brief Stops robot abruptly.
    inline void driveHardStop()
    {
        Lock<Mutex> lock(m_drive_mutex);
        m_drive_controller.hardStop();
    }

    /// @brief Retrieves the robot's calculated velocity in m/s (thread-safe).
    [[nodiscard]] Eigen::Vector2f getVelocity()
    {
        Lock<Mutex> lock(m_kinematic_tracker_mutex);
        return m_kinematic_tracker.getVelocity();
    }

    /// @brief Retrieves the robot's acceleration in m/s^2 (thread-safe).
    [[nodiscard]] Eigen::Vector2f getAcceleration()
    {
        Lock<Mutex> lock(m_kinematic_tracker_mutex);
        return m_kinematic_tracker.getAcceleration();
    }

    /// @brief Retrieves the robot's calculated position (thread-safe).
    [[nodiscard]] Eigen::Vector2f getPosition()
    {
        Lock<Mutex> lock(m_kinematic_tracker_mutex);
        return m_kinematic_tracker.getPosition();
    }

    /// @brief Retrieves the robot's calculated orientation in deg/s (thread-safe).
    [[nodiscard]] float getOrientation()
    {
        Lock<Mutex> lock(m_kinematic_tracker_mutex);
        return m_kinematic_tracker.getOrientation();
    }

    /// @brief Retrieves the robot's angular velocity in deg/s (thread-safe).
    [[nodiscard]] float getAngularVelocity()
    {
        Lock<Mutex> lock(m_kinematic_tracker_mutex);
        return m_kinematic_tracker.getAngularVelocity();
    }

private:
    /**
     * @brief Reads IMU data and updates IMU kinematic tracker in a thread-safe way.
     */
    void updateKinematicTracker();

    /**
     * @brief Entry point for the drive controller update task.
     */
    static inline void driveControllerUpdateTaskEntry(void* param)
    {
        reinterpret_cast<RobotFacility*>(param)->driveControllerUpdateTaskLoop();
    }

    /**
     * @brief Main loop for the drive controller update task.
     */
    inline void driveControllerUpdateTaskLoop()
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

    /**
     * @brief Entry point for the kinematic tracker task.
     */
    static inline void kinematicTrackerUpdateTaskEntry(void* param)
    {
        reinterpret_cast<RobotFacility*>(param)->kinematicTrackerUpdateTaskLoop();
    }

    /**
     * @brief Main loop for the kinematic tracker update task.
     */
    inline void kinematicTrackerUpdateTaskLoop()
    {
        while (true)
        {
            if (m_imu_data_ready_sem.lock())
            {
                updateKinematicTracker();

                if (m_imu.fifoSamplesAvailable() > kImuFifoBatchSize)
                {
                    m_imu_data_ready_sem.unlock();
                }
            }
        }
    }

};

} // namespace pisar::driveunit
