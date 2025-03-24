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

    mutable Mutex m_drive_mutex;

    TaskHandle_t m_dc_update_task_handle;             ///< FreeRTOS task handle for drive controller update task.


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
        : m_drive_controller(drive_controller), m_kinematic_tracker(kinematic_tracker), m_dc_update_task_handle(nullptr)
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

        // Spawn the processing task
        if (xTaskCreate(driveControllerUpdateTaskEntry, "dc_update_task", 2048, this, dc_update_task_priority, &m_dc_update_task_handle) != pdPASS)
        {
            PISAR_LOG_ERROR("Failed to create drive controller update task");
            return false;
        }

        m_drive_controller.get().enable();
        return true;
    }

    /// @brief Gets a reference to the robot motor controller.
    inline DifferentialDriveController& getMotorController()
    {
        return m_drive_controller.get();
    }

    /// @brief Gets a refernce to the robot motor controller mutex primitive.
    inline Mutex& getMotorControllerMutex()
    {
        return m_drive_mutex;
    }

    /// @brief Gets a reference to the robot kinematic tracker.
    inline ImuPlanarKinematicTracker& geImuPlanarKinematicTracker()
    {
        return m_kinematic_tracker;
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
        m_drive_controller.get().tankDrive(left_speed, right_speed);
    }

    /// @brief Stops robot smoothly.
    inline void driveStop()
    {
        Lock<Mutex> lock(m_drive_mutex);
        m_drive_controller.get().stop();
    }

    /// @brief Stops robot abruptly.
    inline void driveHardStop()
    {
        Lock<Mutex> lock(m_drive_mutex);
        m_drive_controller.get().hardStop();
    }

    [[nodiscard]] inline float getMinSpeed() const 
    { 
        Lock<Mutex> lock(m_drive_mutex);
        return m_drive_controller.get().getMinSpeed(); 
    }

    [[nodiscard]] inline float getMaxSpeed() const 
    { 
        Lock<Mutex> lock(m_drive_mutex);
        return m_drive_controller.get().getMaxSpeed(); 
    } 

    [[nodiscard]] inline float getSpeedRange() const 
    { 
        Lock<Mutex> lock(m_drive_mutex);
        return m_drive_controller.get().getSpeedRange(); 
    } 

    /// @brief Retrieves the robot's calculated velocity in m/s (thread-safe).
    [[nodiscard]] Eigen::Vector2f getVelocity() const
    {
        return m_kinematic_tracker.get().getVelocity();
    }

    /// @brief Retrieves the robot's acceleration in m/s^2 (thread-safe).
    [[nodiscard]] Eigen::Vector2f getAcceleration() const
    {
        return m_kinematic_tracker.get().getAcceleration();
    }

    /// @brief Retrieves the robot's pose.
    [[nodiscard]] KinematicPose getPose() const
    {
        return m_kinematic_tracker.get().getPose();
    }

    /// @brief Retrieves the robot's calculated position (thread-safe).
    [[nodiscard]] Eigen::Vector2f getPosition() const
    {
        return m_kinematic_tracker.get().getPosition();
    }

    /// @brief Retrieves the robot's calculated orientation in deg/s (thread-safe).
    [[nodiscard]] float getOrientation() const
    {
        return m_kinematic_tracker.get().getOrientation();
    }

    /// @brief Retrieves the robot's angular velocity in deg/s (thread-safe).
    [[nodiscard]] float getAngularVelocity() const
    {
        return m_kinematic_tracker.get().getAngularVelocity();
    }

    void setPoseReference()
    {
        m_kinematic_tracker.get().setPoseReference();
    }

    void setPoseReference(const KinematicPose& ref)
    {
        m_kinematic_tracker.get().setPoseReference(ref);
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
                m_drive_controller.get().update();
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

};

} // namespace pisar::driveunit
