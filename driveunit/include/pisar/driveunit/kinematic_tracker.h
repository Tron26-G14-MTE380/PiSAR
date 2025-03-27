#pragma once

#include "pisar/driveunit/imu.h"
#include "pisar/driveunit/pose_history.h"
#include "pisar/driveunit/filesystem_saveable.h"
#include "pisar/low_pass_filter.h"

#include <cstdint>

namespace pisar::driveunit {

struct KinematicState
{
    KinematicPose pose;

    Eigen::Vector2f velocity;
    float angular_velocity;

    Eigen::Vector2f acceleration;

    static inline KinematicState zero()
    {
        return {
            .pose = KinematicPose{Eigen::Vector2f::Zero(), 0.0f},
            .velocity = Eigen::Vector2f::Zero(),
            .angular_velocity = 0.0f,
            .acceleration = Eigen::Vector2f::Zero()
        };
    } 

};

/**
 * @brief Provides position and orientation tracking using an Imu.
 */
class ImuPlanarKinematicTracker
{
public:
    static constexpr size_t kPoseHistorySize = 128;

    using TimestampT = std::chrono::microseconds;
    using PoseHistoryT = PoseHistory<TimestampT, kPoseHistorySize>;

    struct CalibrationData : FileSystemSaveable<CalibrationData> 
    {
        using serialize = zpp::bits::members<2>;

        Eigen::Vector2f velocity_slope;
        Eigen::Vector2f position_slope;
    };

private:
    static constexpr size_t kImuFifoBatchSize = 16;
    static constexpr float kAccelLpfCutoffFreq = 5.0f;
    static constexpr float kGyroLpfCutoffFreq = 10.0f;

    std::reference_wrapper<Imu> m_imu;                              ///< Imu reference.
    std::chrono::duration<float> m_sample_time;                     ///< IMU sampling time.
    std::string_view m_calibration_data_file_path;                  ///< Calibration data filepath.
    TimestampT m_last_update_time;                                  ///< Timestamp of last update.
    KinematicState m_current_state;                                 ///< The current kinematic state of the robot.
    PoseHistoryT m_pose_history;                                    ///< Tracks the pose history.

    LowPassFilter<Eigen::Vector3f> m_accel_filter;                  ///< Accelerometer data digital low pass filter.
    LowPassFilter<Eigen::Vector3f> m_gyro_filter;                   ///< Gyroscope data digital low pass filter.

    CalibrationData m_calibration_data;                             ///< Calibration data.

    mutable Mutex m_mutex;
    TaskHandle_t m_update_task_handle;                              ///< FreeRTOS task handle for kinematic tracker update task.
    BinarySemaphore m_imu_data_ready_sem;                           ///< Semaphore to signal new IMU data is ready.

public:
    /**
     * @brief Constructs a ImuPlanarKinematicTracker.
     */
    constexpr ImuPlanarKinematicTracker(Imu& imu, std::string_view calibration_data_file_path) :
        m_imu(imu),
        m_sample_time(m_imu.get().getSampleTime()),
        m_calibration_data_file_path(calibration_data_file_path),
        m_last_update_time(0),
        m_current_state(KinematicState::zero()),
        m_pose_history(),
        m_accel_filter(kAccelLpfCutoffFreq, m_imu.get().getSampleTime()),
        m_gyro_filter(kGyroLpfCutoffFreq, m_imu.get().getSampleTime()),
        m_update_task_handle(nullptr)
        {}

    /// @brief Destructor
    ~ImuPlanarKinematicTracker();

    [[nodiscard]] bool initialize(BaseType_t update_task_priority);

    [[nodiscard]] bool calibrate(const size_t num_batches, const size_t slope_sample_batch_size, const bool save = true);

    /**
     * @brief Sets the calibration data.
     * @param calib_data The calibration data to set.
     */
    inline void setCalibration(const CalibrationData& calib_data)
    {
        Lock<Mutex> lock(m_mutex);
        m_calibration_data = calib_data;
    }

    /// @brief Gets the calibration data.
    [[nodiscard]] inline CalibrationData getCalibration() const
    {
        Lock<Mutex> lock(m_mutex);
        return m_calibration_data;
    }

    /// @brief Get whether the calibration data has been saved.
    [[nodiscard]] inline bool calibrationDataSaved() const
    {
        return LittleFS.exists(m_calibration_data_file_path.data());
    }

    /**
     * @brief Loads the calibration data from the filesystem.
     * @return True if the data was successfully loaded, false otherwise.
     */
    [[nodiscard]] inline bool loadCalibrationData()
    {
        Lock<Mutex> lock(m_mutex);

        return m_calibration_data.load(m_calibration_data_file_path);
    }

    /**
     * @brief Saves the calibration data to the filesystem.
     * @return True if the data was successfully saved, false otherwise.
     */
    [[nodiscard]] inline bool saveCalibrationData() const
    {
        Lock<Mutex> lock(m_mutex);
        
        return m_calibration_data.save(m_calibration_data_file_path);
    }

    /**
     * @brief Sets the position and orientation estimate reference point to provided pose @p ref.
     *
     * @param ref The reference pose to set.
     */
    inline void setPoseReference(const KinematicPose& ref)
    {
        Lock<Mutex> lock(m_mutex);

        m_current_state.pose.position -= ref.position;
        m_current_state.pose.orientation -= ref.orientation;
        m_pose_history.setReference(ref);
    }

    /// @brief Sets the position and orientation estimate reference point to current pose. Same effect as resetting them.
    inline void setPoseReference()
    {
        Lock<Mutex> lock(m_mutex);

        m_pose_history.setReference(m_current_state.pose);
        m_current_state.pose.position = Eigen::Vector2f::Zero();
        m_current_state.pose.orientation = 0.0f;
    }

    /**
     * @brief Resets the tracker state.
     * @param clear_history Whether to clear the pose history.
     */
    inline void reset(bool clear_history = false)
    {
        Lock<Mutex> lock(m_mutex);

        m_current_state = KinematicState::zero();
        m_last_update_time = TimestampT(0);
        m_accel_filter = LowPassFilter<Eigen::Vector3f>(kAccelLpfCutoffFreq, m_sample_time);
        m_gyro_filter = LowPassFilter<Eigen::Vector3f>(kGyroLpfCutoffFreq, m_sample_time);

        if (clear_history)
        {
            m_pose_history.clearHistory();
        }
    }

    /// @brief Gets the current kinematic state (all variables).
    [[nodiscard]] inline KinematicState getState() const noexcept
    {
        Lock<Mutex> lock(m_mutex);
        return m_current_state;
    }

    /// @brief Gets the current pose (position and orientation) with respect to the reference.
    [[nodiscard]] inline KinematicPose getPose() const noexcept
    {
        Lock<Mutex> lock(m_mutex);
        return m_current_state.pose;
    }

    /// @brief Gets the pose (position and orientation) history with respect to the reference.
    [[nodiscard]] inline const PoseHistoryT& getPoseHistory() const noexcept
    {
        Lock<Mutex> lock(m_mutex);
        return m_pose_history;
    }

    /// @brief Gets the current position estimate with respect to the reference.
    [[nodiscard]] inline Eigen::Vector2f getPosition() const noexcept
    {
        Lock<Mutex> lock(m_mutex);
        return m_current_state.pose.position;
    }

    /// @brief Gets the current orientation estimate (heading) in radians with respect to the reference.
    [[nodiscard]] inline float getOrientation() const noexcept
    {
        Lock<Mutex> lock(m_mutex);
        return m_current_state.pose.orientation;
    }

    /// @brief Gets the current velocity estimate.
    [[nodiscard]] inline Eigen::Vector2f getVelocity() const noexcept
    {
        Lock<Mutex> lock(m_mutex);
        return m_current_state.velocity;
    }

    /// @brief Gets the current angular velocity estimate in radians/sec.
    [[nodiscard]] inline float getAngularVelocity() const noexcept
    {
        Lock<Mutex> lock(m_mutex);
        return m_current_state.angular_velocity;
    }

    /// @brief Gets the current acceleration estimate.
    [[nodiscard]] inline Eigen::Vector2f getAcceleration() const noexcept
    {
        Lock<Mutex> lock(m_mutex);
        return m_current_state.acceleration;
    }

    /// @brief The timestamp of the last update.
    [[nodiscard]] inline TimestampT getLastUpdateTime() const noexcept
    {
        Lock<Mutex> lock(m_mutex);
        return m_last_update_time;
    }

    [[nodiscard]]  inline std::optional<KinematicPose> poseAtNearest(TimestampT time) const noexcept
    {
        Lock<Mutex> lock(m_mutex);
        return m_pose_history.poseAtNearest(time);
    }

private:
    [[nodiscard]] CalibrationData calibrateImpl(const size_t num_batches, const size_t slope_sample_batch_size);

    /**
     * @brief Entry point for the kinematic tracker task.
     */
    static inline void updateTaskEntry(void* param)
    {
        reinterpret_cast<ImuPlanarKinematicTracker*>(param)->updateTaskLoop();
    }

    /**
     * @brief Main loop for the kinematic tracker update task.
     */
    void updateTaskLoop();

    void onImuDataReady();
    /**
     * @brief Updates the internal kinematic state estimates using new IMU data.
     *
     * @param data The new accelerometer and gyroscope data reading.
     * @param timestamp The imu data timestamp.
     */
    void onImuReading(const Imu::AccelData& accel_data, const Imu::GyroData& gyro_data, TimestampT timestamp);
};

}