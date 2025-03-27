#include "pisar/driveunit/kinematic_tracker.h"

namespace pisar::driveunit {

[[nodiscard]] bool ImuPlanarKinematicTracker::initialize(BaseType_t update_task_priority)
{
    if (update_task_priority < 0 || update_task_priority > configMAX_PRIORITIES)
    {
        PISAR_LOG_ERROR("Kinematic tracker update task priority %u is out of range", update_task_priority);
        return false; // TODO ERROR CODE
    }

    // Load calibration data
    if (calibrationDataSaved())
    {
        PISAR_LOG_INFO("Kinematic tracker calibration data saved to %s, loading...", m_calibration_data_file_path.data());
        if (!loadCalibrationData())
        {
            PISAR_LOG_ERROR("Failed to load calibration data!");
            return false;
        }
        PISAR_LOG_INFO("Calibration data successfully loaded!");
    }
    else
    {
        PISAR_LOG_WARN("No calibration data found at %s, call calibrate to calibrate kinematic tracker", m_calibration_data_file_path.data());
        // Fresh calibration
        m_calibration_data = {
            .velocity_slope = {0.0f, 0.0f},
            .position_slope = {0.0f, 0.0f}
        };
    }

    if (!m_imu.get().setFifoWatermarkInterrupt(kImuFifoBatchSize, [this]() {
        BaseType_t higher_priority_task_woken = pdFALSE;
        m_imu_data_ready_sem.unlockIsr(&higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }))
    {
        PISAR_LOG_ERROR("Failed to set IMU FIFO watermark interrupt");
        return false;
    }

    if (xTaskCreate(updateTaskEntry, "kt_update_task", 16384, this, update_task_priority, &m_update_task_handle) != pdPASS)
    {
        PISAR_LOG_ERROR("Failed to create kinematic tracker update task");
        return false;
    }

    // Initially unlock incase fifo is past threshold
    m_imu_data_ready_sem.unlock();

    return true;
};


void ImuPlanarKinematicTracker::updateTaskLoop()
{
    while (true)
    {
        if (m_imu_data_ready_sem.lock())
        {
            onImuDataReady();

            if (m_imu.get().fifoSamplesAvailable() > kImuFifoBatchSize)
            {
                m_imu_data_ready_sem.unlock();
            }
        }
    }
}

void ImuPlanarKinematicTracker::onImuDataReady()
{
    std::array<Imu::Data, kImuFifoBatchSize> imu_data;
    const auto time_stamp = std::chrono::microseconds(micros()); // Get current timestamp
    const size_t data_samples = m_imu.get().readFifoPaired(std::span(imu_data));

    if (data_samples == 0)
    {
        return;
    }

    Lock<Mutex> lock(m_mutex);

    for (int i = 0; i < data_samples; ++i)
    {
        onImuReading(
            imu_data[i].accel_data,
            imu_data[i].gyro_data,
            time_stamp - (data_samples - 1 - i) * std::chrono::duration_cast<std::chrono::microseconds>(m_sample_time)
        );
    }
}

void ImuPlanarKinematicTracker::onImuReading(const Imu::AccelData& accel_data, const Imu::GyroData& gyro_data, TimestampT timestamp)
{
    if (m_last_update_time.count() == 0)
    {
        m_last_update_time = timestamp;
        m_current_state = KinematicState::zero(); // ensure clean state
        return; // Skip first update
    }

    const auto oriented_accel_data = Eigen::Vector3f(-accel_data.values.y(), accel_data.values.x() , accel_data.values.z());

    // --- APPLY LOW-PASS FILTERS TO ACCEL & GYRO ---
    const auto filtered_accel_data = m_accel_filter.update(oriented_accel_data);
    const auto filtered_gyro_data = m_gyro_filter.update(gyro_data.values);

    m_current_state.angular_velocity = filtered_gyro_data.z();
    m_current_state.acceleration = filtered_accel_data.head<2>(); // Take X and Y

    // --- INTEGRATE VELOCITY ---
    m_current_state.velocity += (m_current_state.acceleration - m_calibration_data.velocity_slope) * m_sample_time.count() * 0.5f;

    // --- INTEGRATE POSITION ---
    m_current_state.pose.position += (m_current_state.velocity - m_calibration_data.position_slope) * m_sample_time.count() * 0.5f;

    // -- INTEGRATE ANGULAR VELOCITY ---
    m_current_state.pose.orientation += m_current_state.angular_velocity * m_sample_time.count() * 0.5f;

    // Store updated state
    m_pose_history.addRecord(timestamp, m_current_state.pose);
}

[[nodiscard]] ImuPlanarKinematicTracker::CalibrationData ImuPlanarKinematicTracker::calibrateImpl(const size_t num_batches, const size_t slope_sample_batch_size)
{
    const std::chrono::duration<float> batch_time = m_sample_time * slope_sample_batch_size;

    Eigen::Vector2f velocity_slope_sum = Eigen::Vector2f::Zero();
    Eigen::Vector2f position_slope_sum = Eigen::Vector2f::Zero();

    for (size_t batch_num = 0; batch_num < num_batches; ++batch_num)
    {
        reset();

        // Let it update from fifo
        delay(std::chrono::duration_cast<std::chrono::milliseconds>(batch_time).count());

        velocity_slope_sum += getVelocity() / batch_time.count();
        position_slope_sum += getPosition() / batch_time.count();
    }

    return CalibrationData{
        .velocity_slope = velocity_slope_sum / static_cast<float>(num_batches), 
        .position_slope = position_slope_sum / static_cast<float>(num_batches)
    };
};

[[nodiscard]] bool ImuPlanarKinematicTracker::calibrate(const size_t num_batches, const size_t slope_sample_batch_size, const bool save)
{    
    CalibrationData calibration_data = {
        .velocity_slope = {0.0f, 0.0f},
        .position_slope = {0.0f, 0.0f}
    };

    setCalibration(calibration_data);

    constexpr int kDelaySeconds = 3;
    PISAR_LOG_INFO("Starting Kinematic tracker calibration in %d seconds... Keep the IMU **completely still**!", kDelaySeconds);
    delay(kDelaySeconds * 1000);
    PISAR_LOG_INFO("Starting Calibration...");

    // Calibrate velocity first without caring about position. Then once velocity is calibrated, we calibrated position
    // with already calibrated velocity.

    // Calibrate velocity
    {
        auto calib_data = calibrateImpl(num_batches, slope_sample_batch_size);
        calibration_data.velocity_slope = calib_data.velocity_slope;
        setCalibration(calibration_data);
    }

    // Calibrate position
    {
        auto calib_data = calibrateImpl(num_batches, slope_sample_batch_size);
        calibration_data.position_slope = calib_data.position_slope;
        setCalibration(calibration_data);
    }

    PISAR_LOG_INFO("Kinematic tracker Calibration complete.");
    PISAR_LOG_INFO("Velocity Slope: x=%f, y=%f", calibration_data.velocity_slope.x(), calibration_data.velocity_slope.y());
    PISAR_LOG_INFO("Position Slope: x=%f, y=%f", calibration_data.position_slope.x(), calibration_data.position_slope.y());

    if (save)
    {
        if (!saveCalibrationData())
        {
            PISAR_LOG_ERROR("Failed to save calibration data.");
            return false;
        }

        PISAR_LOG_INFO("Calibration data saved successfully!");
    }

    return true;
}

}