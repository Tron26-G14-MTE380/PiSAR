#include "pisar/driveunit/facility.h"

namespace pisar::driveunit {

void RobotFacility::updateKinematicTracker()
{
    using namespace std::chrono_literals;
    std::chrono::microseconds sample_time = 0us;
    std::chrono::microseconds time_stamp = 0us;
    std::array<Imu::Data, kImuFifoBatchSize> imu_data;
    size_t data_samples = 0;

    Lock<Mutex> lock(m_kinematic_tracker_mutex);

    {
        Lock<Mutex> lock(m_imu_mutex);
        sample_time = m_imu.getSampleTime();
        time_stamp = static_cast<std::chrono::microseconds>(micros()); // Get current timestamp
        data_samples = m_imu.readFifoPaired(std::span(imu_data));
    }

    for (int i = 0; i < data_samples; ++i)
    {
        PISAR_LOG_INFO("Sample %d: Accel[%f, %f, %f] | Gyro[%f, %f, %f] | Timestamp: %lld",
            i,
            imu_data[i].accel_data.values.x(),
            imu_data[i].accel_data.values.y(),
            imu_data[i].accel_data.values.z(),
            imu_data[i].gyro_data.values.x(),
            imu_data[i].gyro_data.values.y(),
            imu_data[i].gyro_data.values.z(),
            time_stamp.count() - (data_samples - 1 - i) * sample_time.count());

        m_kinematic_tracker.onImuReading(
            imu_data[i].accel_data,
            imu_data[i].gyro_data,
            time_stamp - (data_samples - 1 - i) * sample_time
        );
    }
}

}