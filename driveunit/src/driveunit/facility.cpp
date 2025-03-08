#include "pisar/driveunit/facility.h"

namespace pisar::driveunit {

void RobotFacility::updateKinematicTracker()
{
    using namespace std::chrono_literals;
    std::chrono::microseconds sample_time = 0us;
    std::chrono::microseconds time_stamp = 0us;
    std::array<Imu::Data, 128> imu_data;
    size_t data_samples = 0;

    Lock<Mutex> lock(m_kinematic_tracker_mutex);

    {
        Lock<Mutex> lock(m_imu_mutex);
        sample_time = m_imu.getSampleTime();
        time_stamp = static_cast<std::chrono::microseconds>(micros()); // Get current timestamp
        data_samples = m_imu.readFifo(std::span(imu_data));
    }

    for (int i = 0; i < data_samples; ++i)
    {
        m_kinematic_tracker.onImuReading(
            imu_data[i].accel_data,
            imu_data[i].gyro_data,
            time_stamp - (data_samples - 1 - i) * sample_time
        );
    }
}

}