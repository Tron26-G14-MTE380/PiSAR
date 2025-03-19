#include "pisar/driveunit/facility.h"
#include "pisar/driveunit/imu.h"

#include "Arduino.h"
#include <LittleFS.h>
#include <variant>
#include <span>
#include <vector>

using namespace pisar::driveunit;

// IMU instance
Imu imu(SPI1, 13, 12, 11, 10, 14, "/calibration_data.bin");

void setup()
{
    initLogging(115200, LogLevel::kInfo, true);

    if(!LittleFS.begin())
    {
        PISAR_LOG_ERROR("Failed to initialize LittleFS!");
        return;
    }

    imu.calibrate(10000, true);

    if (!imu.initialize())
    {
        PISAR_LOG_ERROR("Failed to initialize IMU!");
        return;
    }

}

void loop()
{

    auto accelData = imu.readAccel();
    PISAR_LOG_INFO("Accel Data READ: x = %i, y = %i, z = %i",
                   accelData.values.x(), accelData.values.y(), accelData.values.z()); 
    
    auto gyroData = imu.readGyro();
    PISAR_LOG_INFO("Gyro Data READ: x = %i, y = %i, z = %i",
        gyroData.values.x(), gyroData.values.y(), gyroData.values.z());


    constexpr size_t kNumSamples = 10; // Adjust based on available FIFO samples

    std::array<std::variant<Imu::AccelData, Imu::GyroData>, kNumSamples> fifo_scaled;
    size_t samples_read = imu.readFifo(std::span(fifo_scaled));

    PISAR_LOG_INFO("Read %d scaled FIFO samples:", samples_read);

    for (size_t i = 0; i < samples_read; ++i)
    {
        std::visit([](auto&& sample) {
            using T = std::decay_t<decltype(sample)>;
            if constexpr (std::is_same_v<T, Imu::AccelData>)
            {
                PISAR_LOG_INFO("Accel: x=%.3f y=%.3f z=%.3f",
                    sample.values.x(), sample.values.y(), sample.values.z());
            }
            else if constexpr (std::is_same_v<T, Imu::GyroData>)
            {
                PISAR_LOG_INFO("Gyro: x=%.3f y=%.3f z=%.3f",
                    sample.values.x(), sample.values.y(), sample.values.z());
            }
        }, fifo_scaled[i]);
    }

    delay(10000); // Wait before reading again
}