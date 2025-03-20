#include "pisar/driveunit/facility.h"

#include "Arduino.h"
#include <LittleFS.h>


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


    if (!imu.initialize())
    {
        PISAR_LOG_ERROR("Failed to initialize IMU!");
        return;
    }

    // First, calibrate and save data
    if (!imu.calibrate(10000, true)) // Simulate calibration
    {
        PISAR_LOG_ERROR("Calibration failed!");
        return;
    }

}

void loop()
{
    auto accelData = imu.readAccel();
    PISAR_LOG_INFO("Accel Data: x = %f, y = %f, z = %f",
                   accelData.values.x(), accelData.values.y(), accelData.values.z()); 
    
    auto gyroData = imu.readGyro();
    PISAR_LOG_INFO("Gyro Data: x = %f, y = %f, z = %f",
        gyroData.values.x(), gyroData.values.y(), gyroData.values.z());

    delay(250);
}
