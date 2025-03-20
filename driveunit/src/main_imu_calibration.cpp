#include "pisar/driveunit/facility.h"

#include "Arduino.h"
#include <LittleFS.h>
#include <Plotter.h>


using namespace pisar::driveunit;

// IMU instance
Imu imu(SPI1, 13, 12, 11, 10, 14, "/calibration_data.bin");

Plotter plotter;
float accel_x = 0, accel_y = 0, accel_z = 0;
float gyro_x = 0, gyro_y = 0, gyro_z = 0;

void setup()
{
    initLogging(115200, LogLevel::kInfo, true);
    plotter.Begin();

    plotter.AddTimeGraph("Accel Data", 500, "Accel x", accel_x, "Accel y", accel_y, "Accel z", accel_z);
    plotter.AddTimeGraph("Gyro Data", 500, "Gyro x", gyro_x, "Gyro y", gyro_y, "Gyro z", gyro_z);

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
    auto accel_data = imu.readAccel();
    accel_x = accel_data.values.x();
    accel_y = accel_data.values.y();
    accel_z = accel_data.values.z();

    auto gyro_data = imu.readGyro();
    gyro_x = gyro_data.values.x();
    gyro_y = gyro_data.values.y();
    gyro_z = gyro_data.values.z();

    plotter.Plot();
}
