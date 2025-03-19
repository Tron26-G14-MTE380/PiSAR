#include "pisar/driveunit/facility.h"

#include "Arduino.h"
#include <LittleFS.h>


using namespace pisar::driveunit;

// IMU instance
Imu imu(SPI1, 13, 12, 11, 10, "/calibration_data.bin");

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

    PISAR_LOG_INFO("Calibration successful! Calibration data saved!");

    Imu::CalibrationData saved_calib_data = imu.getCalibration();
    imu.setCalibration({{0, 0, 0}, {0, 0, 0}});

    if (!imu.loadCalibrationData())
    {
        PISAR_LOG_ERROR("Failed to load calibration data.");
        return;
    }

    PISAR_LOG_INFO("Calibration data loaded successfully!");

    auto loaded_calib_data = imu.getCalibration();
    PISAR_LOG_INFO("Accel Offset: x=%i, y=%i, z=%i", loaded_calib_data.accel_offset.x(), loaded_calib_data.accel_offset.y(), loaded_calib_data.accel_offset.z());
    PISAR_LOG_INFO("Gyro Offset: x=%i, y=%i, z=%i", loaded_calib_data.gyro_offset.x(), loaded_calib_data.gyro_offset.y(), loaded_calib_data.gyro_offset.z());

    if (loaded_calib_data.accel_offset != saved_calib_data.accel_offset || loaded_calib_data.gyro_offset != saved_calib_data.gyro_offset)
    {
        PISAR_LOG_ERROR("Loaded calibration data does not match saved data!");
        return;
    }

    PISAR_LOG_INFO("Calibration data saved and loaded successfully!");
}

void loop()
{
}
