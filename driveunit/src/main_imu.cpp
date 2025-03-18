#include "pisar/driveunit/facility.h"

#include "Arduino.h"
#include <LittleFS.h>


using namespace pisar::driveunit;

// IMU instance
Imu imu(SPI1, 13, 12, 11, 10);

void setup()
{
    initLogging(115200, LogLevel::kInfo, true);

    if(!LittleFS.begin())
    {
        PISAR_LOG_ERROR("Failed to initialize LittleFS!");
    }


    imu.initialize();

    // First, calibrate and save data
    imu.calibrate(1000); // Simulate calibration
    imu.saveCalibrationData();
    imu.loadCalibrationData();
}

void loop()
{
}
