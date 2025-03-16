#include "pisar/driveunit/facility.h"

#include "Arduino.h"

using namespace pisar::driveunit;

Imu imu(SPI1, 13, 12, 11, 10);

void setup()
{

    initLogging(115200, LogLevel::kInfo, true);

    if (!imu.initialize())
    {
        PISAR_LOG_ERROR("IMU initialization FAILED!");
        return;
    }
    PISAR_LOG_INFO("IMU initialization SUCCESS!");
}

void loop()
{
    // skibidi pa pa
}