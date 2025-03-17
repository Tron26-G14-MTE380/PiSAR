#include "pisar/driveunit/facility.h"

#include "Arduino.h"

using namespace pisar::driveunit;

Imu imu(SPI1, 13, 12, 11, 10);

void setup()
{

    initLogging(115200, LogLevel::kInfo, true);

    if (imu.initialize())
    {
        PISAR_LOG_INFO("IMU initialization SUCCESS!");
    }

    imu.calibrate(10000);
}

void loop()
{
}
