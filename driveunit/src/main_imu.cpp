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
        return;
    }

}

void loop()
{
    auto accelData = imu.readAccel();
    PISAR_LOG_INFO("Accel Data: x = %i, y = %i, z = %i",
                   accelData.values.x(), accelData.values.y(), accelData.values.z());   
}