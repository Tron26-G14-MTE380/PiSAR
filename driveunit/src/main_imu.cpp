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
}

void loop()
{
    const auto penisjibblets = imu.readAccelRaw();
    PISAR_LOG_ERROR("x = %.2f, y = %.2f, z = %.2f", penisjibblets.values.x(), penisjibblets.values.y(), penisjibblets.values.z());
    // skibidi pa pa
}