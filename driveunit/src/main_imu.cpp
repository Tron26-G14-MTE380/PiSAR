#include "pisar/driveunit/facility.h"

#include "Arduino.h"
#include <LittleFS.h>


using namespace pisar::driveunit;

MotorDriver left_motor(7, 6, 0.2f);
MotorDriver right_motor(8, 9, 0.2f);

DifferentialDriveController drive_controller(left_motor, right_motor, 10);

Imu imu(SPI1, 13, 12, 11, 10, 14, "/calibration_data.bin");

RobotFacility facility(drive_controller, imu);

void setup()
{
    initLogging(115200, LogLevel::kInfo, true);

    if(!LittleFS.begin())
    {
        PISAR_LOG_ERROR("Failed to initialize LittleFS!");
        return;
    }

    if(!facility.initialize(configMAX_PRIORITIES-4, configMAX_PRIORITIES-3))
    {
        PISAR_LOG_ERROR("Failed to initialize robot facility!");
        return;
    }

    PISAR_LOG_INFO("Robot facility initialized");

}

void loop()
{
    PISAR_LOG_INFO("Hello World!");
    delay(2000);
}
