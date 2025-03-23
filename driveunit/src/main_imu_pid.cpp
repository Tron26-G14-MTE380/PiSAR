#include "pisar/driveunit/facility.h"
#include "pisar/driveunit/operating_mode_manager.h"
#include "pisar/driveunit/operating_mode.h"

#include "Arduino.h"
#include <LittleFS.h>
#include <Plotter.h>

using namespace pisar::driveunit;

MotorDriver left_motor(7, 6, 0.06f, 0.2f);
MotorDriver right_motor(8, 9, 0.06f, 0.2f);

DifferentialDriveController drive_controller(left_motor, right_motor, 10, 1.0f, 4.0f);

Imu imu(SPI1, 13, 12, 11, 10, 14, "/calibration_data.bin");

RobotFacility facility(drive_controller, imu);

/// @brief Factory function for creating the default mode
inline OperatingModeIdle createDefaultMode()
{
    return OperatingModeIdle(facility);
}

using OperatingModeManagerT = OperatingModeManager<OperatingModeIdle, OperatingModeFollowTrajectory, OperatingModeRotate>;
OperatingModeManagerT operating_mode_manager(createDefaultMode);

Plotter plotter;

// Data variables for visualization
float accel_x = 0, accel_y = 0;
float gyro_z = 0;
float velocity_x = 0, velocity_y = 0;
float position_x = 0, position_y = 0;
float orientation = 0;


void setup()
{
    initLogging(115200, LogLevel::kInfo, false);

    //plotter.AddTimeGraph("Acceleration", 500, "Accel X", accel_x, "Accel Y", accel_y);
    //plotter.AddTimeGraph("Velocity", 500, "Vel X", velocity_x, "Vel Y", velocity_y);
    //plotter.AddTimeGraph("Position", 500, "Pos X", position_x, "Pos Y", position_y);
    plotter.AddTimeGraph("Gyroscope", 500, "Gyro Z", gyro_z);
    plotter.AddTimeGraph("Orientation", 500, "Yaw", orientation);

    if (!LittleFS.begin())
    {
        PISAR_LOG_ERROR("Failed to initialize LittleFS!");
        return;
    }

    if(!facility.initialize(configMAX_PRIORITIES-2, configMAX_PRIORITIES-2))
    {
        PISAR_LOG_ERROR("Failed to initialize robot facility!");
        return;
    }

    auto calib_data = imu.getCalibration();
    PISAR_LOG_INFO("Accel Offset: x=%i, y=%i, z=%i", calib_data.accel_offset.x(), calib_data.accel_offset.y(), calib_data.accel_offset.z());
    PISAR_LOG_INFO("Gyro Offset: x=%i, y=%i, z=%i", calib_data.gyro_offset.x(), calib_data.gyro_offset.y(), calib_data.gyro_offset.z());

    operating_mode_manager.initialize(configMAX_PRIORITIES-3);

    PISAR_LOG_INFO("Initiating rotate test in 3 seconds...");
    delay(5000);
    PISAR_LOG_INFO("Starting....");
    operating_mode_manager.switchMode(OperatingModeRotate(facility, 90.0f));
}

void loop()
{
    // // accel_x = facility.getAcceleration().x();
    // // accel_y = facility.getAcceleration().y();

    // gyro_z = facility.getAngularVelocity();

    // // velocity_x = facility.getVelocity().x();
    // // velocity_y = facility.getVelocity().y();

    // // position_x = facility.getPosition().x();
    // // position_y = facility.getPosition().y();

    // orientation = facility.getOrientation(); // Yaw in radians

    // // // Plot the data
    // plotter.Plot();

}