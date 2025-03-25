#include "pisar/driveunit/facility.h"
#include "pisar/driveunit/operating_mode_manager.h"
#include "pisar/driveunit/operating_mode.h"

#include "Arduino.h"
#include <LittleFS.h>
#include <Plotter.h>

using namespace pisar::driveunit;

// IMU Instance
Imu imu(SPI1, 13, 12, 11, 10, 14, "/imu_calibration_data.bin");
ImuPlanarKinematicTracker kinematic_tracker(imu, "/kinematic_tracker_calibration_data.bin");

MotorDriver left_motor(7, 6, 0.06f, 0.2f);
MotorDriver right_motor(8, 9, 0.06f, 0.2f);
DifferentialDriveController drive_controller(left_motor, right_motor, 10, 1.0f, 4.0f);

RobotFacility facility(drive_controller, kinematic_tracker);

/// @brief Factory function for creating the default mode
inline OperatingModeIdle createDefaultMode()
{
    return OperatingModeIdle(facility);
}

using OperatingModeManagerT = OperatingModeManager<OperatingModeIdle, OperatingModeFollowTrajectory, OperatingModeRotate>;
OperatingModeManagerT operating_mode_manager(createDefaultMode);

// Plotter plotter;

// // Data variables for visualization
// float accel_x = 0, accel_y = 0;
// float gyro_z = 0;
// float velocity_x = 0, velocity_y = 0;
// float position_x = 0, position_y = 0;
// float orientation = 0;


void pisarSetup()
{
    initLogging(115200, LogLevel::kInfo, false);

    if (!LittleFS.begin())
    {
        PISAR_LOG_ERROR("Failed to initialize LittleFS!");
        return;
    }

    if (!imu.initialize())
    {
        PISAR_LOG_ERROR("Failed to initialize IMU!");
        return;
    }

    if (!kinematic_tracker.initialize(5))
    {
        PISAR_LOG_ERROR("Failed to initialize kinematic tracker!");
        return;
    }

    if (!drive_controller.initialize(6))
    {
        PISAR_LOG_ERROR("Failed to initialize drive controller!");
        return;
    }

    if (!facility.initialize())
    {
        PISAR_LOG_ERROR("Failed to initialize facility!");
        return;
    }

    // auto calib_data = imu.getCalibration();
    // PISAR_LOG_INFO("Accel Offset: x=%i, y=%i, z=%i", calib_data.accel_offset.x(), calib_data.accel_offset.y(), calib_data.accel_offset.z());
    // PISAR_LOG_INFO("Gyro Offset: x=%i, y=%i, z=%i", calib_data.gyro_offset.x(), calib_data.gyro_offset.y(), calib_data.gyro_offset.z());

    operating_mode_manager.initialize(configMAX_PRIORITIES-3);

    PISAR_LOG_INFO("Initiating rotate test in 3 seconds...");
    delay(3000);
    PISAR_LOG_INFO("Starting....");
    operating_mode_manager.switchMode(OperatingModeRotate(facility, 90.0f));
}

void pisarLoop()
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