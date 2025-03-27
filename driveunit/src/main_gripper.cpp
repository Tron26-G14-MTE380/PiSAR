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

// Servo Driver Instance
GripperController gripper_controller(15, 0, 180);

RobotFacility facility(drive_controller, kinematic_tracker, gripper_controller);

/// @brief Factory function for creating the default mode
inline OperatingModeIdle createDefaultMode()
{
    return OperatingModeIdle(facility);
}

using OperatingModeManagerT = OperatingModeManager<OperatingModeIdle, OperatingModeSetGripper, OperatingModeHardStop, OperatingModeFollowTrajectory, OperatingModeGoToTarget, OperatingModeRotate>;
OperatingModeManagerT operating_mode_manager(createDefaultMode);

void pisarSetup()
{
    initLogging(115200, LogLevel::kInfo, false);

    if (!gripper_controller.initialize())
    {
        PISAR_LOG_ERROR("Failed to initialize gripper controller!");
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

    PISAR_LOG_INFO("Initiating gripper test in 3 seconds...");
    delay(3000);
    PISAR_LOG_INFO("Starting....");
}

void pisarLoop()
{
    operating_mode_manager.switchMode(OperatingModeSetGripper(facility, true));
    delay(3000);
    operating_mode_manager.switchMode(OperatingModeSetGripper(facility, false));
    delay(3000);
}