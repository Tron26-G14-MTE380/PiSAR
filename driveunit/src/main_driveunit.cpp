#include "pisar/driveunit/facility.h"
#include "pisar/driveunit/operating_mode_manager.h"
#include "pisar/driveunit/transport_interface.h"
#include "pisar/driveunit/message_handler.h"
#include "pisar/driveunit/async_serial_uart.h"

#include "pisar/driveunit/rtos/debug.h"

#include <Eigen/Dense>

#include "Arduino.h"

using namespace pisar::driveunit;

Imu imu(SPI1, 13, 12, 11, 10, 14, "/imu_calibration_data.bin");
ImuPlanarKinematicTracker kinematic_tracker(imu, "/kinematic_tracker_calibration_data.bin");

MotorDriver left_motor(7, 6, 0.06f, 0.1f);
MotorDriver right_motor(8, 9, 0.06f, 0.1f);
DifferentialDriveController drive_controller(left_motor, right_motor, 10, 1.0f, 4.0f);

GripperController gripper_controller(15);

RobotFacility facility(drive_controller, kinematic_tracker, gripper_controller);

/// @brief Factory function for creating the default mode
inline OperatingModeIdle createDefaultMode()
{
    return OperatingModeIdle(facility);
}

using OperatingModeManagerT = OperatingModeManager<OperatingModeIdle, OperatingModeHardStop, OperatingModeFollowTrajectory, OperatingModeGoToTarget, OperatingModeRotate>;
OperatingModeManagerT operating_mode_manager(createDefaultMode);

MessageHandler message_handler(facility, operating_mode_manager);
TransportInterface<decltype(message_handler)> transport_interface(AsyncSerial2, message_handler);

void pisarSetup()
{
    initLogging(115200, LogLevel::kInfo, false);

    AsyncSerial2.setTX(4);
    AsyncSerial2.setRX(5);

    if (!LittleFS.begin())
    {
        PISAR_LOG_ERROR("Failed to initialize LittleFS!");
        return;
    }
    PISAR_LOG_INFO("Little FS Initialized");

    if (!imu.initialize())
    {
        PISAR_LOG_ERROR("Failed to initialize IMU!");
        return;
    }
    PISAR_LOG_INFO("Imu Initialized");

    imu.setCalibration({ .accel_offset = {-96, -95, 8351}, .gyro_offset = {15, -1, -7}});


    // auto calib_data = imu.getCalibration();
    // PISAR_LOG_INFO("Accel Offset: x=%i, y=%i, z=%i", calib_data.accel_offset.x(), calib_data.accel_offset.y(), calib_data.accel_offset.z());
    // PISAR_LOG_INFO("Gyro Offset: x=%i, y=%i, z=%i", calib_data.gyro_offset.x(), calib_data.gyro_offset.y(), calib_data.gyro_offset.z());

    if (!kinematic_tracker.initialize(configMAX_PRIORITIES-2))
    {
        PISAR_LOG_ERROR("Failed to initialize kinematic tracker!");
        return;
    }
    PISAR_LOG_INFO("Kinematic tracker initialized");

    kinematic_tracker.setCalibration({.velocity_slope = {0.000280f, -0.001267f}, .position_slope = {-0.000439f, -0.000288f}});

    if (!drive_controller.initialize(configMAX_PRIORITIES-2))
    {
        PISAR_LOG_ERROR("Failed to initialize drive controller!");
        return;
    }
    PISAR_LOG_INFO("Drive controller initialized");

    if (!gripper_controller.initialize())
    {
        PISAR_LOG_ERROR("Failed to initialize gripper controller!");
        return;
    }    
    PISAR_LOG_INFO("Gripper Controller initialized");
    
    if (!facility.initialize())
    {
        PISAR_LOG_ERROR("Failed to initialize facility!");
        return;
    }
    PISAR_LOG_INFO("Facility initialized");

 
    if (!operating_mode_manager.initialize(configMAX_PRIORITIES-3))
    {
        PISAR_LOG_ERROR("Failed to initialize operating mode manager!");
        return;
    }
    PISAR_LOG_INFO("Operating mode manager initialized");

    if (!transport_interface.initialize(configMAX_PRIORITIES-1))
    {
        PISAR_LOG_ERROR("Failed to initialize transport interface!");
        return;
    }
    PISAR_LOG_INFO("Transport interface initialized");
}

void pisarLoop()
{
    //PISAR_LOG_INFO("Hello World!");
    delay(3000);
}

