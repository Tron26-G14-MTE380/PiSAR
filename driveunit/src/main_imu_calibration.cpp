#include "pisar/driveunit/facility.h"

#include "Arduino.h"
#include <LittleFS.h>
#include <Plotter.h>

using namespace pisar::driveunit;

// IMU instance
Imu imu(SPI1, 13, 12, 11, 10, 14, "/imu_calibration_data.bin");
ImuPlanarKinematicTracker kinematic_tracker(imu, "/kinematic_tracker_calibration_data.bin");

static constexpr bool kDoCalibration = false;
static constexpr auto kTrackerResetTime = std::chrono::milliseconds(5000);

Plotter plotter;
float accel_x = 0, accel_y = 0;
float gyro_z = 0;

float velocity_x = 0, velocity_y = 0;
float position_x = 0, position_y = 0;
float orientation = 0;

auto last_reset_time = std::chrono::milliseconds(0);

void setup()
{
    initLogging(115200, LogLevel::kInfo, true);

    if(!LittleFS.begin())
    {
        PISAR_LOG_ERROR("Failed to initialize LittleFS!");
        return;
    }

    if (!imu.initialize())
    {
        PISAR_LOG_ERROR("Failed to initialize IMU!");
        return;
    }
    
    // First, calibrate and save data
    if (!imu.calibrate(1000, true)) // Simulate calibration
    {
        PISAR_LOG_ERROR("IMU Calibration failed!");
        return;
    }
    
    PISAR_LOG_INFO("IMU Calibration done!");

    if (!kinematic_tracker.initialize(5))
    {
        PISAR_LOG_ERROR("Failed to initialize kinematic tracker!");
        return;
    }
    
    if (!kinematic_tracker.calibrate(3, 1000, true)) // Simulate calibration
    {
        PISAR_LOG_ERROR("Kinematic tracker calibration failed!");
        return;
    }

    PISAR_LOG_INFO("Kinematic tracker calibration done!");

    plotter.Begin();


    plotter.AddTimeGraph("Accel Data", 500, "Accel x", accel_x, "Accel y", accel_y);
    plotter.AddTimeGraph("Velocity", 1000, "Vel X", velocity_x, "Vel Y", velocity_y);
    plotter.AddTimeGraph("Position", 1500, "Pos X", position_x, "Pos Y", position_y);

    plotter.AddTimeGraph("Gyro Data", 1000, "Gyro z", gyro_z);
    plotter.AddTimeGraph("Orientation", 1500, "Yaw", orientation);

}


void loop()
{
    auto now = std::chrono::milliseconds(millis());
    if (now - last_reset_time > kTrackerResetTime)
    {
        kinematic_tracker.reset();
        last_reset_time = now;
    }


    const auto current_state = kinematic_tracker.getState();

    accel_x = current_state.acceleration.x();
    accel_y = current_state.acceleration.y();

    velocity_x = current_state.velocity.x();
    velocity_y = current_state.velocity.y();

    position_x = current_state.pose.position.x();
    position_y = current_state.pose.position.y();

    gyro_z = current_state.angular_velocity;

    orientation = current_state.pose.orientation;

    plotter.Plot();
}
