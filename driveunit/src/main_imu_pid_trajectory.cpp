#include "pisar/driveunit/facility.h"
#include "pisar/driveunit/operating_mode_manager.h"
#include "pisar/driveunit/operating_mode.h"

#include "Arduino.h"
#include <LittleFS.h>
#include <Plotter.h>
#include "test_data.h"

using namespace pisar::driveunit;

////////////////////////////////////////////////////TESTING////////////////////////////////////////////////////////

size_t currentSampleIndex = 0;
float nowOffset = 0;
bool first_time = true;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// IMU Instance
Imu imu(SPI1, 13, 12, 11, 10, 14, "/imu_calibration_data.bin");
ImuPlanarKinematicTracker kinematic_tracker(imu, "/kinematic_tracker_calibration_data.bin");

// Motor Driver Instance
MotorDriver left_motor(7, 6, 0.06f, 0.2f);
MotorDriver right_motor(8, 9, 0.06f, 0.2f);
DifferentialDriveController drive_controller(left_motor, right_motor, 10, 1.0f, 4.0f);

static constexpr bool kDoCalibration = false;
static constexpr auto kTrackerResetTime = std::chrono::milliseconds(5000);

RobotFacility facility(drive_controller, kinematic_tracker);

/// @brief Factory function for creating the default mode
inline OperatingModeIdle createDefaultMode()
{
    return OperatingModeIdle(facility);
}

using OperatingModeManagerT = OperatingModeManager<OperatingModeIdle, OperatingModeFollowTrajectory, OperatingModeRotate>;
OperatingModeManagerT operating_mode_manager(createDefaultMode);

// Visualization (verify every time you make changes... Gabe)
// Plotter plotter;
// float accel_x = 0, accel_y = 0;
// float gyro_z = 0;
// float velocity_x = 0, velocity_y = 0;
// float position_x = 0, position_y = 0;
// float orientation = 0;

// IMU zeroing still not perfect, force after a certain amount of time
auto last_reset_time = std::chrono::milliseconds(1500);

void pisarSetup()
{
    initLogging(115200, LogLevel::kInfo, true);

    // plotter.AddTimeGraph("Acceleration", 500, "Accel X", accel_x, "Accel Y", accel_y);
    // plotter.AddTimeGraph("Velocity", 500, "Vel X", velocity_x, "Vel Y", velocity_y);
    // plotter.AddTimeGraph("Position", 500, "Pos X", position_x, "Pos Y", position_y);
    // plotter.AddTimeGraph("Gyroscope", 500, "Gyro Z", gyro_z);
    // plotter.AddTimeGraph("Orientation", 500, "Yaw", orientation);

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

    // if (!imu.calibrate(5000, true))
    // {
    //     PISAR_LOG_ERROR("IMU Calibration failed!");
    //     return;
    // }

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

    // if (!kinematic_tracker.calibrate(3, 1000, true))
    // {
    //     PISAR_LOG_ERROR("Kinematic tracker calibration failed!");
    //     return;
    // }

    if (!facility.initialize())
    {
        PISAR_LOG_ERROR("Failed to initialize facility!");
        return;
    }

    // We do all of this in respective calibrate functions, maybe move? ELEPHANT
    // auto calib_data = imu.getCalibration();
    // auto slope_data = kinematic_tracker.getCalibration();
    // PISAR_LOG_INFO("Accel Offset: x=%i, y=%i, z=%i", calib_data.accel_offset.x(), calib_data.accel_offset.y(), calib_data.accel_offset.z());
    // PISAR_LOG_INFO("Gyro Offset: x=%i, y=%i, z=%i", calib_data.gyro_offset.x(), calib_data.gyro_offset.y(), calib_data.gyro_offset.z());
    // PISAR_LOG_INFO("Velocity Slope: x=%f, y=%f", slope_data.velocity_slope().x(), slope_data.velocity_slope().y());
    // PISAR_LOG_INFO("Position Slope: x=%f, y=%f", slope_data.position_slope().x(), slope_data.position_slope().y());

    operating_mode_manager.initialize(configMAX_PRIORITIES-3);

    // PISAR_LOG_INFO("Initiating drive test in 3 seconds...");
    // delay(3000);
    PISAR_LOG_INFO("Starting....");


    // Test data ELEPHANT
    // for(int i = 0; i < kTestData.size(); i++)
    // {
    //     ParsedData data = kTestData[i];
    //     PISAR_LOG_INFO("Timestamp: %f", data.timestamp);
    //     for(int j = 0; j < data.trajectory.size(); j++)
    //     {
    //         PISAR_LOG_INFO("Vector %d: %f, %f", j, data.trajectory[j].x(), data.trajectory[j].y());
    //     }
    // }

    std::array<Eigen::Vector2f, 1> penis1 = {Eigen::Vector2f(0.04, 0.0)};    
    // std::array<Eigen::Vector2f, 1> penis2 = {Eigen::Vector2f(0.08, 0.06)};
    // std::array<Eigen::Vector2f, 1> penis2 = {Eigen::Vector2f(0.08, -0.06)};
    operating_mode_manager.switchMode(OperatingModeFollowTrajectory(facility, penis1));
}

void pisarLoop()
{

    if (first_time) 
    {
        PISAR_LOG_INFO("STARTING");
        nowOffset = static_cast<float>(micros()) / 1000000.0f;
        first_time = false;
    }

    // Test data refresh
    if (currentSampleIndex < kTestData.size())
    { 
        auto now = (static_cast<float>(micros()) / 1000000.0f) - nowOffset;

        const auto& data = kTestData[currentSampleIndex];

        if(now > data.timestamp)
        {
            // Debug ELEPHANT
            // PISAR_LOG_INFO("Timestamp: %f, Now: %f", data.timestamp, now);
            operating_mode_manager.switchMode(OperatingModeFollowTrajectory(facility, data.trajectory));
            currentSampleIndex++;
        }
    }

    // auto now = std::chrono::milliseconds(millis());
    // if (now - last_reset_time > kTrackerResetTime)
    // {
    //     kinematic_tracker.reset();
    //     last_reset_time = now;
    // }

    // const auto current_state = kinematic_tracker.getState();

    // accel_x = current_state.acceleration.x();
    // accel_y = current_state.acceleration.y();

    // velocity_x = current_state.velocity.x();
    // velocity_y = current_state.velocity.y();

    // position_x = current_state.pose.position.x();
    // position_y = current_state.pose.position.y();

    // gyro_z = current_state.angular_velocity;

    // orientation = current_state.pose.orientation;

    // plotter.Plot();
}