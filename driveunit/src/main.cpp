#include "pisar/driveunit/facility.h"
#include "pisar/driveunit/operating_mode_manager.h"
#include "pisar/driveunit/transport_interface.h"
#include "pisar/driveunit/message_handler.h"

#include <Eigen/Dense>

#include "Arduino.h"

using namespace pisar::driveunit;

MotorDriver left_motor(1, 2);
MotorDriver right_motor(3, 4);

DifferentialDriveController drive_controller(left_motor, right_motor, 10);
Imu imu(SPI1, 3);
RobotFacility facility(drive_controller, imu);


/// @brief Factory function for creating the default mode
OperatingModeIdle createDefaultMode()
{
    return OperatingModeIdle(facility);
}

using OperatingModeManagerT = OperatingModeManager<OperatingModeIdle, OperatingModeFollowTrajectory, OperatingModeRotate>;
OperatingModeManagerT operating_mode_manager(createDefaultMode);

MessageHandler message_handler(facility, operating_mode_manager);
TransportInterface<decltype(message_handler)> transport_interface(SPISlave, message_handler);

void setup()
{
    Serial.begin(115200);
    initLogging(LogLevel::kInfo);

    facility.initialize();
    PISAR_LOG_INFO("Robot facility initialized");

    operating_mode_manager.initialize(5);
    PISAR_LOG_INFO("Operating mode manager initialized");

    transport_interface.initialize(6);
    PISAR_LOG_INFO("Transport interface initialized");
}

void loop()
{
    PISAR_LOG_INFO("Hello World!");
    delay(1000);
}

