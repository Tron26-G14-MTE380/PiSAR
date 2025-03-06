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
OperatingModeManagerT operating_mode_manager(createDefaultMode, 10);

MessageHandler message_handler(facility, operating_mode_manager);
TransportInterface<256, 256, decltype(message_handler)> transport_interface(SPISlave, message_handler, 10);


void setup()
{

}

void loop()
{

}

