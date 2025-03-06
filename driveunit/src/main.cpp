#include "pisar/driveunit/facility.h"

#include "driveunit_interface/codec.h"
#include "driveunit_interface/interface.h"

#include <Eigen/Dense>

#include "Arduino.h"

using namespace pisar::driveunit;

MotorDriver left_motor(1, 2);
MotorDriver right_motor(3, 4);

DifferentialDriveController drive_controller(left_motor, right_motor, 10);
Imu imu(SPI1, 3);
ImuPlanarKinematicTracker<512> kinematic_tracker(imu.getSampleTime());

RobotFacility facility(drive_controller, imu, kinematic_tracker);

void setup()
{

}

void loop()
{

}

