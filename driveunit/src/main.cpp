#include "pisar/driveunit/facility.h"
#include "pisar/driveunit/operating_mode_manager.h"
#include "pisar/driveunit/transport_interface.h"
#include "pisar/driveunit/message_handler.h"

#include <Eigen/Dense>

#include "Arduino.h"

using namespace pisar::driveunit;

MotorDriver left_motor(6, 7);
MotorDriver right_motor(8, 9);

DifferentialDriveController drive_controller(left_motor, right_motor, 10);
// Imu imu(SPI1, 3);
// RobotFacility facility(drive_controller, imu);


// /// @brief Factory function for creating the default mode
// OperatingModeIdle createDefaultMode()
// {
//     return OperatingModeIdle(facility);
// }

// using OperatingModeManagerT = OperatingModeManager<OperatingModeIdle, OperatingModeFollowTrajectory, OperatingModeRotate>;
// OperatingModeManagerT operating_mode_manager(createDefaultMode);

// MessageHandler message_handler(facility, operating_mode_manager);
// TransportInterface<decltype(message_handler)> transport_interface(SPISlave, message_handler);

constexpr int kLeftMotorControlPin = 16;
constexpr int kRightMotorControlPin = 19;

void setup()
{
    Serial.begin(115200);
    initLogging(LogLevel::kInfo);

    pinMode(kLeftMotorControlPin, INPUT);
    pinMode(kRightMotorControlPin, INPUT);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);

    // SPISlave.setRX(16);
    // SPISlave.setCS(17);
    // SPISlave.setSCK(18);
    // SPISlave.setTX(19);

    drive_controller.initialize();
    // facility.initialize();
    // PISAR_LOG_INFO("Robot facility initialized");

    // operating_mode_manager.initialize(5);
    // PISAR_LOG_INFO("Operating mode manager initialized");

    // transport_interface.initialize(6);
    // PISAR_LOG_INFO("Transport interface initialized");
}

void loop()
{
    const bool left_on = digitalRead(kLeftMotorControlPin);
    const bool right_on = digitalRead(kRightMotorControlPin);
    const char* left_status = (left_on ? "on" : "off");
    const char* right_status = (right_on ? "on" : "off");

    //drive_controller.tankDrive(left_on, right_on);
    PISAR_LOG_INFO("Left: %s, Right: %s", left_status, right_status);
    //drive_controller.update();
    delay(10);
}

