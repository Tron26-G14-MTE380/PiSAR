#include "pisar/driveunit/facility.h"
#include "pisar/driveunit/operating_mode_manager.h"
#include "pisar/driveunit/transport_interface.h"
#include "pisar/driveunit/message_handler.h"
#include "pisar/driveunit/async_serial_uart.h"

#include "pisar/driveunit/rtos/debug.h"

#include <Eigen/Dense>

#include "Arduino.h"

using namespace pisar::driveunit;

MotorDriver left_motor(7, 6);
MotorDriver right_motor(8, 9);

DifferentialDriveController drive_controller(left_motor, right_motor, 10);
Imu imu(SPI1, 3);
RobotFacility facility(drive_controller, imu);


/// @brief Factory function for creating the default mode
inline OperatingModeIdle createDefaultMode()
{
    return OperatingModeIdle(facility);
}

using OperatingModeManagerT = OperatingModeManager<OperatingModeIdle, OperatingModeFollowTrajectory, OperatingModeRotate>;
OperatingModeManagerT operating_mode_manager(createDefaultMode);

MessageHandler message_handler(facility, operating_mode_manager);
TransportInterface<decltype(message_handler)> transport_interface(AsyncSerial2, message_handler);

void setup()
{
    initLogging(115200, LogLevel::kDebug, true);

    AsyncSerial2.setTX(4);
    AsyncSerial2.setRX(5);

    pinMode(25, OUTPUT);
    digitalWrite(25, LOW);

    // initDebugMonitor();

    facility.initialize(configMAX_PRIORITIES-3);
    PISAR_LOG_INFO("Robot facility initialized");

    operating_mode_manager.initialize(configMAX_PRIORITIES-2);
    PISAR_LOG_INFO("Operating mode manager initialized");

    transport_interface.initialize(configMAX_PRIORITIES-1);
    PISAR_LOG_INFO("Transport interface initialized");
}

void loop()
{
    PISAR_LOG_INFO("Hello World!");
    delay(2000);
}

