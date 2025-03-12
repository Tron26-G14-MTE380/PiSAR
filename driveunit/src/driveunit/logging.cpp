#include "pisar/driveunit/logging.h"

#include "Arduino.h"

namespace pisar::driveunit {

#ifdef PISAR_DRIVEUNIT_LOGGING_ENABLED

Logger g_uart_logger;

static void uartLogger(std::string_view msg)
{
    Serial.println(msg.data());
}

void waitForSerialConnection()
{
    while(!Serial.dtr()) {}
    delay(500); // Give some time to allow computer drivers to settle
}

void initLogging(unsigned int baud, LogLevel level, bool wait_for_connection)
{
    Serial.begin(baud);
    g_uart_logger.setOutput(uartLogger);
    g_uart_logger.setLogLevel(level);

    if (wait_for_connection)
    {
        waitForSerialConnection();
    }
}

#endif

}