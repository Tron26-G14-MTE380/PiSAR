#include "pisar/driveunit/logging.h"

#include "Arduino.h"

namespace pisar::driveunit {

static void uartLogger(std::string_view msg)
{
    Serial.println(msg.data());
}

Logger g_uart_logger;

void initLogging(LogLevel level)
{
    g_uart_logger.setOutput(uartLogger);
    g_uart_logger.setLogLevel(level);
}

}