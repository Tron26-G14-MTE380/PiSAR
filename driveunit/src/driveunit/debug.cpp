#include "pisar/driveunit/logging.h"

#include "FreeRTOS.h"

#include <Arduino.h>

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    digitalWrite(25, HIGH);
    PISAR_LOG_ERROR("Stack overflow in task: %s\n", pcTaskName);
    while (true) {}
}

extern "C" void vApplicationMallocFailedHook()
{
    digitalWrite(25, HIGH);
    PISAR_LOG_ERROR("Memory allocation failed!\n");
    while (true) {}  // Halt to debug
}

extern "C" void HardFault_Handler()
{
    digitalWrite(25, HIGH);
    PISAR_LOG_ERROR("!!! HARD FAULT DETECTED !!!\n");
    while (true) {}  // Halt to debug
}

namespace pisar::driveunit {

void printTaskInfo()
{
    char taskStats[512];
    vTaskList(taskStats);
    PISAR_LOG_DEBUG("Task Info:\n%s\n", taskStats);
}

void checkStackUsage()
{
    UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
    PISAR_LOG_DEBUG("Task %s stack high watermark: %u bytes\n", pcTaskGetName(NULL), watermark);
}


void debugMonitor(void* params)
{
    while (true)
    {
        printTaskInfo();
        vTaskDelay(pdMS_TO_TICKS(5000));  // Wait 5 seconds
    }
}

void initDebugMonitor()
{
    xTaskCreate(debugMonitor, "Monitor", 1024, NULL, 1, NULL);
}

}