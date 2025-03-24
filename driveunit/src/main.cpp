#include "Arduino.h"

#include <FreeRTOS.h>
#include <task.h>


#ifndef PISAR_DRIVEUNIT_MAIN_TASK_STACK_SIZE
#define PISAR_DRIVEUNIT_MAIN_TASK_STACK_SIZE 4096
#endif

#ifndef PISAR_DRIVEUNIT_MAIN_TASK_PRIORITY
#define PISAR_DRIVEUNIT_MAIN_TASK_PRIORITY 4
#endif

void pisarSetup();

extern void __attribute__((weak)) pisarLoop();

void pisarMain(void*)
{
    pisarSetup();

    if (pisarLoop)
    {
        // Someone implemented the loop, call it
        while (true)
        {
            pisarLoop();
        }
    }
}

void setup()
{
    if (xTaskCreate(pisarMain, "pisar_main", PISAR_DRIVEUNIT_MAIN_TASK_STACK_SIZE, nullptr, PISAR_DRIVEUNIT_MAIN_TASK_PRIORITY, nullptr) != pdPASS)
    {
        return;
    }
}

void loop()
{
    vTaskDelete(NULL);
}
