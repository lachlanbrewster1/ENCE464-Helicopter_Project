// *******************************************************
//
// uartTask.c
//
// Definition of the UART task. This task sends important, real
// time information about the heli righ to serial communications
//
// Author: Lachlan Brewster
// Last modified:  23/08/2019
//
// *******************************************************


// Put std includes before everything else
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

// Custom application includes
#include "priorities.h"
#include "sharedConstants.h"
#include "uartTask.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// FreeRTOS task specific defines
#define UARTTASKSTACKSIZE        128         // Stack size in words
extern xSemaphoreHandle g_pUARTMutex;

// Global, module-specific, non-FreeRTOS defines
extern OperatingData_t g_programStatus;
extern uint32_t g_landedAltitudeADCValue;

//*****************************************************************************
// This task handles the UART communications, sending current info about the
// helirig to serial communications. Sends the following info:
// Heli mode, current altitude %, reference altitude %, PWM duty cycle
//*****************************************************************************
static void
uartTask(void *pvParameters)
{

    portTickType ui16LastTime;
    uint32_t ui32PollDelay = 1000;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount ();

    xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
    UARTprintf("UART task starting.\n");
    xSemaphoreGive(g_pUARTMutex);

    // Loop forever
    while(1) {

        // Calculate the helirig altitude as a percentage
        int8_t percentAltitude = (100 * (g_landedAltitudeADCValue - g_programStatus.currentAltDig)) / HELI_OFFSET_FULL;
        if (percentAltitude > 100)
        {
            percentAltitude = 100;
        }
        else if (percentAltitude < 0)
        {
            percentAltitude = 0;
        }

        // Handling printing the mode enum as a string
        char heliMode[10];
        switch (g_programStatus.mode)
        {
          case idle:
              usnprintf (heliMode, sizeof(heliMode), "idle");
              break;
          case calibrate:
              usnprintf (heliMode, sizeof(heliMode), "calibrate");
              break;
          case landed:
              usnprintf (heliMode, sizeof(heliMode), "landed");
              break;
          case flying:
              usnprintf (heliMode, sizeof(heliMode), "flying");
              break;
          case landing:
              usnprintf (heliMode, sizeof(heliMode), "landing");
              break;
        }

        // Print the helirigh info, guarded by the uart mutex
        xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
        UARTprintf("Current altitude: %d %% \n", percentAltitude);
        UARTprintf("Reference altitude: %d %% \n", g_programStatus.referenceAltPercent);

        if (g_programStatus.mode == flying || g_programStatus.mode == landing)
        {
            UARTprintf("PWM: %d %% \n", g_programStatus.mainMotorPWMDuty);
        } else {
            UARTprintf("PWM: INACTIVE %% \n");
        }

        UARTprintf("Mode: %s\n", heliMode);
        xSemaphoreGive(g_pUARTMutex);

        // Wait for the required amount of time.
        vTaskDelayUntil (&ui16LastTime, ui32PollDelay / portTICK_RATE_MS);

    }
}

//*****************************************************************************
// Initializes the UART task.
//*****************************************************************************
uint32_t
uartTaskInit(void)
{

    // Create the UART task.
    if(xTaskCreate(uartTask, (const portCHAR *)"UART",
                   UARTTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_UART_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    UARTprintf("UART task initialized.\n");

    // Success.
    return(0);

}
