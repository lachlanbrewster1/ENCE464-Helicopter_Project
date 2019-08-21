#include <stdint.h>
#include <stdbool.h>

#include <priorities.h>
#include "priorities.h"


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
#include "circBufT.h"
#include "uartTask.h"

#include "sharedConstants.h"

#include "stdio.h"
#include "stdlib.h"

#include "utils/uart.h"


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"



//*****************************************************************************
//
// The stack size for the UART task.
//
//*****************************************************************************
#define UARTTASKSTACKSIZE        128         // Stack size in words

extern OperatingData_t g_programStatus;

// FreeRTOS structures.
extern xSemaphoreHandle g_pUARTMutex;

// Globals
extern uint32_t g_landedAltitudeADCValue;


//*****************************************************************************
// This task handles the UART communications, sending current info about the helirig
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

    //                xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
    //                char string[31];
    //                usnprintf (string, sizeof(string), "ADC value: %d\r\n", ulValue);
    //                UARTprintf(string);
    //                xSemaphoreGive(g_pUARTMutex);

    //
    // Loop forever
    while(1) {

        int8_t percentAltitude = (100 * (g_landedAltitudeADCValue - g_programStatus.currentAltDig)) / HELI_OFFSET_FULL;
        if (percentAltitude > 100) {
            percentAltitude = 100;
        } else if (percentAltitude < 0) {
            percentAltitude = 0;
        }

        char heliMode[10];
        switch (g_programStatus.mode) {
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

        xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
        UARTprintf("Current altitude: %d %% \n", percentAltitude);
        UARTprintf("Reference altitude: %d %% \n", g_programStatus.referenceAltPercent);

        if (g_programStatus.mode == flying || g_programStatus.mode == landing) {
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
    //
    // Initialize UART things
    //initialiseUSB_UART();

    //
    // Create the UART task.
    if(xTaskCreate(uartTask, (const portCHAR *)"UART",
                   UARTTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_UART_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    UARTprintf("UART task initialized.\n");

    //
    // Success.
    return(0);


}








