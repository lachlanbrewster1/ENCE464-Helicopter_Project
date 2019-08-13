#include <stdint.h>
#include <stdbool.h>

#include <priorities.h>

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

// #include HELI INFO // TODO


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

// TEMPORARY // TODO
//#define PRIORITY_UART_TASK       3
//#define BLOCK_TIME_MAX          1



// FreeRTOS structures.
extern xSemaphoreHandle g_pUARTMutex;
extern xQueueHandle g_adcReadQueue;



//*****************************************************************************
// This task handles the UART communications, sending current info about the helirig
//*****************************************************************************
static void
uartTask(void *pvParameters)
{

    xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
    char string[100];  // 100 characters across the display
    usnprintf (string, sizeof(string), "UART task starting.\r\n");
    UARTSend(string);
    xSemaphoreGive(g_pUARTMutex);


    //
    // Loop forever
    while(1) {

        xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
        char string[100];  // 100 characters across the display
        usnprintf (string, sizeof(string), "HELI INFO.\r\n");
        UARTSend(string);
        xSemaphoreGive(g_pUARTMutex);


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
    initialiseUSB_UART();

    //
    // Create the UART task.
    if(xTaskCreate(uartTask, (const portCHAR *)"UART",
                   UARTTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_UART_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    return(0);


}







