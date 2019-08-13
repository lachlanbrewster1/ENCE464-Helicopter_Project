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
#include "adcTriggerTask.h"

#include "stdio.h"
#include "stdlib.h"


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

static circBuf_t g_inBuffer;     // Buffer of size BUF_SIZE
static uint16_t landed_ref;      // Landed reference of the helicopter



//*****************************************************************************
//
// The stack size for the ADC Queue task.
//
//*****************************************************************************
#define ADCTASKSTACKSIZE        128         // Stack size in words

// TEMPORARY // TODO
//#define PRIORITY_ADC_TASK       3
//#define BLOCK_TIME_MAX          1



// FreeRTOS structures.
extern xSemaphoreHandle g_pUARTMutex;
extern xSemaphoreHandle g_adcConvSemaphore;
extern xQueueHandle g_adcReadQueue;



//*****************************************************************************
// This task handles ADC for the helirig, constantly monitoring the height of the rig,
// storing the received values in a circular buffer
//*****************************************************************************
static void
adcQueueTask(void *pvParameters)
{


    // ADCTrigger task triggers ADC conversion, sets semaphore/flag indicating it is ready to be written to the circular buffer
    // ADCQueue task stores the value in the circular buffer upon seeing the flag set then resets the flag

    xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
    char string[100];  // 100 characters across the display
    usnprintf (string, sizeof(string), "ADCQueueTask starting.\r\n");
    UARTSend(string);
    xSemaphoreGive(g_pUARTMutex);



    //
    // Loop forever
    while(1) {


        if (adcTriggerFlag) {

            //
            // Place it in the circular buffer (advancing write index)
            writeCircBuf (&g_inBuffer, ulValue);

            // Reset adcTriggerFlag
        }


        //
        // Clean up, clearing the interrupt
        ADCIntClear(ADC0_BASE, 3);
        //
        // Add the ADC read to queue.
        xQueueSend(g_adcReadQueue, &ulValue, BLOCK_TIME_MAX);


    }


}


//*****************************************************************************
// Initializes the ADC task.
//*****************************************************************************
uint32_t
adcQueueTaskInit(void)
{
    //
    // Initialize ADC things
    initADC();

    //
    // Create the ADC task.
    if(xTaskCreate(adcTask, (const portCHAR *)"ADC",
                   ADCTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_ADC_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    return(0);


}








