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
//#define PRIORITY_ADC_QUEUE_TASK       3
//#define BLOCK_TIME_MAX          1



// FreeRTOS structures.
extern xSemaphoreHandle g_pUARTMutex;
extern xSemaphoreHandle g_adcConvSemaphore;
extern xQueueHandle g_buttsAdcEventQueue;



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


    // butEvents_t eventMessage;
    uint32_t meanADCValue;


    //
    // Loop forever
    while(1) {

        // eventMessage = ADC_BUFFER_UPDATED_EVENT;

        if (g_adcConvSemaphore) {       // If flag is set // TODO


            // Calculate average
            meanADCValue = calculateMeanHeight();

//            // Append event message to the queue
//            if (xQueueSend (g_buttsAdcEventQueue, &eventMessage, portMAX_DELAY) != pdPASS)
//            {
//                // Queue is full - not good. Should never happen
//                xSemaphoreTake (g_pUARTMutex, portMAX_DELAY);
//                UARTprintf("\nQueue full. This should never happen.\n");
//                xSemaphoreGive (g_pUARTMutex);
//                while(1)
//                {
//                    // Infinite loop
//                }
//        }




            // Put item on event queue    hw_evt_queue_item_e, which contains ADC_BUFFER_UPDATED_EVENT and uint32_t adcBufferAverag

            // Reset adcTriggerFlag
            // g_adcConvSemaphore = 0;       // TODO what value to set it to?
        }


        //
        // Clean up, clearing the interrupt
        ADCIntClear(ADC0_BASE, 3);

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
    if(xTaskCreate(adcQueueTask, (const portCHAR *)"ADC",
                   ADCTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_ADC_TRIGGER_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    return(0);


}





//*****************************************************************************
// Calculate and return the rounded mean of the buffer contents
//*****************************************************************************
uint32_t
calculateMeanHeight(void)
{
    uint32_t sum = 0;
    uint16_t i = 0;

    for ( i = 0; i < BUF_SIZE; i++)
            sum = sum + readCircBuf (&g_inBuffer);

    // Calculate the rounded mean of the buffer contents
    uint32_t mean = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;

    // return (2 * (100 * (landed_ref - mean)) + 1000) / 2 / 1000; //100% *(our new height) / 1000mV
    return mean;
}







