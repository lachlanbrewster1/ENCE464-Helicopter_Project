#include <stdint.h>
#include <stdbool.h>
#include "stdbool.h"


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
#include "adcQueueTask.h"
#include "queue_reader.h"
#include "sharedConstants.h"



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

// FreeRTOS structures.
extern xSemaphoreHandle g_pUARTMutex;
extern xQueueHandle g_buttsAdcEventQueue;
xSemaphoreHandle g_adcConvSemaphore;
xSemaphoreHandle g_calibrationCompleteSemaphore;


//*****************************************************************************
// This task handles calculating, then sending the current average ADC value
// stored in the circular buffer to the program status through use of the event queue
//*****************************************************************************
static void
adcQueueTask(void *pvParameters)
{

    portTickType ui16LastTime;
    uint32_t ui32PollDelay = 20;
    hwEventQueueItem_t eventItem;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount();

    uint32_t calibrationDone = 0;
    uint32_t circBufcounter = 0;


    xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
    UARTprintf("ADCQueueTask starting.\n");
    xSemaphoreGive(g_pUARTMutex);

    //
    // Loop forever
    while(1) {

        if (xSemaphoreTake(g_adcConvSemaphore, portMAX_DELAY) == pdTRUE) {       // If flag is set

            // If ADC value is ready
            //if (ADCIntStatus(ADC0_BASE, 3, true)) {
            if (true) {

                // Value to be read
                uint32_t ulValue;

                //
                // Get the single sample from ADC0 and write it to ulValue
                ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);

                xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
                char string[31];
                usnprintf (string, sizeof(string), "ADC value: %d\r\n", ulValue);
                UARTprintf(string);
                xSemaphoreGive(g_pUARTMutex);

                //
                // Place it in the circular buffer (advancing write index)
                writeCircBuf (&g_inBuffer, ulValue);


                if (calibrationDone == 0) {circBufcounter++;}

            }


            if (calibrationDone == 1) {

                // Create event message to send, calculate buffer average
                eventItem.buttonADCEventType = ADC_BUFFER_UPDATED_EVENT;
                eventItem.adcBufferAverage = calculateMeanHeight();

                // Append event message to the queue
                if (xQueueSend (g_buttsAdcEventQueue, &eventItem, portMAX_DELAY) != pdPASS)
                {
                    // Queue is full - not good. Should never happen
                    xSemaphoreTake (g_pUARTMutex, portMAX_DELAY);
                    UARTprintf("\nQueue full. This should never happen.\n");
                    xSemaphoreGive (g_pUARTMutex);
                    while(1)
                    {
                        // Infinite loop
                    }
                }

            } else {

                if (circBufcounter >= BUF_SIZE) {
                    calibrationDone = 1;

                    xSemaphoreTake (g_pUARTMutex, portMAX_DELAY);
                    UARTprintf("\nCALIBRATION DONE\n");
                    xSemaphoreGive (g_pUARTMutex);

                    xSemaphoreGive(g_calibrationCompleteSemaphore);

                    // Create event message to send, calculate buffer average
                   eventItem.buttonADCEventType = ADC_BUFFER_UPDATED_EVENT;
                   eventItem.adcBufferAverage = calculateMeanHeight();

                   // Append event message to the queue
                   if (xQueueSend (g_buttsAdcEventQueue, &eventItem, portMAX_DELAY) != pdPASS)
                   {
                       // Queue is full - not good. Should never happen
                       xSemaphoreTake (g_pUARTMutex, portMAX_DELAY);
                       UARTprintf("\nQueue full. This should never happen.\n");
                       xSemaphoreGive (g_pUARTMutex);
                       while(1)
                       {
                           // Infinite loop
                       }
                   }

                }


            }



        } else {

            xSemaphoreTake (g_pUARTMutex, portMAX_DELAY);
            UARTprintf("\nCouldn't take semaphore for Q task\n");
            xSemaphoreGive (g_pUARTMutex);

        }

        // Wait for the required amount of time.
        vTaskDelayUntil (&ui16LastTime, ui32PollDelay / portTICK_RATE_MS);

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
    // initADC(); // Done in ADC Trigger Task

    initCircBuf (&g_inBuffer, BUF_SIZE);

    g_calibrationCompleteSemaphore = xSemaphoreCreateBinary();
    g_adcConvSemaphore = xSemaphoreCreateBinary();


    //
    // Create the ADC task.
    if(xTaskCreate(adcQueueTask, (const portCHAR *)"ADC",
                   ADCTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_ADC_TRIGGER_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    UARTprintf("ADC Queue task initialized.\n");

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







