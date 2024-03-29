// *******************************************************
//
// adcQueueTask.c
//
// Definition of the ADC queue task. This task populates the circular
// buffer with ADC values, and calculates the average of the buffer and
// sends it to the event queue to be processed
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
#include "adcQueueTask.h"
#include "queue_reader.h"

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

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// FreeRTOS task specific defines
#define ADCTASKSTACKSIZE        128         // Stack size in words
extern xSemaphoreHandle g_pUARTMutex;
extern xQueueHandle g_buttsADCEventQueue;
xSemaphoreHandle g_adcConvSemaphore;
xSemaphoreHandle g_calibrationCompleteSemaphore;

// Global, module-specific, non-FreeRTOS defines
static circBuf_t g_inBuffer;     // Buffer of size BUF_SIZE
extern uint32_t g_landedAltitudeADCValue;
extern uint32_t g_fullAltitudeADCValue;

//*****************************************************************************
// This task handles calculating, then sending the current average ADC value
// stored in the circular buffer to the program status through use of the event queue
//*****************************************************************************
static void
adcQueueTask(void *pvParameters)
{

    portTickType ui16LastTime;
    uint32_t ui32PollDelay = 2;
    hwEventQueueItem_t eventItem;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount();

    // Used for the 'calibration stage'
    volatile uint32_t calibrationDone = 0;
    uint32_t circBufcounter = 0;

    xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
    UARTprintf("ADCQueueTask starting.\n");
    xSemaphoreGive(g_pUARTMutex);

    // Loop forever
    while(1)
    {
        // If flag is set
        if (xSemaphoreTake(g_adcConvSemaphore, portMAX_DELAY) == pdTRUE)
        {

            // Value to be read
            uint32_t ulValue;

            // Get the single sample from ADC0 and write it to ulValue
            ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);

            // Place it in the circular buffer (advancing write index)
            writeCircBuf (&g_inBuffer, ulValue);

            if (calibrationDone == 1)
            {
                // Normal operation can commence

                // Create event message to send, calculate buffer average
                eventItem.buttonADCEventType = ADC_BUFFER_UPDATED_EVENT;
                eventItem.adcBufferAverage = calculateMeanHeight();

                // Append event message to the queue
                if (xQueueSend (g_buttsADCEventQueue, &eventItem, portMAX_DELAY) != pdPASS)
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
            else if (calibrationDone == 0)
            {
                // Calibration mode commencing

                circBufcounter++;

                if (circBufcounter >= BUF_SIZE)
                {
                    calibrationDone = 1;

                    g_landedAltitudeADCValue = calculateMeanHeight();
                    g_fullAltitudeADCValue = g_landedAltitudeADCValue - HELI_OFFSET_FULL;

                    // Signaling that calibration is complete, buffer is full and average has been computed
                    if (xSemaphoreGive(g_calibrationCompleteSemaphore) != pdPASS)
                    {
                        xSemaphoreTake (g_pUARTMutex, portMAX_DELAY);
                        UARTprintf("\nCouldn't give semaphore for calibration!\n");
                        xSemaphoreGive (g_pUARTMutex);
                    }

                    xSemaphoreTake (g_pUARTMutex, portMAX_DELAY);
                    UARTprintf("CALIBRATION DONE, AVERAGE COMPUTED\n");
                    xSemaphoreGive (g_pUARTMutex);
                }
            }
        }
        else
        {
            // Failed to take the semaphore
            xSemaphoreTake (g_pUARTMutex, portMAX_DELAY);
            UARTprintf("\nCouldn't take semaphore for Queue task\n");
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
    // Initialise the circular buffer used for ADC values
    initCircBuf (&g_inBuffer, BUF_SIZE);

    // Create FreeRTOS structures used
    g_calibrationCompleteSemaphore = xSemaphoreCreateBinary();
    g_adcConvSemaphore = xSemaphoreCreateBinary();

    // Create the ADC task.
    if(xTaskCreate(adcQueueTask, (const portCHAR *)"ADC",
                   ADCTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_ADC_TRIGGER_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    UARTprintf("ADC Queue task initialized.\n");

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
    {
            sum = sum + readCircBuf (&g_inBuffer);
    }
    // Calculate the rounded mean of the buffer contents
    uint32_t mean = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;

    return mean;
}

