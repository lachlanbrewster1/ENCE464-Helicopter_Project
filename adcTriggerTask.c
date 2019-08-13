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
// The stack size for the ADC task.
//
//*****************************************************************************
#define ADCTASKSTACKSIZE        128         // Stack size in words

// TEMPORARY // TODO
//#define PRIORITY_ADC_TASK       3
//#define BLOCK_TIME_MAX          1



// FreeRTOS structures.
extern xSemaphoreHandle g_pUARTMutex;
extern xQueueHandle g_adcReadQueue;



//*****************************************************************************
// This task handles ADC for the helirig, constantly monitoring the height of the rig,
// storing the received values in a circular buffer
//*****************************************************************************
static void
adcTriggerTask(void *pvParameters)
{


    //xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
    //UARTprintf("ADCTask starting.\n");
    //xSemaphoreGive(g_pUARTMutex);

    xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
    char string[100];  // 100 characters across the display
    usnprintf (string, sizeof(string), "ADCTriggerTask starting.\r\n");
    UARTSend(string);
    xSemaphoreGive(g_pUARTMutex);



    //
    // Loop forever
    while(1) {

        // ADCIntHandler();    // old

        // Value to be read
        uint32_t ulValue;

        // Trigger ADC conversion. // ISR?
        ADCProcessorTrigger(ADC0_BASE, 3);
        //
        // Wait for sample.
        // while(!ADCIntStatus(ADC_BASE, ADC_SEQUENCE, false));

        //
        // Get the single sample from ADC0.
        ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);

//        xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
//        char string[31];
//        usnprintf (string, sizeof(string), "ADC value: %d\r\n", ulValue);
//        UARTSend(string);
//        xSemaphoreGive(g_pUARTMutex);

        //
        // Place it in the circular buffer (advancing write index)
        writeCircBuf (&g_inBuffer, ulValue);
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
adcTriggerTaskInit(void)
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


//*****************************************************************************
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//*****************************************************************************
void
ADCIntHandler(void)
{
    uint32_t ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}

//*****************************************************************************
// Initialisation functions for the ADC
//*****************************************************************************
void
initADC (void)
{

    initCircBuf (&g_inBuffer, BUF_SIZE);

    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}






//*****************************************************************************
// Calculate and return the rounded mean of the buffer contents
//*****************************************************************************
uint16_t
calculateMeanHeight(void)
{
    uint32_t sum = 0;
    uint16_t i = 0;

    for ( i = 0; i < BUF_SIZE; i++)
            sum = sum + readCircBuf (&g_inBuffer);
        // Calculate and display the rounded mean of the buffer contents
    uint16_t mean = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
    return (2 * (100 * (landed_ref - mean)) + 1000) / 2 / 1000; //100% *(our new height) / 1000mV
}

//*****************************************************************************
// Set the landed reference of the helicopter.
// Uses the rounded mean of the circular buffer contents
//*****************************************************************************
void
setLandedRef(void)
{
    uint16_t i;
    uint32_t sum = 0;
    for (i = 0; i < BUF_SIZE; i++)
            sum = sum + readCircBuf (&g_inBuffer);
        // Calculate the rounded mean of the buffer contents
    landed_ref = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
}
