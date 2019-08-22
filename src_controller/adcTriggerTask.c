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

#include "sharedConstants.h"

#include "stdio.h"
#include "stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"



//*****************************************************************************
//
// The stack size for the ADC task.
//
//*****************************************************************************
#define ADCTASKSTACKSIZE        128         // Stack size in words


// FreeRTOS structures.
extern xSemaphoreHandle g_pUARTMutex;
xSemaphoreHandle g_adcConvSemaphore;


//*****************************************************************************
// This task handles monitoring ADC values of the helirig, constantly monitoring the height,
// sets a flag after storing the received value in a circular buffer
//*****************************************************************************
static void
adcTriggerTask(void *pvParameters)
{

    portTickType ui16LastTime;
    uint32_t ui32PollDelay = 3;


    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount ();


    xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
    UARTprintf("ADCTriggerTask starting.\n");
    xSemaphoreGive(g_pUARTMutex);


    //
    // Loop forever
    while(1) {

        // Trigger ADC conversion.
        ADCProcessorTrigger(ADC0_BASE, 3);

        // Wait for the required amount of time.
        vTaskDelayUntil (&ui16LastTime, ui32PollDelay / portTICK_RATE_MS);

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

    g_adcConvSemaphore = xSemaphoreCreateBinary();

    //
    // Create the ADC task.
    if(xTaskCreate(adcTriggerTask, (const portCHAR *)"ADC",
                   ADCTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_ADC_TRIGGER_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    UARTprintf("ADC trigger task initialized.\n");

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

    // Set ADC conversion flag
    xSemaphoreGiveFromISR(g_adcConvSemaphore, NULL);

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



////*****************************************************************************
//// Set the landed reference of the helicopter.
//// Uses the rounded mean of the circular buffer contents
////*****************************************************************************
//void
//setLandedRef(void)
//{
//    uint16_t i;
//    uint32_t sum = 0;
//    for (i = 0; i < BUF_SIZE; i++)
//            sum = sum + readCircBuf (&g_inBuffer);
//        // Calculate the rounded mean of the buffer contents
//    landed_ref = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
//}
