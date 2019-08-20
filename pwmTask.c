/*
 * pwm.c
 *
 *  Created on: 1/08/2019
 *      Author: lbr63
 *
 *  Uses a modified version of pwnGen.c, by P.J. Bones UCECE
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"
#include "stdlib.h"

#include <priorities.h>
#include "uart.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"

#include "utils/ustdlib.h"
#include "circBufT.h"
#include "pwmTask.h"
#include "sharedConstants.h"


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"




//*****************************************************************************
//
// The stack size for the PWM task.
//
//*****************************************************************************
#define PWMTASKSTACKSIZE        128         // Stack size in words

// FreeRTOS structures.
extern xSemaphoreHandle g_pUARTMutex;
extern xQueueHandle g_pwmWriteQueue;
extern OperatingData_t g_programStatus;



//*****************************************************************************
// This task handles PWM for the helirig,  ...     ...
//*****************************************************************************
static void
pwmTask(void *pvParameters)
{

    portTickType ui16LastTime;
    uint32_t ui32PollDelay = 25;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount ();


    xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
    UARTprintf("PWMTask starting.\n");
    xSemaphoreGive(g_pUARTMutex);


    //
    // Loop forever
    while(1) {


        // Get values from program status // TODO what import do I use?
        //uint32_t mainMotorPWMDuty = operating_data_t_s.mainMotorPWMDuty;

        uint32_t mainMotorPWMDuty = 30;


        //
        // Set duty cycle of main rotor
        // Might need to scale it first? What kind of value is received?
        setDutyCycle(mainMotorPWMDuty, MAIN_ROTOR);


		// Wait for the required amount of time.
		vTaskDelayUntil (&ui16LastTime, ui32PollDelay / portTICK_RATE_MS);

    }

}


//*****************************************************************************
// Initializes the PWM task.
//*****************************************************************************
uint32_t
pwmTaskInit(void)
{
    //
    // Initialize PWM things
    initPWM();

    //
    // Create the PWM task.
    if(xTaskCreate(pwmTask, (const portCHAR *)"PWM",
                   PWMTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_PWM_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    UARTprintf("PWM task initialized.\n");

    //
    // Success.
    return(0);


}


//*****************************************************************************
// Initialisation functions for the PWM. Initialises module 0 and module 1.
// Using the constants defined in pwmTask.h
//*****************************************************************************
void
initPWM (void)
{
    // Initialise main rotor
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters

    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, SysCtlClockGet() / PWM_FIXED_RATE_HZ);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);


    // Initialise secondary rotor
    SysCtlPeripheralEnable(PWM_SECONDARY_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_SECONDARY_PERIPH_GPIO);

    GPIOPinConfigure(PWM_SECONDARY_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_SECONDARY_GPIO_BASE, PWM_SECONDARY_GPIO_PIN);

    PWMGenConfigure(PWM_SECONDARY_BASE, PWM_SECONDARY_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    PWMGenPeriodSet(PWM_SECONDARY_BASE, PWM_SECONDARY_GEN, SysCtlClockGet() / PWM_FIXED_RATE_HZ);

    PWMGenEnable(PWM_SECONDARY_BASE, PWM_SECONDARY_GEN);

    PWMOutputState(PWM_SECONDARY_BASE, PWM_SECONDARY_OUTBIT, true);
}


//*****************************************************************
//  Function to set the duty cycle of the main, or secondary rotor
// ****************************************************************
void
setDutyCycle (uint32_t ui32Duty, uint8_t rotor)
{

    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_FIXED_RATE_HZ;

    if (rotor == MAIN_ROTOR){
        PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
        ui32Period * ui32Duty / 100);

        xSemaphoreTake(g_pUARTMutex, BLOCK_TIME_MAX);
        char string[31];
        usnprintf (string, sizeof(string), "Set rotor duty cycle to: %d\r\n", ui32Duty);
        UARTprintf(string);
        xSemaphoreGive(g_pUARTMutex);

    }
    else if (rotor == SECONDARY_ROTOR){
        PWMPulseWidthSet(PWM_SECONDARY_BASE, PWM_SECONDARY_OUTNUM,
            ui32Period * ui32Duty / 100);
    }
}
