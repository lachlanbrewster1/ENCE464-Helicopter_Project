// *******************************************************
//
// pwmTask.c
//
// Definition of the PWM task. This task handles the generation of a PWM
// signal to power the main helicopter rotor. It monitors the desired PWM
// Duty cycle using the program status structure
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
#include "pwmTask.h"
#include "sharedConstants.h"

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

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// FreeRTOS task specific defines
#define PWMTASKSTACKSIZE        128         // Stack size in words
extern xSemaphoreHandle g_pUARTMutex;
extern xQueueHandle g_pwmWriteQueue;

// Global, module-specific, non-FreeRTOS defines
extern OperatingData_t g_programStatus;

//*****************************************************************************
// This task handles PWM generation for the helirig, It gets the desired
// PWM duty cycle from the program status structure, and sets the Main rotor
// to that duty cycle
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

    // Loop forever
    while(1)
    {

        if (g_programStatus.mode == flying || g_programStatus.mode == landing)
        {
            // Enable the motors
            PWMOutputState (PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);

            // Get the PWM value from program status
            uint32_t mainMotorPWMDuty = g_programStatus.mainMotorPWMDuty;

            // Set duty cycle of the main rotor
            setDutyCycle(mainMotorPWMDuty);

        } else
        {
            // Disable the motors
            PWMOutputState (PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
        }

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
    // Initialize PWM things
    initPWM();

    // Create the PWM task.
    if(xTaskCreate(pwmTask, (const portCHAR *)"PWM",
                   PWMTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_PWM_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    UARTprintf("PWM task initialized.\n");

    // Success.
    return(0);

}

//*****************************************************************************
// Initialisation functions for the PWM. Initialises module 0.
// Using the constants defined in pwmTask.h
//*****************************************************************************
void
initPWM (void)
{

    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);

    // Initialise main rotor
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);

}

//*****************************************************************
//  Function to set the duty cycle of the main rotor
// ****************************************************************
void
setDutyCycle (uint32_t ui32Duty)
{

    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / PWM_FIXED_RATE_HZ;

    // Set the PWM period
    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);

    // Set the PWM pulse width
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
    ui32Period * ui32Duty / 100);
}
