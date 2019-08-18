//*****************************************************************************
//
// freertos_demo.c - Simple FreeRTOS example.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "stdio.h"
#include "stdlib.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"

#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

#include "led_task.h"
#include "switch_task.h"
#include "adcQueueTask.h"
#include "adcTriggerTask.h"
#include "uartTask.h"

#include "uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#define DATA_QUEUE_LENGTH               1
#define DATA_QUEUE_ITEM_SIZE            sizeof(uint16_t)



//*****************************************************************************
// FreeRTOS structures used by program. Includes The mutex that protects
// concurrent access of UART from multiple tasks, Queue for ADC and PWM task,
// and queue for adc and button events
//*****************************************************************************

xQueueHandle g_buttsAdcEventQueue;
xSemaphoreHandle g_queueMutex;        // Mutex to guard the event queue from being modified

xSemaphoreHandle g_pUARTMutex;      // Mutex to guard the UART.
SemaphoreHandle_t g_adcConvSemaphore;    // Flag to signal the ADC value is ready to be written to buffer




//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}


//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run at 50 MHz from the PLL.
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    //ConfigureUART();
    initialiseUSB_UART();

    //
    // Print introduction.
    UARTprintf("\n\nWelcome to the ENCE464 helirig thing!\n");

    //
    // Creating needed FreeRTOS structures
    g_buttsAdcEventQueue = xQueueCreate(DATA_QUEUE_LENGTH, DATA_QUEUE_ITEM_SIZE);
    g_pUARTMutex = xSemaphoreCreateMutex();
    g_adcConvSemaphore = xSemaphoreCreateBinary();


//    //
//    // Create the UART task.
//    if(uartTaskInit() != 0)
//    {
//
//        while(1)
//        {
//        }
//    }

//    //
//    // Create the LED task.
//    if(LEDTaskInit() != 0)
//    {
//
//        while(1)
//        {
//        }
//    }
//
//    //
//    // Create the switch task.
//    if(SwitchTaskInit() != 0)
//    {
//
//        while(1)
//        {
//        }
//    }

//    //
//    // Create the ADC queue task.
//    if(adcQueueTaskInit() != 0)
//    {
//
//        while(1)
//        {
//        }
//    }

    //
    // Create the ADC trigger task.
    if(adcTriggerTaskInit() != 0)
    {

        while(1)
        {
        }
    }

    //
    // Create the PWM task.
    if(pwmTaskInit() != 0)
    {

        while(1)
        {
        }
    }


    //
    // Start the scheduler.  This should not return.
    vTaskStartScheduler();

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    while(1)
    {
    }
}










////*****************************************************************************
////
//// Configure the UART and its pins.  This must be called before UARTprintf().
////
////*****************************************************************************
//void
//ConfigureUART(void)
//{
//    //
//    // Enable the GPIO Peripheral used by the UART.
//    //
//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//
//    //
//    // Enable UART0
//    //
//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
//
//    //
//    // Configure GPIO Pins for UART mode.
//    //
//    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
//    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
//    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//
//    //
//    // Use the internal 16MHz oscillator as the UART clock source.
//    //
//    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
//
//    //
//    // Initialize the UART for console I/O.
//    //
//    UARTStdioConfig(0, 115200, 16000000);
//}
