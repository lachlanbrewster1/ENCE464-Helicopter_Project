// *****************************************************************************
//
// initialisers.c
// Currently just includes function definitions for initialising the SSI peripheral
// to be used as SPI.
//
// Authors:  J.R Crosland- UCECE
// Created:  07/02/2018
// Last modified:  31/05/2018
//
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "../inc/hw_memmap.h" // Macros for the different ports and peripherals defined here.
#include "../driverlib/pin_map.h" // Definitions of pin addresses
#include "../driverlib/adc.h"
#include "../driverlib/gpio.h"
#include "../driverlib/sysctl.h"
#include "../driverlib/systick.h"
#include "initialisers.h"
#include "mainProg.h"

// *****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, yaw, display
// and UART
// *****************************************************************************
void
initSSIGPIO (void)
{
    // Enable the GPIO port A peripheral
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);
    // Enabling the SSI0 peripheral
    SysCtlPeripheralEnable (SYSCTL_PERIPH_SSI0);
    // Wait for it to be ready
    // Adding string
    while (!(SysCtlPeripheralReady (SYSCTL_PERIPH_GPIOA)))
    {
    }
    while (!(SysCtlPeripheralReady (SYSCTL_PERIPH_SSI0)))
    {
    }
    // Configure pins PA2 and PA5 as SSI0Clk and SSI0Tx with SSI0 peripheral
    GPIOPinTypeSSI (GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_5);
    // Configure PA2 and PA5 pins specifically
    GPIOPinConfigure (GPIO_PA2_SSI0CLK);
    GPIOPinConfigure (GPIO_PA5_SSI0TX);
}

void
peripheralReset(void)
{
    // Reset GPIOA peripheral (used by SSI0 peripheral)
    SysCtlPeripheralReset (SYSCTL_PERIPH_GPIOA);

    // Reset GPIOA peripheral (Used for Chip Select on the DAC)
    SysCtlPeripheralReset (SYSCTL_PERIPH_GPIOC);

    // Reset DAC functions
    SysCtlPeripheralReset (SYSCTL_PERIPH_GPIOB);

    // Reset SSI peripheral
    SysCtlPeripheralReset (SYSCTL_PERIPH_SSI0);
}

void
initClock(void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
//    SysTickPeriodSet(SysCtlClockGet() / SYSTICK_SAMPLE_RATE_HZ);
    //
//    // Register the interrupt handler
//    SysTickIntRegister(sysTickIntHandler);
//    //
//    // Enable interrupt and device
//    SysTickIntEnable();
//    SysTickEnable();
}



//void
//initProgReset(void)
//{
//    // Enable GPIO Port A
//    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//    // Wait until Port A is ready to avoid a bus fault
//    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
//    {
//        // twiddle your thumbs
//        continue;
//    }
//    // Registering program reset handler
//    GPIOIntRegister(GPIO_PORTA_BASE, progResetIntHandler);
//    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);
//    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA,
//                    GPIO_PIN_TYPE_STD_WPU);
//    // Low level triggered interrupt
//    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
//    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6);
//}








