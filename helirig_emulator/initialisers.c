// *****************************************************************************
//
// initialisers.c
//
// Support for a set of four specific buttons and one slider switch on the Tiva/Orbit.
// The buttons are:  UP and DOWN (on the Orbit daughterboard) plus
// LEFT and RIGHT on the Tiva.
// SW1 (switch 1) is on the Orbit daughterboard.
//
// Note: Pin PF0 (the pin for the RIGHT pushbutton - SW2 on
// the Tiva board) needs special treatment - See PhilsNotesOnTiva.rtf.
//
// Note: The virtual signal and the remote interface does not correctly simulate
//       a physical slider switch. As such, slider switch one is debounced the same
//       way as the buttons. The naming of the functions and variables does not make
//       a distinction between the buttons and the slider switch.
//
// Authors:  J.R Crosland, A.K Greer, J.S Chen - UCECE, largely based on
//	         sample code 'buttons4.c' by Professor P. Bones.
// Created:  07/02/2018
// Last modified:  31/05/2018
//
// *****************************************************************************


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h" // Macros for the different ports and peripherals defined here.
#include "driverlib/pin_map.h" // Definitions of pin addresses
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "initialisers.h"
#include "mainProg.h"
//#include "uart.h"
//#include "buttons.h"
//#include "pwmGen.h"



// *****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, yaw, display
// and UART
// *****************************************************************************
//void
//initOperatingData (OperatingData_t* data, uint32_t altValue)
//{
//    data->mode = idle;
//    data->referenceYaw = STARTING_REFERENCE;
//    data->referenceAltitude = altValue;
//    data->currentYaw = STARTING_POSITION;
//    data->currentAltitude = altValue;
//    data->mainMotorPWMDuty = PWM_DUTY_IDLE_STATE;
//    data->tailMotorPWMDuty = PWM_DUTY_IDLE_STATE;
//    data->referenceYawDeg = STARTING_REFERENCE;
//    data->referenceAltPercent = STARTING_REFERENCE;
//    data->iPartYaw = 0;
//    data->pPartYaw = 0;
//    data->iPartAlt = 0;
//    data->pPartAlt = 0;
//}

void initSSIGPIO (void)
{
    // Enable the GPIO port A peripheral
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);
    // Enabling the SSI0 peripheral
    SysCtlPeripheralEnable (SYSCTL_PERIPH_SSI0);
    // Wait for it to be ready
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








