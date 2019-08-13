//*****************************************************************************
//
// mainProg.c - just currently runs an infinite loops that sends a string
// over SPI using the SSI peripheral. Originally borrowed from the 361 helicopter
// projects
//
// Authors:  J.R Crosland- UCECE
// Created: 05/04/2018
// Last modified:   29/07/2019
//
//*****************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "../inc/hw_memmap.h" // Macros for the different ports and peripherals defined here.
#include "../driverlib/pin_map.h" // Definitions of pin addresses
#include "../driverlib/adc.h" //         }
#include "../driverlib/gpio.h" //         }
#include "../driverlib/sysctl.h" //        }-> API functions
#include "../driverlib/systick.h" //      }
#include "../driverlib/interrupt.h" //   }
#include "../driverlib/ssi.h"
#include "../initialisers.h"
#include "../sharedConstantsTypes.h"
#include "mainProg.h"
#include "../inc/tm4c123gh6pm.h"
#include "../inc/hw_types.h"
#include "../driverlib/timer.h"


#include "../inc/hw_ints.h"
#include "../driverlib/pwm.h"
#include "../driverlib/debug.h"
#include "../utils/ustdlib.h"
#include "../driverlib/uart.h"



//Input Edge Time Capture include and defines
volatile double duty = 0, start = 0, stop = 0;
#define PWM_Frequency 250 //Hz
#define PWM_DIVIDER_CODE     SYSCTL_PWMDIV_4
#define SAMPLE_RATE_HZ     250
#define SYSTICK_RATE_HZ    100    // SysTick interrupt config.
static uint32_t g_ulSampCnt;



//DAC related includes
#define DAC_WRITE_CMD_NGA_NSHDN_2048 0x3800
#define DAC_CMD_ARRAY_LENGTH 4
static const uint32_t dacWriteCmdValue = 0x00003800; // First four zeros are redundant
#define DAC_CMD_LENGTH_W_RDDT_BITS 16


//Helicopter model related includes and globals
volatile double current_height = 0;     //Current height of the helicopter initialised to zero


//*****************************************************************************
// Local prototypes
//*****************************************************************************
void initClock (void);
void SysTickIntHandler(void);
void helicopterHeight (void);
void initTimer (void);
void dutyCycle (void);

// ----------------------------------------------------------------------------
// Start of interrupt handler definitions
// ----------------------------------------------------------------------------

//*****************************************************************************
// The interrupt handler for the for SysTick interrupt.
//*****************************************************************************
//void
//sysTickIntHandler (void)
//{
//    uint8_t i;
//    for (i = 0; i < NUM_TASKS; i++)
//    {
//        if ((scheduledTasks[i].ticks == 0) && scheduledTasks[i].enabled)
//        {
//            scheduledTasks[i].ready = true;
//            scheduledTasks[i].ticks = scheduledTasks[i].timingPeriod;
//        }
//        else
//        {
//            scheduledTasks[i].ticks -= 1;
//        }
//    }
//}

// ----------------------------------------------------------------------------
// End of interrupt handler definitions
// ----------------------------------------------------------------------------
// Start of task definitions
// ----------------------------------------------------------------------------
// Definitions of task functions for triggering ADC conversions,
// updating the states of the buttons and switches, updating
// the controller, updating the PWM, refreshing the display
// and transmitting current program data with the UART.
// ****************************************************************************

//*****************************************************************************
// ADC task: triggers an ADC conversion and updates the current altitude.
// Also sets the zero altitude reference ADC value.
//*****************************************************************************

//void
//taskUpdateADC (void)
//{
//    // Trigger an ADC conversion, this will call the adcIntHandler to do the sampling.
//    ADCProcessorTrigger (ADC1_BASE, ADC_SEQUENCE_NUM);
//    // Update the current altitude.
//    programData.currentAltitude = signalAveraging ();
//    // Initial slight delay process of 30 SysTick cycles to ensure the buffer fills at program start
//    // before setting the landed ADC value.
//    if (initLandedADCTicks >= BUF_SIZE && !(initLandedADCDone))
//    {
//        landedAltADCVal = signalAveraging ();
//        initLandedADCDone = true;
//    }
//    else
//    {
//        initLandedADCTicks += 1;
//    }
//}

//*****************************************************************************
// Display task: updates the display with current program values.
//*****************************************************************************
//void
//taskUpdateDisplay (void)
//{
//    // Update the display using the current altitude and yaw, which are contained in the program struct
//    displayStats (&programData, landedAltADCVal);
//}

// ----------------------------------------------------------------------------
// End of task definitions
// ----------------------------------------------------------------------------

//*****************************************************************************
// Initialises the scheduled tasks with their timing periods, enabled status
// and their function pointer.
//*****************************************************************************
//void
//initialiseScheduledTasks (Task_t* arrayPointer)
//{
//    uint8_t i;
//    for (i = 0; i < NUM_TASKS; i++)
//    {
//       arrayPointer[i].timingPeriod = taskTimingPeriods[i];
//       arrayPointer[i].ticks = taskTimingPeriods[i];
//       arrayPointer[i].ready = false;
//       arrayPointer[i].enabled = taskEnableStatus[i];
//       arrayPointer[i].run = taskFctnPointers[i];
//    }
//}

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    // Set up the period for the SysTick timer
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    SysTickIntRegister(SysTickIntHandler);
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    SysTickIntEnable();
    SysTickEnable();
}

//*****************************************************************************
// Interrupt handler for the SysTick interrupt.
//*****************************************************************************
void
SysTickIntHandler(void)
{
    // Initiate a conversion
    ADCProcessorTrigger(ADC0_BASE, 3);
    g_ulSampCnt++;
}


//*****************************************************************************
// This is attempt 1 to model the behaviour of the helicopter
//*****************************************************************************
/*void
helicopterHeight (double main_thrust)
{
    //Effect of the main rotor on the height of the helicopter.
    //Equation from discretising is (5*z^2)/(5*z^2 + 0.5z)
    double main_rotor_thrust;
    main_rotor_thrust = (5*main_thrust*main_thrust)/(5*main_thrust*main_thrust + 0.5*main_thrust);

    //Main Force vs PWM LUT
    //Going to use a generic scalar of 10.
    main_rotor_thrust = main_rotor_thrust * 10;

    //Effect of Gravity
    //Assuming that the mass of the helicopter is 0.25 kg
    //Therefore, force g = 0.25*9.81 = 2.4525
    main_rotor_thrust = main_rotor_thrust - 2.4525;

    //Effect from the helicopter mount
    //Equation from discretising is 1.5*((10*z^2)/(10*z^2 + 3*z))
    current_height = 1.5*((10*main_rotor_thrust*main_rotor_thrust)/(10*main_rotor_thrust*main_rotor_thrust + 3*main_rotor_thrust));
}*/



//*****************************************************************************
// This is attempt 2 to model the behaviour of the helicopter
//*****************************************************************************
void
helicopterHeight (void)
{
    //Force due to weight is a changing quantity which is influenced from the current height of the helicopter.
    double weight_force = 0.2 + current_height/100;

    //Effective thrust is the force of the motor minus the weight force
    double thrust = duty - (weight_force * 100);

    //Acceleration is dependent upon the effective thrust of the motor
    double acceleration = ((thrust*thrust)/400);

    //Check the direction
    if (thrust < 0) {
        acceleration = -acceleration;
    }

    //change in height is the effective thrust multiplied by the sampling rate
    current_height += acceleration * SAMPLE_RATE_HZ; //This is the change in height calculation

    if (current_height < 0) {
        current_height = 0;
    }

    if (current_height > 100) {
        current_height = 100;
    }
}

//*****************************************************************************
// Function to initalise all the required features to measure duty cycle of input PWM signal
//*****************************************************************************
void
initTimer(void)
{
    //Calls to initialise Interrupts and PWM reads
    GPIOIntRegister (GPIO_PORTC_BASE, dutyCycle);
    GPIOPinTypeGPIOInput (GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOIntTypeSet (GPIO_PORTC_BASE, GPIO_INT_PIN_4, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_4);

    //Initalise Timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_A_CAP_TIME);



}

//*****************************************************************************
// Function calculate the duty cycle of the input PWM signal
//*****************************************************************************
void
dutyCycle(void)
{
    //When negative edge is hit, record the values and find the difference
    int32_t state = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4);

    //If the signal is a leading edge
    if (state == 1) {
        start = TimerValueGet(TIMER0_BASE, TIMER_A);
    } else { //If the signal is falling edge
        stop = TimerValueGet(TIMER0_BASE, TIMER_A);
        //Only want to calculate on the falling edge or else duty will go from positive to negative.
        duty = (stop - start)*PWM_Frequency*100;
    }

    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
}

int
main (void)
{
    // Reset just the SSI peripheral for now
    peripheralReset ();
    // Initialise the CPU clock
    initClock ();
    initSSIGPIO ();
    // Enable interrupts to the processor.
    IntMasterEnable ();
    
    // Wait for the SSI0 module to be ready
    while (!(SysCtlPeripheralReady (SYSCTL_PERIPH_SSI0)))
    {
    }

    // Configuring the SSI
    SSIConfigSetExpClk (SSI0_BASE, SysCtlClockGet (), SSI_FRF_MOTO_MODE_0,
                        SSI_MODE_MASTER, 2000000, 4);

    // Enable the SSI module
    SSIEnable (SSI0_BASE);

    // Send DAC command
    uint8_t ui8Idx;
    char *pcChars = "DACA";
    // Loop and send this string infinitely

    //Configure to receive input PWM signals
    initTimer();

    TimerEnable(TIMER0_BASE, TIMER_BOTH);

    while(true)
    {
        for (ui8Idx = 0; ui8Idx < DAC_CMD_ARRAY_LENGTH; ui8Idx++)
        {
            SSIDataPut (SSI0_BASE, pcChars[ui8Idx]);
        }

        helicopterHeight ();
    }
}

