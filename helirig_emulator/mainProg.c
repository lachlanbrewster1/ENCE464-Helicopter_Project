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
#include "../driverlib/pwm.h"
#include "initialisers.h"
#include "sharedConstantsTypes.h"
#include "mainProg.h"
#include "../inc/tm4c123gh6pm.h"
#include "../inc/hw_types.h"
#include "../driverlib/timer.h"


//Input Edge Time Capture include and defines
volatile double duty = 0, start = 0, stop = 0;
#define PWM_Frequency 250 //Hz
#define PWM_DIVIDER_CODE     SYSCTL_PWMDIV_4
#define SAMPLE_RATE_HZ     250
#define SYSTICK_RATE_HZ    100    // SysTick interrupt config.


//DAC related includes
#define DAC_WRITE_CMD_NGA_NSHDN_2048 0x3800
#define DAC_CMD_ARRAY_LENGTH 12
static const uint32_t dacWriteCmdValue = 0x00003800; // First four zeros are redundant
#define DAC_CMD_LENGTH_W_RDDT_BITS 16


//Helicopter model related includes and globals
volatile double g_current_height = 0;     //Current height of the helicopter initialised to zero


//*****************************************************************************
// Local prototypes
//*****************************************************************************
void initClock (void);
void SysTickIntHandler(void);
void helicopterHeight (void);
void initTimer (void);
void initDACSignals (void);
void timer_int (void);
void dutyCycle (void);
int conversion (void);
void DAC_Inputs (int16_t bits);

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
// This is attempt 2 to model the behaviour of the helicopter
//*****************************************************************************
void
helicopterHeight (void)
{
    //Force due to weight is a changing quantity which is influenced from the current height of the helicopter.
    double weight_force = 0.2 + g_current_height/100;

    //Effective thrust is the force of the motor minus the weight force
    double thrust = duty - (weight_force * 100);

    //Acceleration is dependent upon the effective thrust of the motor
    double acceleration = ((thrust*thrust)/400);

    //Check the direction
    if (thrust < 0) {
        acceleration = -acceleration;
    }

    //change in height is the effective thrust multiplied by the sampling rate
    g_current_height += acceleration * SAMPLE_RATE_HZ; //This is the change in height calculation

    if (g_current_height < 0) {
        g_current_height = 0;
    }

    if (g_current_height > 100) {
        g_current_height = 100;
    }
}



//*****************************************************************************
// Function to initalise all the required features to measure duty cycle of input PWM signal
//*****************************************************************************
void
initTimer(void)
{
    //Calls to initialise Interrupts and PWM reads
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOIntRegister (GPIO_PORTC_BASE, dutyCycle);
    GPIOPinTypeGPIOInput (GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOIntTypeSet (GPIO_PORTC_BASE, GPIO_INT_PIN_4, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_4);

    //Initalise Timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerIntRegister(TIMER0_BASE, TIMER_A, timer_int);
    TimerEnable(TIMER0_BASE, TIMER_A);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}



//*****************************************************************************
// Function to initalise all the required signals for the DAC
//*****************************************************************************
void
initDACSignals (void)
{
    //For the Chip Select Signal (PIN 2), SCK (PIN 4) and LDAC (PIN 5)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);

    //Send clock signal for the SCK line
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PC5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);
    SysCtlPWMClockSet (SYSCTL_PWMDIV_4);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the initial PWM parameters
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / 4 / 250;

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32Period);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,
        ui32Period * 50 / 100);


    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

}



//*****************************************************************************
// Interrupt handler for the timer
//*****************************************************************************
void
timer_int (void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}



//*****************************************************************************
// Function calculate the duty cycle of the input PWM signal
//*****************************************************************************
void
dutyCycle(void)
{
    //When negative edge is hit, record the values and find the difference
    int8_t state = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4);

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



//*****************************************************************************
// converts the data into a suitable format for the DAC
//*****************************************************************************
int
conversion (void)
{
    double voltage = 2 - g_current_height/100;

    int16_t bits = 8190 - 4095*voltage;

    return bits;
}



//*****************************************************************************
// Functions to change all the signals of the DAC
//*****************************************************************************
void
DAC_Inputs (int16_t bits)
{
    //Set Chip Select to Low for the transmission of the Data
    GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_2, 0);

    //Set the LDAC signal to High
    GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_5, 1);

    //Send the Bits of data to the SDI
    bits = conversion();
    bits += 4096; //This is equivalent to 0001 0000 0000 0000 which is used to set the bits of the headers for the 12-bit DAC
    SSIDataPut(SSI0_BASE, bits);


    //Rest the Chip Select and LDAC lines
    GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_2, 1);
    GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_5, 0);
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
    SSIConfigSetExpClk (SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                        SSI_MODE_MASTER, 2000000, 16);

    // Enable the SSI module
    SSIEnable (SSI0_BASE);



    //Configure to receive input PWM signals
    initTimer();

    //Initalise all the DAC signals. That is the Chip Select and LDAC
    initDACSignals();

    g_current_height = 60;

    //local variable for the bits sent to the DAC
    int16_t bits;

    while(true)
    {
        /*for (ui8Idx = 0; ui8Idx < DAC_CMD_ARRAY_LENGTH; ui8Idx++)
        {
            if (ui8Idx == 12) {
                SSIDataPut (SSI0_BASE, pcChars[ui8Idx]);
            } else if
        }*/
        //helicopterHeight ();

        DAC_Inputs(bits);
    }
}

