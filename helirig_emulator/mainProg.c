//*****************************************************************************
//
// mainProg.c - just currently runs an infinite loops that sends a string
// over SPI using the SSI peripheral. Originally borrowed from the 361 helicopter
// projects
//
// Authors:  J.R Crosland and Greg Bates - UCECE
// Created: 05/04/2018
// Last modified:   17/08/2019
//
//*****************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
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
#include "../inc/hw_ints.h"

//Flags for the Timer
uint32_t g_ui32Flags;
volatile uint32_t g_ui32IntCount;

//Input Edge Time Capture include and defines
volatile int32_t g_duty = 0, g_start = 0, g_stop = 0;
volatile int32_t g_duty_prev = 0;
#define DUTY_SCALAR 800
#define VOLT_BITS 2000 //This is the rate of change of bits to voltage for the bit conversion for the DAC

volatile int8_t state = 0;

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
#define GRAVITY       10
#define WEIGHT        0.08
#define DELTA_T       0.004

//volatile double g_current_height = 0;     //Current height of the helicopter initialised to zero
//volatile double g_prev_height = 0;
//
//volatile double g_MR_t = 0;
//volatile double g_MR_t_prev = 0;
//
//volatile double g_Force = 0;
//
//volatile double g_G_t = 0;
//volatile double g_G_t_prev = 0;
//
//volatile double g_HM_t = 0;
//volatile double g_HM_t_prev = 0;


volatile double g_height = 0;     //Current height of the helicopter initialised to zero

volatile int32_t g_MR_t = 0;
volatile int32_t g_MR_t_prev = 0;

volatile double g_Force = 0;

volatile double g_G_t = 0;
volatile double g_G_t_prev = 0;

volatile double g_HM_t = 0;
volatile double g_HM_t_prev = 0;

int32_t index = 0;
int16_t bits;


//Look-up Table
/*struct LUT_PWM
{
    int PWM;
    double force;
};

static struct LUT_PWM LUT_PWM_DATA[] =
{
    { 0 , 0 },
    { 5 , 0.0268 },
    { 10 , 0.1609 },
    { 15 , 0.3142 },
    { 20 , 0.4748 },
    { 25 , 0.6190 },
    { 30 , 0.7626 },
    { 35 , 0.8796 },
    { 40 , 1.0104 },
    { 45 , 1.1098 },
    { 50 , 1.2155 },
    { 55 , 1.3227 },
    { 60 , 1.4339 },
    { 65 , 1.5124 },
    { 70 , 1.6039 },
    { 75 , 1.6824 },
    { 80 , 1.7707 }
};*/

double LUT_PWM_DATA[17] = {0, 0.0268, 0.1609, 0.3142, 0.4748, 0.6190, 0.7626, 0.8796, 1.0104, 1.1098, 1.2155, 1.3227, 1.4339, 1.5124, 1.6039, 1.6824, 1.7707};

//enum { NUM_LUT_PWM = sizeof(LUT_PWM_DATA) / sizeof(LUT_PWM_DATA[1]) };



//*****************************************************************************
// Local prototypes
//*****************************************************************************
//void helicopterHeight1 (void);
void helicopterHeight2 (void);
void initTimer (void);
void initDACSignals (void);
void timer_int (void);
void dutyCycle (void);
void DAC_Transmitt (void);

// ----------------------------------------------------------------------------
// Start of interrupt handler definitions
// ----------------------------------------------------------------------------



//*****************************************************************************
// Code for the behaviour of the helicopter based on Phil Bones model
//*****************************************************************************
/*void
helicopterHeight1 (void)
{
    //Constants
    double k_1 = 1;
    double t_1 = 0.1;
    double k_2 = 1.53;
    double t_2 = 0.638;

    //Main Rotor part of the Heli Rig Model
    g_MR_t = g_MR_t_prev*(1 - DELTA_T/t_1) + k_1*g_duty_prev/t_1;

    //LUT stuff
    uint32_t index = 0;
    index = round(85*g_MR_t/8000);
    //index = 85*g_MR_t/8000;
    g_Force = LUT_PWM_DATA[index].force;

    //Gravitational force on the system effect on Heli Rig Model
    g_G_t = g_Force - GRAVITY * WEIGHT;

    //Helicopter Mount part of the Heli Rig Model
    g_HM_t = g_HM_t_prev*(1 - DELTA_T/t_2) + k_2*g_G_t_prev/t_2;

    //Integrator Limited
    g_height += g_HM_t*DELTA_T*SCALAR;

    //Keep safes incase of the worst extremes
    if(g_height < 0) {
        g_height = 0;
    } else if (g_height > 100) {
        g_height = 100;
    }

    //Update the Variables.
    g_duty_prev = g_duty;
    g_MR_t_prev = g_MR_t;
    g_G_t_prev = g_G_t;
    g_HM_t_prev = g_HM_t;

}*/



//*****************************************************************************
// Code for the behaviour of the helicopter based on Phil Bones model
//*****************************************************************************
void
helicopterHeight2 (void)
{
    //Constants
    int k_1 = 1;
    double t_1 = 0.1;
    double k_2 = 1.53;
    double t_2 = 0.638;

    //Main Rotor part of the Heli Rig Model
    g_MR_t = k_1*g_duty_prev/t_1;

    //LUT stuff
    index = round(g_MR_t*17/900);
    if (index < 0) {
        index = 0;
    } else if (index > 17) {
        index = 17;
    }
    g_Force = LUT_PWM_DATA[index];

    //Gravitational force on the system effect on Heli Rig Model
    g_G_t = g_Force - GRAVITY * WEIGHT;

    //Helicopter Mount part of the Heli Rig Model
    g_HM_t = k_2*g_G_t_prev/t_2;

    //Integrator Limited
    g_height += g_HM_t*DELTA_T;

    //Keep safes incase of the worst extremes
    if(g_height < 0) {
        g_height = 0;
    } else if (g_height > 100) {
        g_height = 100;
    }

    //Update the Variables.
    g_duty_prev = g_duty;
    g_MR_t_prev = g_MR_t;
    g_G_t_prev = g_G_t;
    g_HM_t_prev = g_HM_t;

}



//*****************************************************************************
// Function to initalise all the required features to measure duty cycle of input PWM signal
//*****************************************************************************
void
initTimer(void)
{
    //Calls to initialise Interrupts and PWM reads

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
    {
    }
    GPIOIntRegister (GPIO_PORTC_BASE, dutyCycle);
    GPIOPinTypeGPIOInput (GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOIntTypeSet (GPIO_PORTC_BASE, GPIO_INT_PIN_4, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_4);


    //Initalise Timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC_UP);
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);
    TimerEnable(TIMER0_BASE, TIMER_A);
}



//*****************************************************************************
// Function to initalise all the required signals for the DAC
//*****************************************************************************
void
initDACSignals (void)
{
    //For the Chip Select Signal (PIN 1), SHDN (PIN 4) and LDAC (PIN 3)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {

    }

    //GPIOPinConfigure(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_3);
}



//*****************************************************************************
// Interrupt handler for the timer
//*****************************************************************************
void
timer_int (void)
{
    //Update the interrupt counter
    g_ui32IntCount++;

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}



//*****************************************************************************
// Function calculate the duty cycle of the input PWM signal
//*****************************************************************************
void
dutyCycle(void)
{
    //When negative edge is hit, record the values and find the difference
    //int32_t state = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4);

    state = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4);

    //If the signal is a falling edge
    if (state == 0) {
        g_stop = TimerValueGet(TIMER0_BASE, TIMER_A);
        //Only want to calculate on the falling edge or else duty will go from positive to negative.
        g_duty = (g_stop - g_start)/DUTY_SCALAR;
    } else { //If the signal is leading edge
        g_start = TimerValueGet(TIMER0_BASE, TIMER_A);

    }

    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
}



//*****************************************************************************
// Functions to change all the signals of the DAC
//*****************************************************************************
void
DAC_Transmitt (void)
{
    //converts the height to the voltage
    double voltage = 2 - g_height/100;

    //Converts the voltage to a bit string
    bits = VOLT_BITS*voltage;

    bits += 12288; //This is equivalent to 0011 0000 0000 0000 which is used to set the bits of the headers for the 12-bit DAC

    //Send the Bits of data to the SDI
    SSIDataPut(SSI0_BASE, bits);
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

    //Initalise all the DAC signals. That is the Chip Select, SHDN and LDAC
    initDACSignals();

    //Preset the Chip Select Signal to High
    GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);

    //Set the LDAC signal to High
    GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

    //Set the SHDN signal to High
    GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6);

    while(true)
    {
        /*for (ui8Idx = 0; ui8Idx < DAC_CMD_ARRAY_LENGTH; ui8Idx++)
        {
            if (ui8Idx == 12) {
                SSIDataPut (SSI0_BASE, pcChars[ui8Idx]);
            } else if
        }*/

        helicopterHeight2();

        //Set Chip Select to Low for the transmission of the Data
        GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_1, 0);

        DAC_Transmitt();

        SysCtlDelay(50);

        //Rest the Chip Select line to high
        GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);

        //Toggle LDAC
        GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_3, 0);

        SysCtlDelay(50);

        GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
    }
}
