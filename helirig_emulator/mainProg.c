//*****************************************************************************
//
// mainProg.c - Has the entire functionality for a working Emulator for the
// height of the helicopter.
//
// Authors:  J.R Crosland and Greg Bates - UCECE
// Created: 05/04/2018
// Last modified:   22/08/2019
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


//Input Edge Time Capture include and defines
volatile int32_t g_duty = 0, g_start = 0, g_stop = 0;  //Start is the count on the leading edge. Stop is the count on the falling edge.
volatile int32_t g_duty_prev = 0;                      //Previous duty cycle count.
#define DUTY_SCALAR 800 //This is the scalar of Count to Duty calculation.
#define VOLT_BITS 2000 //This is the rate of change of bits to voltage for the bit conversion for the DAC

volatile int8_t g_state = 0;    //State variable to determine if rising edge or falling edge on the PWM

#define PWM_Frequency 250 //Hz
#define PWM_DIVIDER_CODE     SYSCTL_PWMDIV_4
#define SAMPLE_RATE_HZ     250
#define SYSTICK_RATE_HZ    100    // SysTick interrupt config.


//DAC related includes
#define DAC_WRITE_CMD_NGA_NSHDN_2048 0x3800
#define DAC_CMD_ARRAY_LENGTH 12  //This defines that we are using the 12 Bit DAC
static const uint32_t dacWriteCmdValue = 0x00003800; // First four zeros are redundant
#define DAC_CMD_LENGTH_W_RDDT_BITS 16


//Helicopter model related includes and globals
#define GRAVITY       10 //This is the acceleration due to gravity (M/s^2)
#define WEIGHT        0.08 //Approximated mass of the system.
#define DELTA_T       0.004 //This is 1/sampling frequency. Used for the helirig model.


//Helirig variables. Each variable represents the next stage in the model.
volatile double g_height = 0;     //Current height of the helicopter initialised to zero

volatile int32_t g_MR_t = 0;      //Main Rotor
volatile int32_t g_MR_t_prev = 0;

volatile double g_force = 0;      //Force due to gravity

volatile double g_G_t = 0;        //Thrust after the force of gravity has been subtracked.
volatile double g_G_t_prev = 0;

volatile double g_HM_t = 0;       //Helicopter Mount effect.
volatile double g_HM_t_prev = 0;

//Index used for the LUT. This is to index into the LUT.
int32_t g_index = 0;

//This is the global variable used for the bit string to the DAC.
int16_t g_bits;

//LUT for the main force vs PWM.
double LUT_PWM_DATA[17] = {0, 0.0268, 0.1609, 0.3142, 0.4748, 0.6190, 0.7626, 0.8796, 1.0104, 1.1098, 1.2155, 1.3227, 1.4339, 1.5124, 1.6039, 1.6824, 1.7707};




//*****************************************************************************
// Local prototypes
//*****************************************************************************

void helicopterHeight (void);
void initTimer (void);
void initDACSignals (void);
void dutyCycle (void);
void DACTransmitt (void);


//*****************************************************************************
// Code for the behaviour of the helicopter based on Phil Bones model
//*****************************************************************************
void
helicopterHeight (void)
{
    //Constants
    int8_t k_1 = 1;
    double t_1 = 0.1;
    double k_2 = 1.53;
    double t_2 = 0.638;

    //Main Rotor part of the Heli Rig Model
    g_MR_t = k_1*g_duty_prev/t_1;

    //LUT indexing. the 17 and 900 round the index value to be in the length of the LUT.
    g_index = round(g_MR_t*17/900);

    //Safe case for when the counter ticks down therefore forcing the duty high above indexing range for one period. Very uncommon event.
    if (g_index < 0) {
        g_index = 0;
    } else if (g_index > 17) {
        g_index = 17;
    }
    g_force = LUT_PWM_DATA[g_index];

    //Gravitational force on the system effect on Heli Rig Model
    g_G_t = g_force - GRAVITY * WEIGHT;

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

    //Update the previous variables to be the new variables.
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
// Function to initalise all the required controlsignals for the DAC.
//These being the Chip Select, Shutdown and LDAC.
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
// Function calculate the duty cycle of the input PWM signal.
// This is an interrupt driven function
//*****************************************************************************
void
dutyCycle(void)
{
    //State determines whether the PWM is a rising edge or falling edge.
    g_state = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4);

    //If the signal is a falling edge
    if (g_state == 0) {
        //TimerValueGet takes the time when the PWM is a falling edge.
        g_stop = TimerValueGet(TIMER0_BASE, TIMER_A);
        //Only want to calculate on the falling edge or else duty will go from positive to negative.
        g_duty = (g_stop - g_start)/DUTY_SCALAR; //This Duty Scalar works because we know the frequency of the input signal (250Hz)
    } else { //If the signal is leading edge
        g_start = TimerValueGet(TIMER0_BASE, TIMER_A);
    }

    //Clear the interrupt on the ISR
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
}



//*****************************************************************************
// Functions to change the height to a usable bit string to send to the DAC and
// then transmitt the bit string on the SSI.
//*****************************************************************************
void
DACTransmitt (void)
{
    //converts the height to a voltage between 1-2volts.
    double voltage = 2 - g_height/100;

    //Converts the voltage to a bit string
    g_bits = VOLT_BITS*voltage; //VOLT_BITS is a basic scalar for the conversion.

    g_bits += 12288; //This is equivalent to 0011 0000 0000 0000 which is used to set the bits of the headers for the 12-bit DAC

    //Send the Bits of data to the SDI
    SSIDataPut(SSI0_BASE, g_bits);
}



int
main (void)
{
    // Reset just the SSI peripheral for now
    peripheralReset ();

    // Initialise the CPU clock
    initClock ();

    //Initialise the SSI
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

    //Configure to receive input PWM sign als
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
        //Calls for the Helicopter height to be calculated.
        helicopterHeight();

        //Set Chip Select signal to Low for the transmission of the Data
        GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_1, 0);

        //Transmit the height.
        DACTransmitt();

        SysCtlDelay(50);

        //Reset the Chip Select line to high
        GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);

        //Toggle LDAC
        GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_3, 0);

        //Basic wait function so that the LDAC does not go back high straight away.
        SysCtlDelay(50);

        GPIOPinWrite (GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
    }
}

