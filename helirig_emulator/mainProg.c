//*****************************************************************************
//
// mainProg.c - Time triggered kernel based program that controls the remote
//              helicopters at the University of Canterbury. Contains the 
//              function definitions for the interrupt service routines,
//              each of the seven tasks and main().
//
// Authors:  J.R Crosland, A.K Greer, J.S Chen - UCECE
// Created: 05/04/2018
// Last modified:   30/05/2018
//
//*****************************************************************************
// Based on Milestone_1_b code, but initially based on and expanded from
// Professor P Bones' ADCdemo program. Further expanded and modified to create
// the full program for the helicopter project.
//*****************************************************************************


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h" // Macros for the different ports and peripherals defined here.
#include "driverlib/pin_map.h" // Definitions of pin addresses
#include "driverlib/adc.h" //         }
#include "driverlib/gpio.h" //         }
#include "driverlib/sysctl.h" //        }-> API functions
#include "driverlib/systick.h" //      }
#include "driverlib/interrupt.h" //   }
#include "driverlib/ssi.h"
#include "initialisers.h"
#include "sharedConstantsTypes.h"
#include "mainProg.h"

#define DAC_WRITE_CMD_NGA_NSHDN_2048 0x3800
#define DAC_CMD_ARRAY_LENGTH 4
static const uint32_t dacWriteCmdValue = 0x00003800; // First four zeros are redundant
#define DAC_CMD_LENGTH_W_RDDT_BITS 16

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


//*****************************************************************************
// The handler for the program reset interrupt. If PA6 is pulled down to 0V,
// the system will reset.
//*****************************************************************************
//void
//progResetIntHandler (void)
//{
//    GPIOIntClear (GPIO_PORTA_BASE, GPIO_INT_PIN_6);
//    // Perform soft system reset.
//    SysCtlReset ();
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

//void
//taskUpdateYaw (void)
//{
//
//}

//*****************************************************************************
// Controller task: updates the controller during the calibrate, flying and 
// landing modes. This will return new PWM values for the PWM task to utilise.
//*****************************************************************************
//void
//taskUpdateController (void)
//{
//    programData.currentYaw = getCurrentYaw();
//    // Cycle through the program modes and do as necessary depending on the mode.
//    switch (programData.mode)
//    {
//    case (flying):
//        // Calculate new duty cycle values and update the duty cycle attributes of the
//        // program data struct, in order for the PWM task to reconfigure the duty cycles.
//        updateController(&programData, &yawControlData, &altitudeControlData);
//        break;
//    case (landing):
//        // Check if the helicopter is close enough to the current reference
//        // before decrementing the reference altitude as part of the landing sequence.
//        if (isHeliWithinMargins (&programData, NOISE_MARGIN, YAW_MARGIN))
//        {
//            // Is the helicopter within the landed range?
//            if (programData.referenceAltitude >= landedAltADCVal - (NEAR_LANDED_NOISE_MARGIN))
//            {
//                // Have met the "landed" criteria
//                programData.mode = landed;
//                programData.referenceYaw = 0;
//            }
//            else if (!(programData.referenceAltPercent <= MIN_ALTITUDE_PCT))
//            {
//                // Decrement the reference altitude further
//                programData.referenceAltitude += ALTITUDE_INCREMENT_ADC;
//                programData.referenceAltPercent -= ALTITUDE_INCREMENT_PCT;
//            }
//        }
//        updateController (&programData, &yawControlData, &altitudeControlData);
//        break;
//    }
//}

//*****************************************************************************
// PWM task: sets most recent PWM values. 
//*****************************************************************************
//void
//taskUpdatePWM (void)
//{
//    // Counter variable used as part of the timing of the main rotor PWM
//    // pulsing in calibrate mode.
//    static int calibrateTicks = 0;
//    // Cycle through the program modes and do as necessary depending on the mode.
//    switch (programData.mode)
//    {
//    case(calibrate):
//        enableMainTailPWM ();
//        // Performing the main rotor PWM duty cycling.
//        if (calibrateTicks >= CALIBRATE_TICKS_MAX)
//        {
//            setMainPWM (PWM_DUTY_MAIN_CALIB_HIGH);
//            SysCtlDelay (CALIBRATE_SYSCTL_DELAY);
//            calibrateTicks = 0;
//        }
//        else
//        {
//            setMainPWM (PWM_DUTY_MAIN_CALIB_LOW);
//            calibrateTicks++;
//        }
//        break;
//    case (landed):
//    case (idle):
//        disableMainTailPWM ();
//        break;
//    case (flying):
//        enableMainTailPWM ();
//        setMainPWM (programData.mainMotorPWMDuty);
//        setTailPWM (programData.tailMotorPWMDuty);
//        break;
//    default:
//        // Normal program operation
//        setMainPWM (programData.mainMotorPWMDuty);
//        setTailPWM (programData.tailMotorPWMDuty);
//    }
//}

//*****************************************************************************
// Buttons task: only updates during the flying mode. Polls all four push
// buttons. Updates the reference altitude and yaw if their respective buttons
// are pushed.
//*****************************************************************************
//void
//taskUpdateButtons (void)
//{
//    // Only update the referenceYaw and referenceAltitude if the mode is flying.
//    if (programData.mode == flying)
//    {
//        // Update the current yaw and reference values of the program data struct as necessary
//        // after polling the buttons to see their state change.
//        updateButtons ();
//        // Up button check
//        uint8_t butState = checkButton (UP);
//        if (butState == PUSHED_TOP)
//        {
//            increaseAltitude (&programData);
//        }
//
//        // Down button check
//        butState = checkButton (DOWN);
//        if (butState == PUSHED_TOP)
//        {
//            decreaseAltitude (&programData);
//        }
//
//        // Left button check
//        butState = checkButton (LEFT);
//        if (butState == PUSHED_TOP)
//        {
//            decreaseYaw (&programData);
//        }
//        // Right button check
//        butState = checkButton (RIGHT);
//        if (butState == PUSHED_TOP)
//        {
//            increaseYaw (&programData);
//        }
//    }
//}

//*****************************************************************************
// Slider switch task: polls the slider switch to obtain its current voltage
// level. Causes state transitions as necessary on 'rising' or 'falling'
// edges of the switch.
//*****************************************************************************
//void
//taskUpdateSliderSwitch (void)
//{
//    // Check the current state of slider SW1 on the Orbit board
//    int32_t sliderStatePresent = checkButton (SLIDER);
//    // If current mode is fly and the slider switch has been pushed from up -> down
//    if ((programData.mode == flying) && (isFallingEdgeSlider (sliderStatePresent, sliderStatePast)))
//    {
//            programData.mode = landing;
//            programData.referenceAltitude += ALTITUDE_INCREMENT_ADC;
//            programData.referenceAltPercent -= ALTITUDE_INCREMENT_PCT;
//            // Calculate new reference yaw position which is <= 180 deg movement from current yaw
//            programData.referenceYaw = calculateLandingReferenceYaw (programData.currentYaw);
//            programData.referenceYawDeg = calculateYawDegs (programData.referenceYaw);
//    }
//    else if (isRisingEdgeSlider (sliderStatePresent, sliderStatePast))
//    // The slider switch has been pushed from down -> up
//    {
//        if (programData.mode == landed)
//        {
//            // Transition into the flying mode
//            programData.mode = flying;
//        }
//        else if (programData.mode == idle)
//        {
//            // Transition into the calibrate mode
//            programData.mode = calibrate;
//        }
//    }
//    // Update the past slider state
//    sliderStatePast = sliderStatePresent;
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

//*****************************************************************************
// UART task: sends out current program data.
//*****************************************************************************
//void
//taskUpdateUART(void)
//{
//    // Updates the UART string array with current program data and sends it
//    uartUpdateInformation (&programData, landedAltADCVal);
//}

//void
//taskCheckCalibration (void)
//{
//    static bool isReadyTakeoff = false;
//    if (getCalibrationStatus() && !isReadyTakeoff)
//    {
//        // Set the current yaw counter to 0 in the program data struct and reset
//        // other attributes of the operating data.
//        programData.currentYaw = 0;
//        programData.mode = flying;
//        programData.referenceYaw = 0;
//        programData.referenceAltitude = landedAltADCVal;
//        programData.referenceAltPercent = MIN_ALTITUDE_PCT;
//        programData.referenceYawDeg = 0;
//        programData.mainMotorPWMDuty = PWM_DUTY_FLYING_INIT;
//        programData.tailMotorPWMDuty = PWM_DUTY_FLYING_INIT;
//
//        // Provide both the yaw and altitude controllers with an initial integral
//        // error for a faster takeoff.
//
//        isReadyTakeoff = true;
//    }
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
    for (ui8Idx = 0; ui8Idx < DAC_CMD_ARRAY_LENGTH; ui8Idx++)
    {
        SSIDataPut (SSI0_BASE, pcChars[ui8Idx]);
    }
}

