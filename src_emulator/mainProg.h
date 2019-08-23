/*
 * mainProg.h
 *
 *  Created on: 08/04/2018
 *  Last modified: 09/04/2018
 *  Authors: Jozef Crosland jrc149
 *            
 */

#ifndef MAINPROG_H_
#define MAINPROG_H_

#include <stdint.h>
#include <stdbool.h>
#include "sharedConstantsTypes.h"

//*****************************************************************************
// Constants
//*****************************************************************************


// Sampling frequency needs to be at least 2 x BUF_SIZE x 4
#define NOISE_MARGIN 30
#define NEAR_LANDED_NOISE_MARGIN (NOISE_MARGIN / 2)
#define YAW_MARGIN 12
#define NUM_TASKS 8
#define SYSTICK_SAMPLE_RATE_HZ 600 // Needs to be a common multiple of all the task frequencies

// Task execution frequencies (Hz)
#define TASK_RATE_ADC 300
#define TASK_RATE_BUTS_CONT_PWM 100 // Same rate for button polling and controller, PWM updates
#define TASK_RATE_DISPLAY 20
#define TASK_RATE_SLIDER 10
#define TASK_RATE_UART 2
#define TASK_RATE_CHECK_CALIBRATION 300

// Counter and delay values for the calibrate mode
#define CALIBRATE_TICKS_MAX 35
#define CALIBRATE_SYSCTL_DELAY 9

// Number of ticks to achieve execution frequency dependent on Systick triggering frequency
#define TIMING_PERIOD_ADC (SYSTICK_SAMPLE_RATE_HZ / TASK_RATE_ADC)
#define TIMING_PERIOD_BUTS_CONT_PWM (SYSTICK_SAMPLE_RATE_HZ / TASK_RATE_BUTS_CONT_PWM)
#define TIMING_PERIOD_DISPLAY (SYSTICK_SAMPLE_RATE_HZ / TASK_RATE_DISPLAY)
#define TIMING_PERIOD_SLIDER (SYSTICK_SAMPLE_RATE_HZ / TASK_RATE_SLIDER)
#define TIMING_PERIOD_UART (SYSTICK_SAMPLE_RATE_HZ / TASK_RATE_UART)
#define TIMING_PERIOD_CHECK_CALIBRATION (SYSTICK_SAMPLE_RATE_HZ / TASK_RATE_CHECK_CALIBRATION)

// Declaring the function pointer type
typedef void (*functionPointer_t) (void);

// Timing periods for the tasks 
static const uint16_t taskTimingPeriods[NUM_TASKS] = {
    TIMING_PERIOD_ADC, 
    TIMING_PERIOD_BUTS_CONT_PWM,
    TIMING_PERIOD_BUTS_CONT_PWM,
    TIMING_PERIOD_BUTS_CONT_PWM, 
    TIMING_PERIOD_SLIDER,
    TIMING_PERIOD_DISPLAY, 
    TIMING_PERIOD_UART,
    TIMING_PERIOD_CHECK_CALIBRATION,
    };

// Enables for the tasks 
static const bool taskEnableStatus[NUM_TASKS] = {
    true, // ADC trigger task
    true, // Controller task
    true, // PWM update task
    true, // Buttons update task
    true, // Slider switch update task
    true, // Display task (not needed for project demonstration)
    true, // UART update and send task
    true
    };

// Struct for each task, containing the timing period, its current tick value, ready 
// and enabled flags and the function pointer to the task function.
typedef struct task_t_s {
    uint16_t timingPeriod;
    uint16_t ticks;
    bool ready;
    bool enabled;
    void (*run) (void);
} Task_t;

//*****************************************************************************
// The interrupt handler for the for SysTick interrupt.
//*****************************************************************************
void
sysTickIntHandler (void);

//*****************************************************************************
// The handler for the program reset interrupt.
// Resets the program either via virtual signal or by button.
//*****************************************************************************
void 
progResetIntHandler (void);


//*****************************************************************************
// ADC task: triggers an ADC conversion and updates the current altitude.
// Also sets the zero altitude reference ADC value.
//*****************************************************************************
void
taskUpdateADC (void);

//*****************************************************************************
// Controller task: updates the controller during the calibrate, flying and 
// landing modes. This will return new PWM values for the PWM task to utilise.
//*****************************************************************************
void
taskUpdateController (void);

//*****************************************************************************
// PWM task: sets most recent PWM values. 
//*****************************************************************************
void
taskUpdatePWM (void);

//*****************************************************************************
// Buttons task: only updates during the flying mode. Polls all four push
// buttons. Updates the reference altitude and yaw if their respective buttons
// are pushed.
//*****************************************************************************
void
taskUpdateButtons (void);

//*****************************************************************************
// Slider switch task: polls the slider switch to obtain its current voltage
// level. Causes state transitions as necessary on 'rising' or 'falling'
// edges of the switch.
//*****************************************************************************
void
taskUpdateSliderSwitch (void);

//*****************************************************************************
// Display task: updates the display with current program values.
//*****************************************************************************
void
taskUpdateDisplay (void);

//*****************************************************************************
// UART task: sends out current program data.
//*****************************************************************************
void
taskUpdateUART (void);

void
taskCheckCalibration (void);

// List of function pointers to the tasks
static const functionPointer_t taskFctnPointers[NUM_TASKS] = {
    taskUpdateADC,
    taskUpdateController,
    taskUpdatePWM,
    taskUpdateButtons,
    taskUpdateSliderSwitch,
    taskUpdateDisplay,
    taskUpdateUART,
    taskCheckCalibration
};

#endif /* MAINPROG_H_ */
