// ***********************************************************************************
//
// sharedConstants.h
//
// Header file of constants and type definitions that are used in several 
// modules and need to be shared. The most important content is the 
// OperatingData_t struct

// Authors:  J.R Crosland, A.K Greer, J.S Chen - UCECE 
//           
// Created:  07/04/2018
// Last modified:  03/06/2018
//
// ****************************************************************************


#ifndef SHAREDCONSTANTS_H_
#define SHAREDCONSTANTS_H_

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 30
#define NOISE_MARGIN 15
#define YAW_MARGIN 12
// Below is for converting from the 1.0 volt range to its proportion in 0-4095
#define HELI_OFFSET_FULL 1240
#define NUM_DEGS_REVOLUTION 360
#define STATE_CHANGES_SLOT 4
#define NUM_SLOTS 112
#define STATE_CHANGES_REVOLUTION (STATE_CHANGES_SLOT * NUM_SLOTS)
#define STATE_CHANGES_DEGREE ((STATE_CHANGES_SLOT * NUM_SLOTS) / NUM_DEGS_REVOLUTION)
#define NUM_CHARS_ORBIT_DISPLAY 16
#define PWM_DUTY_MAX 98
#define PWM_DUTY_MIN 5
#define HELI_BAUD_RATE 9600

#define STARTING_REFERENCE 0
#define STARTING_POSITION 0

// Maximum and minimum bounds on the altitude and yaw and increment values
#define MAX_ALTITUDE_ADC 1240
#define MIN_ALTITUDE_ADC 2482
#define MAX_ALTITUDE_PCT 100
#define MIN_ALTITUDE_PCT 0
#define ALTITUDE_INCREMENT_PCT 10
#define ALTITUDE_INCREMENT_ADC 124
#define YAW_INCREMENT_DEGREES 15
// Calculated from the number of state changes * YAW INCREMENT DEGREES. 
// Hard coded to avoid integer arithmetic.
#define YAW_INCREMENT_STATE_CHANGES 19 

// This is the truth table that takes the current and previous yaw level states
// of channels A and B of the helicopter rig as inputs. It outputs an increment
// for a clockwise state change or a decrement for a counterclockwise state
// change. The state is the bit-packed byte representation of the logic levels
// of channels A and B and follows a Gray code.
static const int32_t yawLookupTable[4][4] = {
                        {0, 1, -1, 0},
                        {-1, 0, 0, 1},
                        {1, 0, 0, -1},
                        {0, -1, 1, 0}
                        };


//*****************************************************************************
// Flight modes
//*****************************************************************************
typedef enum helimode_e {idle, calibrate, landed, flying, landing} HeliMode;

//*****************************************************************************
// Display modes
//*****************************************************************************
typedef enum displaymodes_e {Altitude, ADCMean, OFF} displayModes;

typedef struct operating_data_t_s {
    HeliMode mode;
    uint32_t referenceAltDig; // Percentage altitude converted to digital
    uint32_t currentAltDig; // ADC voltage representation between 0 - 4095
    int32_t referenceAltPercent; // Percentage value between 0 and 100
    uint8_t mainMotorPWMDuty;
} OperatingData_t;

/* Name of buttons, switch and ADC event write queue: g_butsADCEventQueue */

#endif /* SHAREDCONSTANTS_H_ */
