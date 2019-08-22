// ***********************************************************************************
//
// sharedConstants.h
//
// Header file of constants and type definitions that are used in several 
// modules and need to be shared. The most important content is the 
// OperatingData_t struct

// Authors:  J.R Crosland, Lachlan Brewster
//           
// Created:  07/04/2018
// Last modified:  23/08/2019
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
#define MAX_ADC_VALUE 4095 
// Below is for converting from the 1.0 volt range to its proportion in 0-4095
#define HELI_OFFSET_FULL 1240
#define NUM_CHARS_ORBIT_DISPLAY 16
#define PWM_DUTY_MAX 90
#define PWM_DUTY_MIN 5

// Defining upper and lower bounds on 32-bit float types used to avoid overflow
#define MAX_32_FLOAT_VALUE 3.403E38
#define MIN_32_FLOAT_VALUE -3.403E38


// Maximum and minimum bounds on the altitude and increment values
#define MAX_ALTITUDE_ADC 1000   // mV
#define MIN_ALTITUDE_ADC 2000   // mV
#define MAX_ALTITUDE_PCT 100
#define MIN_ALTITUDE_PCT 0
#define ALTITUDE_INCREMENT_PCT 10
#define ALTITUDE_INCREMENT_ADC 124

/* Starting positions when the program is initialized */
#define STARTING_REF_ALT_PCT (MIN_ALTITUDE_PCT)
#define STARTING_REF_ALT_DIG (MIN_ALTITUDE_ADC)
#define STARTING_ALT_INTG_ERROR 1600
#define STARTING_MAIN_MOTOR_DUTY 0


//*****************************************************************************
// Flight modes
//*****************************************************************************
typedef enum helimode_e {calibrate, landed, flying, landing} HeliMode;


//*****************************************************************************
// Display modes
//*****************************************************************************
typedef enum displaymodes_e {Altitude, ADCMean, OFF} displayModes;


typedef struct operating_data_t_s {
    HeliMode mode;
    uint32_t referenceAltDig;       // Percentage altitude converted to digital
    uint32_t currentAltDig;         // ADC voltage representation between 0 - 4095
    uint8_t referenceAltPercent;    // Percentage value between 0 and 100
    uint8_t mainMotorPWMDuty;
    int32_t pPartAlt;               // Proportional error component
    int32_t iPartAlt;               // Integral error component
} OperatingData_t;


#endif /* SHAREDCONSTANTS_H_ */
