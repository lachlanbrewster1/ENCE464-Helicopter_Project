// *******************************************************
// 
// controller_task.c
//
// Procedures that implement PID control for the helicopter
// program
//
// Author: Jozef Crosland | jrc149 | 49782422
// Last modified:  14/08/2019
// 
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "controller_task.h"
#include "queue_reader.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/uartstdio.h"
#include "sharedConstants.h"


/* FreeRTOS task specific defines */
#define CONTROLLER_TASK_STACK_SIZE		128
#define CONTROLLER_TASK_POLL_DELAY		3.333
#define SWITCH_EVENT_ITEM_SIZE			sizeof (hwEvent_t)
#define SWITCH_EVENT_QUEUE_SIZE			10


/* Controller application specific defines */
#define TO_LOW					2
#define TO_HIGH					98

#define FROM_LOW_ALT			0
#define FROM_HIGH_ALT			1240000

#define FROM_LOW_YAW			0
#define FROM_HIGH_YAW			448000

#define PWM_SATURATION			    95
#define CONT_TIME_STEP			    0.01
#define INIT_ALT_INTG_ERROR		    1600
#define ZERO_INIT_INTG_ERROR	    0
#define NUM_GAINS_PID			    3
#define PROPORTIONAL_GAIN           1500.0
#define INTEGRAL_GAIN               450.0
#define DERIVATIVE_GAIN             350.0
#define NOISE_MARGIN                15
#define NEAR_LANDED_NOISE_MARGIN    (NOISE_MARGIN / 2)


/* Queue handle and queue mutex handles which are to be initialized in this module. */
xQueueHandle g_switchEventQueue;				// Accessed hardware event queue reader task
uint32_t g_landedAltitudeADCValue = MIN_ALTITUDE_ADC;
uint32_t g_fullAltitudeADCValue;


/* Externally defined global variables, both FreeRTOS-specific and helicopter program specific */
extern xSemaphoreHandle g_pUARTMutex;		// Accessed by most tasks
extern OperatingData_t g_programStatus;			// Accessed by the queue reader, controller, PWM and UART tasks
extern xSemaphoreHandle g_calibrationCompleteSemaphore;



/* Debug strings used to print to serial in debug version of the executable */
#ifdef DEBUG
static const char* downEventString = "SLIDER_PUSH_DOWN_EVENT\n";
static const char* upEventString = "SLIDER_PUSH_UP_EVENT\n";
#endif


/* Control data structure used for the altitude controller.
Initialized with the gains and initial integral error */
static ControlData_t g_altControlData = 
{
	.dt = CONT_TIME_STEP,
	.kp = PROPORTIONAL_GAIN,
	.ki = INTEGRAL_GAIN,
	.kd = DERIVATIVE_GAIN,
	.intError = INIT_ALT_INTG_ERROR
};


/* Maps from the controller output to the main motor PWM duty cycle */
static uint8_t
mapToMainDuty (uint32_t controllerOutput)
{
	uint32_t result = ((controllerOutput - FROM_LOW_ALT) * (TO_HIGH - TO_LOW))
	/ (FROM_HIGH_ALT - FROM_LOW_ALT) + TO_LOW;
	
	return ((uint8_t) result);
}


/* Main procedure for calculating the main motor control PWM input.
Does all the grunty PID maths */
static uint8_t
controllerAlt (int32_t error, uint8_t currentPWM, ControlData_t* data, OperatingData_t* programInfo)
{
	int32_t invertedError = -error;
	int32_t iPart = 0;
	int32_t pPart = 0;
	int32_t preconv;
	int32_t dPart;
	uint16_t returnValue;

	/* To prevent integrator windup, only add to integral error if the PWM is not at maximum.
	Uses the trapezium method */
	if (currentPWM <= PWM_SATURATION)
	{
		data->intError += ((data->pastError + invertedError) / 2) * data->dt; //Trapezium method.
	}

	// Calculate the P, I, and D components
	pPart = data->kp * invertedError;
	dPart = data->kd * (invertedError - data->pastError) / data->dt;
	iPart = data->ki * data->intError;

	// Pass information into task communication structs
	data->pastError = invertedError;
	programInfo->iPartAlt = iPart;
	programInfo->pPartAlt = pPart;

	/* Calculate the return value (sum the P, I, and D components),
	and truncate it if it is below zero, or above the 98% value */
	preconv = pPart + iPart + dPart;
	if (preconv < FROM_LOW_ALT) {
		preconv = FROM_LOW_ALT;
	}
	returnValue = mapToMainDuty(preconv);

	//If the return value is greater than 98% duty cycle, cap it at 98%
	if (returnValue > PWM_DUTY_MAX)
	{
		returnValue = PWM_DUTY_MAX;
	}

	return returnValue;
}


/* Updates the altitude controller */
static void
updateController (OperatingData_t* status, ControlData_t* altData)
{
    // Compute error and pass into controller
    int32_t altError = status->referenceAltDig - status->currentAltDig;
    status->mainMotorPWMDuty = controllerAlt (altError, status->mainMotorPWMDuty, altData, status);
}


/* Returns true if the input event type is a slider switch event type */
static bool
isEventTypeSlider (hwEvent_t eventType)
{
	bool retVal = ((eventType == SLIDER_PUSH_DOWN_EVENT) || 
					(eventType == SLIDER_PUSH_UP_EVENT)) ? true : false;
	return retVal;
}


/* Returns true if the helicopter is close enough to the current reference
altitude. Used to determine when it's appropriate to further reduce the
reference altitude. Ultimately ensures a smooth landing sequence. */
static bool
isWithinRefMargins (OperatingData_t* programInfo, uint8_t altMarg)
{
	uint32_t tempCurrAlt = programInfo->currentAltDig;
	uint32_t tempRefAltDig = programInfo->referenceAltDig;
	bool altCheckLanded = (tempCurrAlt >= tempRefAltDig - altMarg) 
							&& (tempCurrAlt <= tempRefAltDig + altMarg);

	return ((altCheckLanded == true) ? true : false);
}


/* Returns true when the helicopter is close enough to the recorded ADC
value corresponding to the initial landed position */
static bool
isWithinLandedCriteria (OperatingData_t* programInfo, uint32_t landedAltADCValue)
{
	uint32_t temp = g_landedAltitudeADCValue - NEAR_LANDED_NOISE_MARGIN;
	return ((programInfo->currentAltDig >= temp) ? true : false);
}


/* Updates the flight mode in the program status object on switch events */
static void
updateFlightModeOnSwitchEvent (OperatingData_t* programInfo, hwEvent_t eventType)
{
	// Only update flight mode to flying if in the landed state
	if ((eventType == SLIDER_PUSH_UP_EVENT) && (programInfo->mode == landed))
	{
		programInfo->mode = flying;
	}
	// Only update flight mode to landing if in the flying state
	else if ((eventType == SLIDER_PUSH_DOWN_EVENT) && (programInfo->mode == flying))
	{
		programInfo->mode = landing;
	}
}


/* Updates the flight mode to landed if the helicopter meets the landed criteria */
static void
updateFlightModeIfLanding (OperatingData_t* programInfo)
{
	
	if (programInfo->mode == landing)
	{
		/* If currently landing and has met the landed criteria, then
		set mode to landed */
		if (isWithinLandedCriteria (programInfo, g_landedAltitudeADCValue))
		{
			programInfo->mode = landed;
		}
		/* Otherwise helicopter hasn't quite met landed criteria and 
		the reference altitude should be further decremented */
		else if (isWithinRefMargins (programInfo, NOISE_MARGIN))
		{
			/* Decrement the reference altitude further as part
			of the landing sequence */
			updateProgramStatusRefAlt (programInfo, false);
		}
	}
}


/* The controller task. 
Continuously updates the main motor PWM value and monitors and updates the
flight mode as a function of the slider switch events */
static void
ControllerTask (void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32PollDelay = CONTROLLER_TASK_POLL_DELAY;
	hwEvent_t newQueueItem;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount ();

    // Loop forever
    while (1)
    {
        /* If the initial landed ADC value has been measured, the controller can begin
        normal operation */
        if (g_programStatus.mode == calibrate)
        {
            // If semaphore is available, update the flying mode to landed
            if (xSemaphoreTake (g_calibrationCompleteSemaphore, portMAX_DELAY) == pdTRUE)
            {
                g_programStatus.mode = landed;
                g_programStatus.referenceAltDig = g_landedAltitudeADCValue;
            }
        }
        // Normal operation in the landed, flying and landing modes
        else
        {
            // Update the flight mode if the helicopter is landing
            updateFlightModeIfLanding (&g_programStatus);

            // Read from the queue to see if there is a new switch event
            if (xQueueReceive (g_switchEventQueue, &newQueueItem, 0) == pdPASS)
            {
                /* Only act if new queue item is a switch event (should
                only ever be a switch event here) */
                if (isEventTypeSlider (newQueueItem))
                {
                    // Update flight mode if necessary
                    updateFlightModeOnSwitchEvent (&g_programStatus, newQueueItem);

                    // Optional debug statements
                    #if DEBUG
                    xSemaphoreTake (g_pUARTMutex, portMAX_DELAY);
                    if (newQueueItem == SLIDER_PUSH_DOWN_EVENT)
                    {
                        UARTprintf (downEventString);
                    }
                    else
                    {
                        UARTprintf (upEventString);
                    }
                    xSemaphoreGive (g_pUARTMutex);
                    #endif
                }
            }

            // Update the main motor controller
            updateController (&g_programStatus, &g_altControlData);
        }

		// Wait for the required amount of time.
		vTaskDelayUntil (&ui16LastTime, ui32PollDelay / portTICK_RATE_MS);
	}
}


/* Initializes the controller task and the switch event queue. 
No hardware initialization takes places as this task has no interaction with hardware.
 */
uint32_t 
ControllerTaskInit (void)
{
	
	/* Initialize the switch event queue that is only written to by the hardware event
	reader queue */
	g_switchEventQueue = xQueueCreate (SWITCH_EVENT_QUEUE_SIZE, SWITCH_EVENT_ITEM_SIZE);
	
	/* Initialize the initial altitude value semaphore.
	This binary semaphore is taken by the ADC task during program
	initialisation and is never given back - since the initial
	altitude ADC value is only measured once and only once. */
	// g_initAltADCValueSemaphore = xSemaphoreCreateBinary ();
	
    // Create the buttons switches task
    if (xTaskCreate (ControllerTask, 
					(const portCHAR *)"Controller task", 
					CONTROLLER_TASK_STACK_SIZE, 
					NULL, 
					tskIDLE_PRIORITY + PRIORITY_CONTROLLER_TASK, 
					NULL) != pdTRUE)
    {
        return (1);
    }
	
    UARTprintf ("Controller task initialized.\n");
	
    // Success
    return (0);
}




