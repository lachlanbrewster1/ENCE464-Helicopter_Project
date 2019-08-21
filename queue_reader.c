// *******************************************************
// 
// queue_reader.c
//
// Definition of the hardware event queue reader task. This
// task reads the queue at a fixed frequency and updates the
// program flight status.
//
//
// Author: Jozef Crosland | jrc149 | 49782422
// Last modified:  14/08/2019
// 
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "queue_reader.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/uartstdio.h"
#include "sharedConstants.h"


/* FreeRTOS task specific defines */
#define QUEUEREADERTASKSTACKSIZE		128
#define QUEUEREADERTASKPOLLDELAY		2.5		// Might we have to poll faster than 1kHz or faster than the ADC?
#define HWEVENT_ITEM_SIZE				sizeof (hwEventQueueItem_t)
#define HWEVENT_QUEUE_SIZE				10


/* Queue handle and queue mutex handles which are to be initialized in this module. */
xQueueHandle g_buttsADCEventQueue;				// Accessed by the button switch and ADC update tasks and queue reader


/* Externally defined global variables, both FreeRTOS-specific and helicopter program specific */
extern xQueueHandle g_switchEventQueue;			// Accessed by the queue reader and controller tasks
extern xSemaphoreHandle g_pUARTMutex;		// Accessed by most tasks
extern OperatingData_t g_programStatus;			// Accessed by the queue reader, controller, PWM and UART tasks

extern uint32_t g_fullAltitudeADCValue;
extern uint32_t g_landedAltitudeADCValue;


/* Debug strings used to print to serial in debug version of the executable */
#ifdef DEBUG
static const char* debugStrings[SLIDER_PUSH_UP_EVENT + 1] =
{
	"INVALID_EVENT_TYPE\n",
	"NO_HW_EVENT\n",
	"UP_BUTTON_PUSH_EVENT\n",
	"DOWN_BUTTON_PUSH_EVENT\n",
	"UP_AND_DOWN_BUTTON_PUSH_EVENT\n",
	"SLIDER_PUSH_DOWN_EVENT\n",
	"SLIDER_PUSH_UP_EVENT\n"
};
#endif


/* Returns a capped integer value given an input value to be checked and a given bound value.
If the input value is above the given bound, the bound value is returned instead. 
This function can accept input and bound values between -128 and +127 before integer
overflow becomes a problem. This function has no safeguarding mechanism if a parameter
too large in magnitude is passed in. */
static uint8_t 
ui8CapToGivenBoundType (int8_t inputValue, int8_t boundValue, bool isUpperBound)
{
	int8_t retVal;
	if (isUpperBound)
	{
		retVal = (inputValue > boundValue) ? boundValue : inputValue;
	}
	else
	{
		retVal = (inputValue < boundValue) ? boundValue : inputValue;
	}
	return (uint8_t)retVal;
}


/* Returns a capped integer value given an input value to be checked and a given bound value.
If the input value is above the given bound, the bound value is returned instead.
This function can accept input and bound values between -128 and +127 before integer
overflow becomes a problem. This function has no safeguarding mechanism if a parameter
too large in magnitude is passed in. */
static uint32_t
ui32CapToGivenBoundType (int32_t inputValue, int32_t boundValue, bool isUpperBound)
{
	int32_t retVal;
	if (isUpperBound)
	{
		retVal = (inputValue > boundValue) ? boundValue : inputValue;
	}
	else
	{
		retVal = (inputValue < boundValue) ? boundValue : inputValue;
	}
	return (uint32_t)retVal;
}


/* Update the reference percentage altitude in the program status object.
 * NOT STATIC - this needs to be visible to the controller module */
void
updateProgramStatusRefAlt (OperatingData_t *programStatus, bool doIncrease)
{
	uint8_t currRefAltPct = programStatus->referenceAltPercent;
	uint32_t currRefAltDig = programStatus->referenceAltDig;
	uint8_t newRefAltPct;
	uint32_t newRefAltDig;
	
	/* Note that digital representation of the altitude is decremented when the 
	reference altitude increases, and is incremented when the reference altitude
	increases. This is the opposite of the percentage altitude reference and 
	occurs because the height sensor's output voltage decreases with increasing
	helicopter altitude */
	if (doIncrease)
	{
		/* Increase the percentage altitude by 10% and the decrement digital 
		representation by the corresponding amount - both are capped to their
		respective limits */
		newRefAltPct = ui8CapToGivenBoundType ((currRefAltPct + ALTITUDE_INCREMENT_PCT), 
												MAX_ALTITUDE_PCT, true);
		newRefAltDig = ui32CapToGivenBoundType ((currRefAltDig - ALTITUDE_INCREMENT_ADC),
		                                        g_fullAltitudeADCValue, false);
	} 
	else
	{
		/* Decrease both the percentage and digital representations of the
		reference altitude */
		newRefAltPct = ui8CapToGivenBoundType ((currRefAltPct - ALTITUDE_INCREMENT_PCT),
												MIN_ALTITUDE_PCT, false);
		newRefAltDig = ui32CapToGivenBoundType ((currRefAltDig + ALTITUDE_INCREMENT_ADC),
		                                        g_landedAltitudeADCValue, true);
	}
	// Update reference altitude members of structure
	programStatus->referenceAltPercent = newRefAltPct;
	programStatus->referenceAltDig = newRefAltDig;
}


/* Checks that the updated ADC value representing the height is valid and within the
expected bounds */
static void
updateProgramStatusCurAlt (OperatingData_t *programStatus, uint32_t newADCResult)
{
	// Only a weak check. If the ADC value was 4095, something has gone wrong with the rig.
	if (newADCResult <= MAX_ADC_VALUE)
	{
		programStatus->currentAltDig = newADCResult;
	}
}


/* The hardware event queue task. 
Continuously loops, reads the event type and updates the 
helicopter flight program status structure as appropriate */
static void
HWEventQueueReaderTask (void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32PollDelay = QUEUEREADERTASKPOLLDELAY;
	hwEventQueueItem_t newQueueItem;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount ();

    // Loop forever
    while (1)
    {
        // Obtain most recent queue item
		if (xQueueReceive (g_buttsADCEventQueue, &newQueueItem, 0) == pdPASS)
		{
			/* Check for button events and switch events seperately */
			// Checking for button events here
			if (newQueueItem.buttonADCEventType != INVALID_EVENT_TYPE)
			{
				// Determine which hardware event occurred
				switch (newQueueItem.buttonADCEventType)
				{
				case UP_BUTTON_PUSH_EVENT:
					// Increment reference altitude
					updateProgramStatusRefAlt (&g_programStatus, true);
					break;
				case DOWN_BUTTON_PUSH_EVENT:
					// Decrement reference altitude
					updateProgramStatusRefAlt (&g_programStatus, false);
					break;
				case ADC_BUFFER_UPDATED_EVENT:
					// Update current altitude in digital representation
					updateProgramStatusCurAlt (&g_programStatus, newQueueItem.adcBufferAverage);
					break;
				default:
					// In case other type of event, do nothing
					break;
				}
			}
			// Checking for switch events here
			if (newQueueItem.switchEventType != INVALID_EVENT_TYPE)
			{
				switch (newQueueItem.switchEventType)
				{
				/* Append switch event to the switch event queue */
				case SLIDER_PUSH_DOWN_EVENT:
				case SLIDER_PUSH_UP_EVENT:
					if (xQueueSend (g_switchEventQueue, &(newQueueItem.switchEventType), portMAX_DELAY) != pdPASS)
					{
						// Queue is full - not good. Should never happen
						xSemaphoreTake (g_pUARTMutex, portMAX_DELAY);
						UARTprintf ("\nSwitch event queue full. This should never happen.\n");
						xSemaphoreGive (g_pUARTMutex);
						while (1)
						{
							// Infinite loop
						}
					}
					break;
				default:
					// For other types of events, do nothing. This should not occur
					break;
				}
				
			}
				// Optional debug statements
				#if DEBUG
				/* ADC buffer would update at fast rate, so the serial terminal
				would be overloaded with print messages, so only print for other
				hardware event types */
				if (newQueueItem.eventType < ADC_BUFFER_UPDATED_EVENT)
				{
					xSemaphoreTake (g_pUARTMutex, portMAX_DELAY);
					// Index offset by one since one enum value is negative
					uint8_t index = ((uint8_t) newQueueItem.eventType) + 1;
					UARTprintf (debugStrings[index]);
					xSemaphoreGive (g_pUARTMutex);
				}
				#endif
			}
		// Wait for the required amount of time. Delay is probably not necessary
		vTaskDelayUntil (&ui16LastTime, ui32PollDelay / portTICK_RATE_MS);
		}
}


/* Initializes queue in this module and the task. No hardware initialization takes places as this task has no interaction with hardware */
uint32_t
HWEventQueueReaderTaskInit (void)
{
	
	// Initialise the hardware event queue
	g_buttsADCEventQueue = xQueueCreate (HWEVENT_QUEUE_SIZE, HWEVENT_ITEM_SIZE);
	
    // Create the buttons switches task
    if (xTaskCreate (HWEventQueueReaderTask, 
					(const portCHAR *)"Hardware event queue reader", 
					QUEUEREADERTASKSTACKSIZE, 
					NULL, 
					tskIDLE_PRIORITY + PRIORITY_HW_EVENT_QUEUE_TASK, 
					NULL) != pdTRUE)
    {
        return (1);
    }
	
    UARTprintf ("Hardware event task initialized.\n");
    // Success
    return (0);
}




