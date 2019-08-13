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
// Last modified:  13/08/2019
// 
// *******************************************************

#include "queue_reader.h"
#include <stdint.h>
#include <stdbool.h>
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/uartstdio.h"
#include "sharedConstants.h"

/* FreeRTOS task specific defines */
#define QUEUEREADERTASKSTACKSIZE		128
#define QUEUEREADERTASKPOLLDELAY		25		// Might we have to poll faster than 1kHz or faster than the ADC?

#define HWEVENT_ITEM_SIZE				sizeof(hwEventQueueItem_t)
#define HWEVENT_QUEUE_SIZE				10

/* Queue handle and queue mutex handles which are to be initialized in this module. */
xQueueHandle g_butsADCEventQueue;				// Accessed by the button switch and ADC update tasks
xSemaphoreHandle g_butsADCEventQueueSemaphore;	// Accessed by the button switch and ADC update tasks

extern xSemaphoreHandle g_pUARTSemaphore;
extern operatingData_t g_programStatus;			// Accessed by the queue reader, controller, PWM and UART tasks

 static const char* debugStrings[NUM_HW_EVENT_TYPES] = 
 {
	 "UP_BUTTON_PUSH_EVENT\n",
	 "DOWN_BUTTON_PUSH_EVENT\n",
	 "SLIDER_PUSH_DOWN_EVENT\n",
	 "SLIDER_PUSH_UP_EVENT\n",
	 "ADC_BUFFER_UPDATED_EVENT\n"
 };


/* Returns a capped integer value given an input value to be checked and a given bound value.
If the input value is above the given bound, the bound value is returned instead. 
Only to be used with unsigned 8-bit integer values */
static uint8_t 
ui8CapToGivenBoundType (uint8_t inputValue, uint8_t boundValue, bool isUpperBound)
{
	uint8_t retVal;
	if (isUpperBound)
	{
		retVal = (inputValue > boundValue) ? boundValue : inputValue;
	}
	else
	{
		retVal = (inputValue < boundValue) ? boundValue : inputValue;
	}
	return retVal;
}


/* Returns a capped integer value given an input value to be checked and a given bound value.
If the input value is above the given bound, the bound value is returned instead.
Only to be used with unsigned 32-bit integer values */
static uint32_t
ui32CapToGivenBoundType (uint32_t inputValue, uint32_t boundValue, bool isUpperBound)
{
	uint32_t retVal;
	if (isUpperBound)
	{
		retVal = (inputValue > boundValue) ? boundValue : inputValue;
	}
	else
	{
		retVal = (inputValue < boundValue) ? boundValue : inputValue;
	}
	return retVal;
}


/* Update the reference percentage altitude in the program status object */
static void
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
		if (currRefAltPct < MAX_ALTITUDE_PCT)
		{
			newRefAltPct = ui8CapToGivenBoundType((currRefAltPct + ALTITUDE_INCREMENT_PCT), 
													MAX_ALTITUDE_PCT, true);
		}
		if (currRefAltDig > MAX_ALTITUDE_ADC)
		{
			newRefAltPct = ui32CapToGivenBoundType((currRefAltDig - ALTITUDE_INCREMENT_ADC),
													MAX_ALTITUDE_ADC, false);
		}
		
	} 
	else
	{
		/* Decrease both the percentage and digital representations of the
		reference altitude */
		if (currRefAltPct > MIN_ALTITUDE_PCT)
		{
			newRefAltPct = ui8CapToGivenBoundType((currRefAltPct - ALTITUDE_INCREMENT_PCT),
													MIN_ALTITUDE_PCT, false);
		}
		if (currRefAltDig < MIN_ALTITUDE_ADC)
		{
			newRefAltPct = ui32CapToGivenBoundType((currRefAltDig + ALTITUDE_INCREMENT_ADC),
													MIN_ALTITUDE_ADC, true);
		}
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

/* Updates the  */
static void
updateProgramStatusFlightMode (OperatingData_t *programStatus, hwEvent_t switchEventType)
{
	/* Only update the on switch event if flight mode of the 
	helicopter is not currently landing */
	if (programStatus->mode != landing)
	{
		switch (switchEventType)
		{
		case SLIDER_PUSH_UP_EVENT:
			
			break;
		}
	}
}


/* The hardware event queue task. Continuously loops, reads the event type and updates the helicopter flight program status structure as appropriate */
static void
HWEventQueueReaderTask (void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32PollDelay = QUEUEREADERTASKPOLLDELAY;
	hwEventQueueItem_t newQueueItem;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount();

    // Loop forever
    while (1)
    {
        // Obtain most recent queue item
		if (xQueueReceive(g_butsADCEventQueue, &newQueueItem, 0) == pdPASS)
		{
			// Only act if new queue item represents a valid event
			if (newQueueItem.eventType != INVALID_EVENT_TYPE)
			{
				// Determine which hardware event occurred
				switch (newQueueItem.eventType)
				{
					case UP_BUTTON_PUSH_EVENT:
						// Increment reference altitude
						updateProgramStatusRefAlt (&g_programStatus, true);
						break;
					case DOWN_BUTTON_PUSH_EVENT:
						// Decrement reference altitude
						updateProgramStatusRefAlt (&g_programStatus, false);
					/* Update flight mode if applicable regardless of slider
					switch event type */
					case SLIDER_PUSH_DOWN_EVENT:
					case SLIDER_PUSH_UP_EVENT:
						/* Actually, this needs to write the slider switch event to
						another queue (yet to be implemented). this queue will continuously
						be read from at 1 kHz or so by a new task - the flight_mode_monitor
						task. This will continuously monitor the flight mode so that the
						state can be updated from landing to landed. */
						
						/* REDO */
						// updateProgramStatusFlightMode(&g_programStatus, newQueueItem.eventType)
						/* REDO */
						break;
					case ADC_BUFFER_UPDATED_EVENT:
						// Update current altitude in digital representation
						updateProgramStatusCurAlt (&g_programStatus, newQueueItem.adcBufferAverage);
					default:
						break;
				}
				// Optional debug statements
				#if DEBUG
					xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
					UARTprintf(debugStrings[(uint8_t) newQueueItem.eventType]);
					xSemaphoreGive(g_pUARTSemaphore);
				#endif
			}
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
	g_butsADCEventQueue = xQueueCreate(HWEVENT_QUEUE_SIZE, HWEVENT_ITEM_SIZE);
	
    // Create the buttons switches task
    if (xTaskCreate (HWEventQueueReaderTask, (const portCHAR *)"Hardware event queue reader", QUEUEREADERTASKSTACKSIZE, NULL, tskIDLE_PRIORITY + PRIORITY_HW_EVENT_QUEUE_TASK, NULL) != pdTRUE)
    {
        return (1);
    }
	
    UARTprintf("Hardware event queue initialized.\n");
    // Success
    return (0);
}




