#ifndef QUEUE_READER_H_
#define QUEUE_READER_H_

// *******************************************************
//
// queue_reader.h
//
// Definition of the hardware event queue reader task. This
// task reads the queue at a fixed frequency and updates the
// program flight status.
//
//
// Author: Jozef Crosland | jrc149 | 49782422
// Last modified:  23/08/2019
//
// *******************************************************


#include <stdint.h>
#include "sharedConstants.h"


/* Enumerated type identifying each of the hardware event
types - button event, switch event or ADC event */
typedef enum hw_evt_type_e
{
	INVALID_EVENT_TYPE = -1,
	NO_HW_EVENT,
	UP_BUTTON_PUSH_EVENT,
	DOWN_BUTTON_PUSH_EVENT,
	UP_AND_DOWN_BUTTON_PUSH_EVENT,
	SLIDER_PUSH_DOWN_EVENT,
	SLIDER_PUSH_UP_EVENT,
	ADC_BUFFER_UPDATED_EVENT,
	NUM_VALID_HW_EVENT_TYPES,
	NUM_ALL_HW_EVENT_TYPES
} hwEvent_t;


/* Structure that contains the hardware event type and the
most recent ADC buffer average */
typedef struct hw_evt_queue_item_e
{
	hwEvent_t buttonADCEventType;
	hwEvent_t switchEventType;
	uint32_t adcBufferAverage;
} hwEventQueueItem_t;


/* Update the reference percentage altitude in the program status object.
NOT STATIC - this function is also used by the controller task. */
void
updateProgramStatusRefAlt (OperatingData_t *programStatus, bool doIncrease);


extern uint32_t
HWEventQueueReaderTaskInit (void);

#endif /*QUEUE_READER_H*/
