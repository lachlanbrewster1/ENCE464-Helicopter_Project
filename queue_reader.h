#ifndef QUEUE_READER_H_
#define QUEUE_READER_H_

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

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Constants
//*****************************************************************************
typedef enum hw_evt_type_e
{
	INVALID_EVENT_TYPE = -1,
	UP_BUTTON_PUSH_EVENT,
	DOWN_BUTTON_PUSH_EVENT,
	SLIDER_PUSH_DOWN_EVENT,
	SLIDER_PUSH_UP_EVENT,
	ADC_BUFFER_UPDATED_EVENT,
	NUM_HW_EVENT_TYPES
} hwEvent_t;


typedef struct hw_evt_queue_item_e
{
	hwEvent_t eventType;
	uint32_t adcBufferAverage;
} hwEventQueueItem_t;


extern uint32_t HWEventQueueReaderTaskInit (void); 

#endif /*QUEUE_READER_H*/
