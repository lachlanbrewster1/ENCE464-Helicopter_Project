// *******************************************************
//
// adcQueueTask.c
//
// Definition of the ADC queue task. This task populates the circular
// buffer with ADC values, and calculates the average of the buffer and
// sends it to the event queue to be processed
//
// Author: Lachlan Brewster
// Last modified:  23/08/2019
//
// *******************************************************



#ifndef ADCQUEUETASK_H_
#define ADCQUEUETASK_H_

// #define BUF_SIZE 100


//*****************************************************************************
// Initializes the ADC Queue task.
//*****************************************************************************
uint32_t
adcQueueTaskInit(void);

//*****************************************************************************
// This task handles calculating, then sending the current average ADC value
// stored in the circular buffer to the program status through use of the event queue
//*****************************************************************************
static void
adcQueueTask(void *pvParameters);

//*****************************************************************************
// Calculate and return the rounded mean of the buffer contents
//*****************************************************************************
uint32_t
calculateMeanHeight(void);



#endif /* ADCQUEUETASK_H_ */
