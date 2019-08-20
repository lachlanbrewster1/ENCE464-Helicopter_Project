/*
 * adcTask.h
 *
 *  Created on: 1/08/2019
 *      Author: lbr63
 */

#ifndef ADCQUEUETASK_H_
#define ADCQUEUETASK_H_

// #define BUF_SIZE 100


//*****************************************************************************
// Initializes the ADC Queue task.
//*****************************************************************************
uint32_t
adcQueueTaskInit(void);

//*****************************************************************************
// This task handles ADC for the helirig, constantly monitoring the height of the rig,
// storing the received values in a circular buffer
//*****************************************************************************
static void
adcQueueTask(void *pvParameters);

//*****************************************************************************
// Calculate and return the rounded mean of the buffer contents
//*****************************************************************************
uint32_t
calculateMeanHeight(void);



#endif /* ADCQUEUETASK_H_ */
