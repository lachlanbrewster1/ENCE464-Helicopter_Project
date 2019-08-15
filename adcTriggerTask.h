/*
 * adcTask.h
 *
 *  Created on: 1/08/2019
 *      Author: lbr63
 */

#ifndef ADCTRIGGERTASK_H_
#define ADCTRIGGERTASK_H_

#define BUF_SIZE 100


//*****************************************************************************
// Initializes the ADC task.
//*****************************************************************************
uint32_t
adcTriggerTaskInit(void);

//*****************************************************************************
// This task handles ADC for the helirig, constantly monitoring the height of the rig,
// storing the received values in a circular buffer
//*****************************************************************************
static void
adcTriggerTask(void *pvParameters);


//*****************************************************************************
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//*****************************************************************************
void ADCIntHandler(void);

//*****************************************************************************
// Initialisation functions for the ADC
//*****************************************************************************
void initADC (void);

//*****************************************************************************
// Set the landed reference of the helicopter.
// Uses the rounded mean of the circular buffer contents
//*****************************************************************************
void
setLandedRef(void);


#endif /* ADCTRIGGERTASK_H_ */
