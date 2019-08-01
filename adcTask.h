/*
 * adcTask.h
 *
 *  Created on: 1/08/2019
 *      Author: lbr63
 */

#ifndef ADCTASK_H_
#define ADCTASK_H_

#define BUF_SIZE 100

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
// Calculate and return the rounded mean of the buffer contents
//*****************************************************************************
uint16_t
calculateMeanHeight(void);

//*****************************************************************************
// Set the landed reference of the helicopter.
// Uses the rounded mean of the circular buffer contents
//*****************************************************************************
void
setLandedRef(void);


#endif /* ADCTASK_H_ */
