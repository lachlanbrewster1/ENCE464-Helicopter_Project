// *******************************************************
//
// adcTriggerTask.c
//
// Definition of the ADC trigger task. This task triggers a ADC conversion,
// then when the ADC value is ready, sets a flag signalling as such to the
// ADC queue task.
//
// Author: Lachlan Brewster
// Last modified:  23/08/2019
//
// *******************************************************


#ifndef ADCTRIGGERTASK_H_
#define ADCTRIGGERTASK_H_

//*****************************************************************************
// Initializes the ADC task.
//*****************************************************************************
uint32_t
adcTriggerTaskInit(void);

//*****************************************************************************
// This task handles the constant triggering of the ADC conversion, which then triggers
// an ISR which sets a flag/semaphore signalling that a ADC value is available
//*****************************************************************************
static void
adcTriggerTask(void *pvParameters);

//*****************************************************************************
// The handler for the ADC conversion complete interrupt.
// Gives the ADC convert semaphore, signalling that a ADC value is available
//*****************************************************************************
void ADCIntHandler(void);

//*****************************************************************************
// Initialisation functions for the ADC
//*****************************************************************************
void initADC (void);

#endif /* ADCTRIGGERTASK_H_ */
