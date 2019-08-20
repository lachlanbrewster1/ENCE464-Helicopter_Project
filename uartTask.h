/*
 * uartTask.h
 *
 *  Created on: 1/08/2019
 *      Author: lbr63
 */

#ifndef UARTTASK_H_
#define UARTTASK_H_


//*****************************************************************************
// Initializes the UART task.
//*****************************************************************************
uint32_t
uartTaskInit(void);

//*****************************************************************************
// This task handles the UART communications, sending current info about the helirig
//*****************************************************************************
static void
uartTask(void *pvParameters);



#endif /* UARTTASK_H_ */
