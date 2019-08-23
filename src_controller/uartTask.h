// *******************************************************
//
// uartTask.c
//
// Definition of the UART task. This task sends important, real
// time information about the heli righ to serial communications
//
// Author: Lachlan Brewster
// Last modified:  23/08/2019
//
// *******************************************************

#ifndef UARTTASK_H_
#define UARTTASK_H_


//*****************************************************************************
// Initializes the UART task.
//*****************************************************************************
uint32_t
uartTaskInit(void);

//*****************************************************************************
// This task handles the UART communications, sending current info about the
// helirig to serial communications. Sends the following info:
// Heli mode, current altitude %, reference altitude %, PWM duty cycle
//*****************************************************************************
static void
uartTask(void *pvParameters);



#endif /* UARTTASK_H_ */
