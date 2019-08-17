#ifndef CONTROLLER_TASK_H_
#define CONTROLLER_TASK_H_

// *******************************************************
//
// controller_task.c
//
// Procedures that implement PID control for the helicopter
// program
//
// Author: Jozef Crosland | jrc149 | 49782422
// Last modified:  14/08/2019
//
// *******************************************************

#include <stdint.h>

//Structure to hold the gains; lets us switch between them on the
// fly if we want to abstract up a level (for piecewise).
// Also lets us hold infomation related to a single controller seperate
// One for each PID controller.


typedef struct control_data_t_s {
	float pastError;
	float intError;
	float dt;               // TO BE DECIDED
	float kp;
	float kd;
	float ki;
} ControlData_t;

extern uint32_t ControllerTaskInit (void); 

#endif /* CONTROLLER_TASK_H_ */
