#ifndef CONTROLLER_TASK_H_
#define CONTROLLER_TASK_H_

// *******************************************************
//
// controller_task.h
//
// Control data structure definitions
//
// Author: Jozef Crosland | jrc149 | 49782422
// Last modified:  23/08/2019
//
// *******************************************************

#include <stdint.h>

//Structure to hold the gains; lets us switch between them on the
// fly if we want to abstract up a level (for piecewise).
// Also lets us hold infomation related to a single controller seperate
// One for each PID controller.


typedef struct control_data_t_s {
	float pastError; 	// Past error used for the integral error component
	float intError;		// Current integral error
	float dt;			// Time step             	
	float kp;			// Proportional gain
	float kd;			// Derivative gain
	float ki;			// Integral gain
} ControlData_t;

extern uint32_t 
ControllerTaskInit (void); 

#endif /* CONTROLLER_TASK_H_ */
