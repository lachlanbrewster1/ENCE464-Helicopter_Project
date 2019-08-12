#ifndef QUEUE_READER_H_
#define QUEUE_READER_H_

// *******************************************************
// buttons_switch_task.h
//
// Support for a set of two buttons and one switch used on the Orbit
// daughterboard. Also has the buttons and switch FreeRTOS task.
// The buttons are:  UP and DOWN (on the Orbit daughterboard)
//
// Jozef Richard Crosland
// Last modified:  05.08.2019
// 
// *******************************************************

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Constants
//*****************************************************************************


extern uint32_t QueueOnWritingHandlerTaskInit (void); 

#endif /*QUEUE_READER_H*/
