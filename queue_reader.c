// *******************************************************
// 
// buttons_switch_task.c
//
// Support for a set of FOUR specific buttons on the Tiva/Orbit.
// The buttons are:  UP and DOWN (on the Orbit daughterboard) plus
// LEFT and RIGHT on the Tiva.
//
// Note that pin PF0 (the pin for the RIGHT pushbutton - SW2 on
//  the Tiva board) needs special treatment - See PhilsNotesOnTiva.rtf.
//
// Jozef Crosland
// Last modified:  03/08/2019
// 
// *******************************************************

#include "queue_reader.h"
#include <stdint.h>
#include <stdbool.h>
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/uartstdio.h"

/* FreeRTOS task specific defines */
#define QUEUEREADERTASKSTACKSIZE      128
#define QUEUEREADERTASKPOLLDELAY      25

#define HWEVENT_ITEM_SIZE sizeof(HWEventQueueItem_t)
#define HWEVENT_QUEUE_SIZE 10

xQueueHandle g_pButsADCEventQueue;
extern xSemaphoreHandle g_pUARTSemaphore;


/* The hardware event queue task. Continuously loops, reads the event type and updates the helicopter flight program status structure as appropriate */
static void
QueueOnWritingHandlerTask (void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32PollDelay = 25;
    butEvents_t eventMessage;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount();

    // Loop forever
    while (1)
    {
        // Do queue reading
		
		// Update the program status information
		
        // Wait for the required amount of time. Delay is probably not necessary
        vTaskDelayUntil (&ui16LastTime, ui32PollDelay / portTICK_RATE_MS);
    }
    
}

/* Initializes queue in this module and the task. No hardware initialization takes places as this task has no interaction with hardware */
uint32_t 
QueueOnWritingHandlerTaskInit (void)
{
	
	// Initialise the hardware event queue
	g_pButsADCEventQueue = xQueueCreate(HWEVENT_QUEUE_SIZE, HWEVENT_ITEM_SIZE);
	
    // Create the buttons switches task
    if (xTaskCreate (QueueOnWritingHandlerTask, (const portCHAR *)"Hardware Event Queue Task", QUEUEREADERTASKSTACKSIZE, NULL, tskIDLE_PRIORITY + PRIORITY_HW_EVENT_QUEUE_TASK, NULL) != pdTRUE)
    {
        return (1);
    }
    UARTprintf("Hardware event queue initialized.\n");
    // Success
    return (0);
}




