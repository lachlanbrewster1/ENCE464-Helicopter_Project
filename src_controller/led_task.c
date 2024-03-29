//*****************************************************************************
//
// led_task.c - A simple flashing LED task.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/rgb.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"
#include "led_task.h"
#include "buttons_switch_task.h"
#include "queue_reader.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//*****************************************************************************
//
// The stack size for the LED toggle task.
//
//*****************************************************************************
#define LEDTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// The item size and queue size for the LED message queue.
//
//*****************************************************************************
#define LED_ITEM_SIZE           sizeof(hwEventQueueItem_t)
#define LED_QUEUE_SIZE          5

//*****************************************************************************
//
// Default LED toggle delay value. LED toggling frequency is twice this number.
//
//*****************************************************************************
#define LED_TOGGLE_DELAY        250
#define LED_OFF_HEX_VALUE       0x0000
#define LED_ON_HEX_VALUE        0x8000

//*****************************************************************************
//
// The queue that holds messages sent to the LED task.
//
//*****************************************************************************
xQueueHandle g_pLEDQueue;

//
// [G, R, B] range is 0 to 0xFFFF per color.
//
typedef enum rgb_led_index_e {
    RED_INDEX,
    GREEN_INDEX,
    BLUE_INDEX
} rgbLEDIndex_t;

static uint32_t g_pui32Colors[3] = { LED_OFF_HEX_VALUE };
static uint8_t g_ui8ColorsIndx;

extern xSemaphoreHandle g_pUARTMutex;

//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************
static void
LEDTask(void *pvParameters)
{
    portTickType ui32WakeTime;
    uint32_t ui32LEDToggleDelay;
    hwEventQueueItem_t queueEventType;

    //
    // Initialize the LED Toggle Delay to default value.
    //
    ui32LEDToggleDelay = LED_TOGGLE_DELAY;

    //
    // Get the current tick count.
    //
    ui32WakeTime = xTaskGetTickCount();

    //
    // Loop forever.
    //
    while(1)
    {
        //
        // Read the next message, if available on queue.
        //
        if(xQueueReceive(g_pLEDQueue, &queueEventType, 0) == pdPASS)
        {
            //
            // If up button, update to next LED.
            //
            if(queueEventType.buttonADCEventType == UP_BUTTON_PUSH_EVENT)
            {

                // Update the LED buffer to turn off represent disabling the red LED and turning on the blue LED
                g_pui32Colors[RED_INDEX] = LED_OFF_HEX_VALUE;
                g_pui32Colors[BLUE_INDEX] = LED_ON_HEX_VALUE;
                g_pui32Colors[GREEN_INDEX] = LED_OFF_HEX_VALUE;


                // Configure the new LED settings.
                RGBColorSet(g_pui32Colors);
            }

            // If down button, update delay time between toggles of led.
            else if(queueEventType.buttonADCEventType == DOWN_BUTTON_PUSH_EVENT)
            {
                // Update the LED buffer to turn off represent disabling the blue LED and turning on the red LED
                g_pui32Colors[RED_INDEX] = LED_ON_HEX_VALUE;
                g_pui32Colors[BLUE_INDEX] = LED_OFF_HEX_VALUE;
                g_pui32Colors[GREEN_INDEX] = LED_OFF_HEX_VALUE;

                // Configure the new LED settings.
                RGBColorSet(g_pui32Colors);

            }

            else if (queueEventType.buttonADCEventType == UP_AND_DOWN_BUTTON_PUSH_EVENT)
            {
                // Update the LED buffer to go purple
                g_pui32Colors[RED_INDEX] = LED_ON_HEX_VALUE;
                g_pui32Colors[BLUE_INDEX] = LED_ON_HEX_VALUE;
                g_pui32Colors[GREEN_INDEX] = LED_OFF_HEX_VALUE;

                // Configure the new LED settings.
                RGBColorSet(g_pui32Colors);
            }

            if (queueEventType.switchEventType == SLIDER_PUSH_UP_EVENT)
            {
                // Update the LED buffer to represent a cyan LED
                g_pui32Colors[RED_INDEX] = LED_OFF_HEX_VALUE;
                g_pui32Colors[BLUE_INDEX] = LED_ON_HEX_VALUE;
                g_pui32Colors[GREEN_INDEX] = LED_ON_HEX_VALUE;

                // Configure the new LED settings.
                RGBColorSet(g_pui32Colors);
            }
            else if (queueEventType.switchEventType == SLIDER_PUSH_DOWN_EVENT)
            {
                // Update the LED buffer to represent a green LED
                g_pui32Colors[RED_INDEX] = LED_OFF_HEX_VALUE;
                g_pui32Colors[BLUE_INDEX] = LED_OFF_HEX_VALUE;
                g_pui32Colors[GREEN_INDEX] = LED_ON_HEX_VALUE;

                // Configure the new LED settings.
                RGBColorSet(g_pui32Colors);
            }
        }

        //
        // Turn on the LED.
        //
        RGBEnable();

        //
        // Wait for the required amount of time.
        //
        vTaskDelayUntil(&ui32WakeTime, ui32LEDToggleDelay / portTICK_RATE_MS);

        //
        // Turn off the LED.
        //
        RGBDisable();

        //
        // Wait for the required amount of time.
        //
        vTaskDelayUntil(&ui32WakeTime, ui32LEDToggleDelay / portTICK_RATE_MS);
    }
}

//*****************************************************************************
//
// Initializes the LED task.
//
//*****************************************************************************
uint32_t
LEDTaskInit(void)
{
    //
    // Initialize the GPIOs and Timers that drive the three LEDs.
    //
    RGBInit(1);
    RGBIntensitySet(0.3f);

    //
    // Turn on the Green LED
    //
    g_ui8ColorsIndx = 0;
    g_pui32Colors[g_ui8ColorsIndx] = 0x8000;
    RGBColorSet(g_pui32Colors);

    //
    // Print the current loggling LED and frequency.
    //
    UARTprintf("\nLed %d is blinking. [R, G, B]\n", g_ui8ColorsIndx);
    UARTprintf("Led blinking frequency is %d ms.\n", (LED_TOGGLE_DELAY * 2));

    //
    // Create a queue for sending messages to the LED task.
    //
    g_pLEDQueue = xQueueCreate(LED_QUEUE_SIZE, LED_ITEM_SIZE);

    //
    // Create the LED task.
    //
    if(xTaskCreate(LEDTask, (const portCHAR *)"LED", LEDTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_LED_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}
