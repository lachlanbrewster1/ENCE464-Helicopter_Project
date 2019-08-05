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

#include "buttons_switch_task.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "inc/tm4c123gh6pm.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/uartstdio.h"

/* FreeRTOS task specific defines */
#define BUTTONSSWITCHTASKSTACKSIZE      128
#define BUTTONSSWITCHTASKPOLLDELAY      10
extern xQueueHandle g_pLEDQueue;
extern xSemaphoreHandle g_pUARTSemaphore;

/* Global, module-specific, non-FreeRTOS defines */

static bool but_state[NUM_BUTS];	// Corresponds to the electrical state
static uint8_t but_count[NUM_BUTS];
static bool but_flag[NUM_BUTS];
static bool but_normal[NUM_BUTS];   // Corresponds to the electrical state

/* Tiva board up button object instance */
static buttonSwitch_t g_up_button = 
{
    .tiva_peripheral_base = UP_BUT_PERIPH,
    .tiva_gpio_port = UP_BUT_PORT_BASE,
    .tiva_gpio_pin = UP_BUT_PIN,
    .tiva_gpio_strength = UP_BUT_GPIO_STRENGTH,
    .tiva_gpio_pin_type = UP_BUT_GPIO_TYPE,
    .is_button = true,
    .current_logic_level = false,
    .is_active_high = UP_BUT_ACTIVE_HIGH,
    .current_button_state = false,
    .but_deb_count = 0,
    .but_evt_state = NO_CHANGE
};

/* Tiva board down button object instance */
static buttonSwitch_t g_down_button = 
{
    .tiva_peripheral_base = DOWN_BUT_PERIPH,
    .tiva_gpio_port = DOWN_BUT_PORT_BASE,
    .tiva_gpio_pin = DOWN_BUT_PIN,
    .tiva_gpio_strength = DOWN_BUT_GPIO_STRENGTH,
    .tiva_gpio_pin_type = DOWN_BUT_GPIO_TYPE,
    .is_button = true,
    .current_logic_level = false,
    .is_active_high = UP_BUT_ACTIVE_HIGH,
    .current_button_state = false,
    .but_deb_count = 0,
    .but_evt_state = NO_CHANGE
};

/* Orbit OLED daughter board slider switch object instance */
static buttonSwitch_t g_slider_switch_one = 
{
    .tiva_peripheral_base = SLIDER_ONE_PERIPH,
    .tiva_gpio_port = SLIDER_ONE_PORT_BASE,
    .tiva_gpio_pin = SLIDER_ONE_PIN,
    .tiva_gpio_strength = SLIDER_ONE_GPIO_STRENGTH,
    .tiva_gpio_pin_type = SLIDER_ONE_GPIO_TYPE,
    .is_button = false,
    .current_logic_level = false,
    .is_active_high = NA_IS_SWITCH,
    .current_button_state = IS_SWITCH,
    /* Can ignore the debouncing count and event fields as this is a switch instance */
};

/*
Given a button switch type object, the necessary peripheral, port and pin is initialised and enabled. 
*/
void
initButtonSwitchObj (buttonSwitch_t *but_sw_obj)
{
    // Enable the peripheral and configure the port and pin
    SysCtlPeripheralEnable (but_sw_obj->tiva_peripheral_base);
    GPIOPinTypeGPIOInput (but_sw_obj->tiva_gpio_port, but_sw_obj->tiva_gpio_pin); 
    GPIOPadConfigSet (but_sw_obj->tiva_gpio_port, 
                        but_sw_obj->tiva_gpio_pin, 
                        but_sw_obj->tiva_gpio_strength,
                        but_sw_obj->tiva_gpio_pin_type);
}

/* 
Initialises all of the globally defined buttons and switches for this module 
*/
void
initAllButtonSwitchObjs (void)
{
    // Manually call initialiser, but can we do this iteratively eventually?
    initButtonSwitchObj (&g_up_button);
    initButtonSwitchObj (&g_down_button);
    initButtonSwitchObj (&g_slider_switch_one);
}

/*
 Returns the current button event status of the button object
 */
butStates_t
getButtonEventState (buttonSwitch_t *but_obj)
{
    return (but_obj->but_evt_state);
}

/*
Determines whether a button event is a pushed one or a released event.
This is only called if the polls pass the debounce threshold
 */
butStates_t
updateButtonEventState (buttonSwitch_t *but_obj)
{
    // Assume it's release unless otherwise
    butStates_t ret_val = RELEASED;
    if (but_obj->current_button_state == but_obj->is_active_high)
    {
        ret_val = PUSHED;
    }
    return ret_val;
}

/* Reads in the logic level of the button object passed in regardless of whether it's button or switch instance.
If it's a switch instance, only the logic level of the pin is read and updated. Otherwise, debouncing is performed on the button instance and the button event is updated if in fact it has been pressed for long enough or if it has been released */
void 
updateButtonSwitchObj (buttonSwitch_t *but_sw_obj)
{
    /* Check the logic level regardless of whether it's a button or switch instance */ 
    but_sw_obj->current_logic_level = GPIOPinRead (but_sw_obj->tiva_gpio_port, 
                                                    but_sw_obj->tiva_gpio_pin);
    // Only do debouncing if it's a button instance
    if (but_sw_obj->is_button)
    {
        if (but_sw_obj->current_logic_level != but_sw_obj->current_button_state)
        {
            // Increment debounce count
            but_sw_obj->but_deb_count++;
            /* If the count exceeds threshold, update the current button state and reset the debounce count. Also update whether it's push or release event */
            if (but_sw_obj->but_deb_count >= NUM_BUT_POLLS)
            {
                but_sw_obj->current_button_state = but_sw_obj->current_logic_level;
                but_sw_obj->but_deb_count = 0;
                // Set whether it's a released or pushed button event
                but_sw_obj->but_evt_state = updateButtonEventState (but_sw_obj);
            }
        }
        else
        {
            // Reset button debounce count and explicit set no change event
            but_sw_obj->but_deb_count = 0;
            but_sw_obj->but_evt_state = NO_CHANGE;
        }
        
    }
    
}

/* Updates all of the switches and buttons as defined in this module */
void
updateAllButtonSwitchObjs (void)
{
    updateButtonSwitchObj (&g_up_button);
    updateButtonSwitchObj (&g_down_button);
    updateButtonSwitchObj (&g_slider_switch_one);
}

/* The buttons and switches task. This polls and debounces whatever buttons are configured, and sends a message on the queue indicating which button has been pressed. Also prints to UART */
static void
ButtonsSwitchTask (void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32PollDelay = BUTTONSSWITCHTASKPOLLDELAY;
    uint8_t ui8MessageOne, ui8MessageTwo;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount();

    // Loop forever
    while (1)
    {
        // Poll all buttons and update their button event statuses
        updateAllButtonSwitchObjs();
        /* If the up button was pressed, print to UART and append message to queue */
        if (getButtonEventState(&g_up_button) == PUSHED)
        {
            ui8MessageOne = UP_BUTTON;
            // Guard UART from concurrent access
            xSemaphoreTake (g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf ("Up button is pressed.\n");
            xSemaphoreGive (g_pUARTSemaphore);
        }
        /* If the up button was pressed, print to UART and append message to queue */
        if (getButtonEventState(&g_down_button) == PUSHED)
        {
            ui8MessageOne = DOWN_BUTTON;
            // Guard UART from concurrent access
            xSemaphoreTake (g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf ("Down button is pressed.\n");
            xSemaphoreGive (g_pUARTSemaphore);
        }
        if (xQueueSend (g_pLEDQueue, &ui8MessageOne, portMAX_DELAY) != pdPASS)
        {
            // Oh shit, the queue is full. Disastrous. Should never happen.
            UARTprintf("\nQueue full. This should never happen.\n");
            while(1)
            {
                // Infinite loop
            }
        }
        if (xQueueSend (g_pLEDQueue, &ui8MessageTwo, portMAX_DELAY) != pdPASS)
        {
            // Oh shit, the queue is full. Disastrous. Should never happen.
            UARTprintf("\nQueue full. This should never happen.\n");
            while(1)
            {
                // Infinite loop
            }
        }
        // Wait for the required amount of time
        vTaskDelayUntil (&ui16LastTime, ui32PollDelay / portTICK_RATE_MS);
    }
    
}

/* Initialises the peripherals, ports and pins used by the specified buttons and switches in this module */
uint32_t ButtonsSwitchTaskInit (void)
{
    /* Initialise all of the peripherals, ports and pins used by the switches and buttons */
    initAllButtonSwitchObjs ();
    // Create the buttons switches task
    if (xTaskCreate (ButtonsSwitchTask, (const portCHAR *)"Buttons Switch", BUTTONSSWITCHTASKSTACKSIZE, NULL, tskIDLE_PRIORITY + PRIORITY_BUTTONS_SWITCH_TASK, NULL) != pdTRUE)
    {
        return (1);
    }
    // Success
    return (0);
}




