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

/* Put stdint and stdbool includes before everything else */
#include <stdint.h>
#include <stdbool.h>

/* Custom application includes */
#include "buttons_switch_task.h"
#include "queue_reader.h"

/* Tivaware hardware pertinent includes */
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "inc/tm4c123gh6pm.h"

/* FreeRTOS includes */
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/uartstdio.h"

/* FreeRTOS task specific defines */
#define BUTTONSSWITCHTASKSTACKSIZE      128
#define BUTTONSSWITCHTASKPOLLDELAY      10
extern xQueueHandle g_butsADCEventQueue;
extern xQueueHandle g_pLEDQueue;
extern xSemaphoreHandle g_pUARTSemaphore;

/* Global, module-specific, non-FreeRTOS defines */

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
    .but_evt_flag = false,
    .but_deb_count = 0,
    .but_evt_state = NO_CHANGE
    /* Can ignore the switch event field as this is a button instance */
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
    .but_evt_flag = false,
    .but_deb_count = 0,
    .but_evt_state = NO_CHANGE
    /* Can ignore the switch event field as this is a button instance */
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
    .current_logic_level = false, // This is actually set to the current value at initialisation
    .current_button_state = IS_SWITCH,
    .is_active_high = true,
    .but_deb_count = 0,
    .current_sw_state = false,
    .previous_sw_state = false
    /* Can ignore the button event field as this is a switch instance */
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

    // For slider switch instances, read in the current logic level to initialise the switch object
    if (!(but_sw_obj->is_button))
    {
        but_sw_obj->current_sw_state = GPIOPinRead (but_sw_obj->tiva_gpio_port,
                                                 but_sw_obj->tiva_gpio_pin);
        but_sw_obj->previous_sw_state = but_sw_obj->current_sw_state;
    }
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
getButtonEventState (const buttonSwitch_t *but_obj)
{
    return (but_obj->but_evt_state);
}


/* Returns the current switch event status of the switch object
 */
switchStates_t
getSwitchEventState (const buttonSwitch_t *sw_obj)
{
    return (sw_obj->sw_evt_state);
}


/* Returns true if the current button event state is PUSHED, false otherwise.
 * Checks the button flags for a button event and resets it.
 */
bool
isButtonEventStatePushed (buttonSwitch_t *but_obj)
{
    bool retVal = false;
    bool isPushed = (getButtonEventState (but_obj) == PUSHED) ? true : false;
    if ((but_obj->but_evt_flag) && (isPushed))
    {
        retVal = true;
    }

    // Reset the button event flag
    but_obj->but_evt_flag = false;

    return retVal;
}


/*
Determines whether a button event is a pushed one or a released event.
This is only called if the polls pass the debounce threshold
 */
butStates_t
updateButtonEventState (const buttonSwitch_t *but_obj)
{
    butStates_t retVal;
    if (but_obj->current_button_state == but_obj->is_active_high)
    {
        retVal = PUSHED;
    }
    else if (but_obj->current_button_state != but_obj->is_active_high)
    {
        retVal = RELEASED;
    }
    else
    {
        retVal = NO_CHANGE;
    }
    return retVal;
}

/*
Determines which switch transition event has occurred
as defined in the switchStates_t enum.
 */
switchStates_t
updateSwitchEventState (const buttonSwitch_t *sw_obj)
{
    switchStates_t retVal;
    if ((sw_obj->current_sw_state) && !(sw_obj->previous_sw_state))
    {
        retVal = PUSHED_UP;
    }
    else if (!(sw_obj->current_sw_state) && (sw_obj->previous_sw_state))
    {
        retVal = PUSHED_DOWN;
    }
    return retVal;
}


/* Debouncing procedure specifically for a button instance. This
 * updates the button event field of the structure if a button push or
 * release event occurs
*/
void
updateButtonObj (buttonSwitch_t *but_obj)
{
    // Only start debouncing when the current logic level differs from the current button state
    if (but_obj->current_logic_level != but_obj->current_button_state)
    {
        // Increment debounce count
        but_obj->but_deb_count++;
        /* If the count exceeds threshold, update the current button state and reset the debounce count. Also update whether it's push or release event */
        if (but_obj->but_deb_count >= NUM_BUT_POLLS)
        {
            but_obj->current_button_state = but_obj->current_logic_level;
            but_obj->but_deb_count = 0;
            // Set whether it's a released or pushed button event
            but_obj->but_evt_state = updateButtonEventState (but_obj);
            // Set flag. This is reset in the task
            but_obj->but_evt_flag = true;
        }
    }
    else
    {
        // Reset button debounce count and explicit set no change event
        but_obj->but_deb_count = 0;
        but_obj->but_evt_state = NO_CHANGE;
    }
}

/* Debouncing procedure specifically for a switch instance. This
 * updates the button event field of the structure if a switch change instance
 * occurs.
 * This might have dodgy logic that needs to be thoroughly checked
*/
void
updateSwitchObj (buttonSwitch_t *sw_obj)
{
    // Only start debouncing when the current logic level differs from the current state
    if (sw_obj->current_logic_level != sw_obj->current_sw_state)
    {
        // Increment debounce count
        sw_obj->but_deb_count++;
        /* If the count exceeds threshold, update the current button state and reset the debounce count.
         * Also update whether it's switch up or switch down event */
        if (sw_obj->but_deb_count >= NUM_BUT_POLLS)
        {
            sw_obj->previous_sw_state = sw_obj->current_sw_state;
            sw_obj->current_sw_state = sw_obj->current_logic_level;
            sw_obj->but_deb_count = 0;
            // Set whether it's a released or pushed button event
            sw_obj->sw_evt_state = updateSwitchEventState (sw_obj);
        }
    }
    else
    {
        // Reset button debounce count and set state to be current logic level
        sw_obj->but_deb_count = 0;
        switchStates_t tmpState;
        if (sw_obj->current_sw_state)
        {
            tmpState = LOGIC_HIGH_STATE;
        }
        else
        {
            tmpState = LOGIC_LOW_STATE;
        }
        sw_obj->sw_evt_state = tmpState;
    }
}

/* Reads in the logic level of the button object passed in regardless of whether it's button or switch instance.
If it's a switch instance, only the logic level of the pin is read and updated.
Otherwise, debouncing is performed on the button instance and the button event is updated if in fact it has been pressed for long enough or if it has been released.
If the parameter instance is a switch, then a slightly different debouncing function is called specifically designed for the switch
*/
void
updateButtonSwitchObj (buttonSwitch_t *but_sw_obj)
{
    /* Check the logic level regardless of whether it's a button or switch instance */ 
    but_sw_obj->current_logic_level = GPIOPinRead (but_sw_obj->tiva_gpio_port, 
                                                    but_sw_obj->tiva_gpio_pin);
    // Button instance debouncing
    if (but_sw_obj->is_button)
    {
        updateButtonObj (but_sw_obj);
    }
    // Switch instance debouncing
    else if (!(but_sw_obj->is_button))
    {
        updateSwitchObj (but_sw_obj);
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
    hwEventQueueItem_t eventItem;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount();

    // Loop forever
    while (1)
    {
        // Poll all buttons and update their button event statuses
        updateAllButtonSwitchObjs();

        // Get button event states from both the up and down button
        bool upButtonPushed = isButtonEventStatePushed (&g_up_button);
        bool downButtonPushed = isButtonEventStatePushed (&g_down_button);
        bool sliderSwitchPushedUp = (getSwitchEventState (&g_slider_switch_one)
                                        == PUSHED_UP) ? true : false;
        bool sliderSwitchPushedDown = (getSwitchEventState (&g_slider_switch_one)
                                        == PUSHED_DOWN) ? true : false;

        // Determine message to append to queue
        if ((upButtonPushed && downButtonPushed))
        {
            eventItem.buttonADCEventType = UP_AND_DOWN_BUTTON_PUSH_EVENT;
            // Guard UART from concurrent access
            xSemaphoreTake (g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf ("Up and down buttons are pressed.\n");
            xSemaphoreGive (g_pUARTSemaphore);
        }
        else if (upButtonPushed && !(downButtonPushed))
        {
            eventItem.buttonADCEventType = UP_BUTTON_PUSH_EVENT;
            // Guard UART from concurrent access
            xSemaphoreTake (g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf ("Up button is pressed.\n");
            xSemaphoreGive (g_pUARTSemaphore);
        }
        else if (!(upButtonPushed) && downButtonPushed)
        {
            eventItem.buttonADCEventType = DOWN_BUTTON_PUSH_EVENT;
            // Guard UART from concurrent access
            xSemaphoreTake (g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf ("Down button is pressed.\n");
            xSemaphoreGive (g_pUARTSemaphore);
        }
        else
        {
            // There has been no button event this time
            eventItem.buttonADCEventType = NO_HW_EVENT;
        }

        // Switch events can happen at the same time as button events
        if (sliderSwitchPushedUp)
        {
            eventItem.switchEventType = SLIDER_PUSH_UP_EVENT;
            // Guard UART from concurrent access
            xSemaphoreTake (g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf ("Slider switch pushed up.\n");
            xSemaphoreGive (g_pUARTSemaphore);
        }
        else if (sliderSwitchPushedDown)
        {
            eventItem.switchEventType = SLIDER_PUSH_DOWN_EVENT;
            // Guard UART from concurrent access
            xSemaphoreTake (g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf ("Slider switch pushed down.\n");
            xSemaphoreGive (g_pUARTSemaphore);
        }
        else
        {
            eventItem.switchEventType = NO_HW_EVENT;
        }

        // Only append message to the queue if any event occurred
        if ((eventItem.buttonADCEventType != NO_HW_EVENT) ||
                (eventItem.switchEventType != NO_HW_EVENT))
        {
            // Append event message to the queue
            if (xQueueSend (g_butsADCEventQueue, &eventItem, portMAX_DELAY) != pdPASS)
            {
                // Queue is full - not good. Should never happen
                xSemaphoreTake (g_pUARTSemaphore, portMAX_DELAY);
                UARTprintf("\nQueue full. This should never happen.\n");
                xSemaphoreGive (g_pUARTSemaphore);
                while(1)
                {
                    // Infinite loop
                }
            }

        }

        // Wait for the required amount of time
        vTaskDelayUntil (&ui16LastTime, ui32PollDelay / portTICK_RATE_MS);
    }
    
}


/* Initialises the peripherals, ports and pins used by the specified buttons and switches in this module */
uint32_t
ButtonsSwitchTaskInit (void)
{
    /* Initialise all of the peripherals, ports and pins used by the switches and buttons */
    initAllButtonSwitchObjs ();
    // Create the buttons switches task
    if (xTaskCreate (ButtonsSwitchTask,
                     (const portCHAR *)"Buttons Switch",
                     BUTTONSSWITCHTASKSTACKSIZE,
                     NULL,
                     tskIDLE_PRIORITY + PRIORITY_BUTTONS_SWITCH_TASK,
                     NULL) != pdTRUE)
    {
        return (1);
    }
    UARTprintf("Buttons and switch task initialized.\n");
    // Success
    return (0);
}




