#ifndef BUTTONS_SWITCH_TASK_H_
#define BUTTONS_SWITCH_TASK_H_

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
enum butNames {UP_BUTTON = 0, DOWN_BUTTON};

typedef enum but_evt_e
{
    NO_EVENT = 0,
    UP_BUTTON_PUSHED,
    DOWN_BUTTON_PUSHED,
    UP_AND_DOWN_BUTTON_PUSHED
} butEvents_t;

typedef enum but_states_e 
{
    RELEASED = 0, 
    PUSHED, 
    NO_CHANGE, 
    IS_SWITCH
} butStates_t;

typedef enum switch_states_e {
    LOGIC_LOW_STATE = 0,
    LOGIC_HIGH_STATE,
    PUSHED_UP,
    PUSHED_DOWN
} switchStates_t;

typedef enum active_high_type_e {
    NA_IS_SWITCH = -1,
    IS_ACTIVE_LOW = false,
    IS_ACTIVE_HIGH = true
} activeHighType_t;

enum sliderStates {BOT = 0, TOP};
#define NUM_SWITCHES (1)
#define NUM_BUTS (2)
#define NUM_SWITCHES_AND_BUTTONS (NUM_SWITCHES + NUM_BUTS)

/* Button structure (as close as I can get to an object)
Used to initialise and configure the GPIO pins on the Tiva board
and to store the current levels of the button
 */
typedef struct button_switch_s_t
{
    uint32_t tiva_peripheral_base;
    uint32_t tiva_gpio_port;
    uint8_t tiva_gpio_pin;
    uint32_t tiva_gpio_strength;
    uint32_t tiva_gpio_pin_type;
    bool is_button;
    bool is_active_high;
    bool current_logic_level;
    bool current_button_state;
    uint8_t but_deb_count;
    butStates_t but_evt_state;
    switchStates_t sw_evt_state;
    bool current_sw_state;
    bool previous_sw_state;
} buttonSwitch_t;


// UP button
#define UP_BUT_PERIPH  SYSCTL_PERIPH_GPIOE
#define UP_BUT_PORT_BASE  GPIO_PORTE_BASE
#define UP_BUT_PIN  GPIO_PIN_0
#define UP_BUT_ACTIVE_HIGH  true
#define UP_BUT_GPIO_STRENGTH GPIO_STRENGTH_2MA
#define UP_BUT_GPIO_TYPE GPIO_PIN_TYPE_STD_WPD
#define UP_BUT_NORMAL false

// DOWN button
#define DOWN_BUT_PERIPH  SYSCTL_PERIPH_GPIOD
#define DOWN_BUT_PORT_BASE  GPIO_PORTD_BASE
#define DOWN_BUT_PIN  GPIO_PIN_2
#define DOWN_BUT_ACTIVE_HIGH  true
#define DOWN_BUT_GPIO_STRENGTH GPIO_STRENGTH_2MA
#define DOWN_BUT_GPIO_TYPE GPIO_PIN_TYPE_STD_WPD
#define DOWN_BUT_NORMAL false

// Slider switch one on the Orbit Daughter board
#define SLIDER_ONE_PERIPH SYSCTL_PERIPH_GPIOA
#define SLIDER_ONE_PORT_BASE GPIO_PORTA_BASE
#define SLIDER_ONE_PIN GPIO_PIN_7
#define SLIDER_ONE_ACTIVE_HIGH true
#define SLIDER_ONE_GPIO_STRENGTH GPIO_STRENGTH_2MA
#define SLIDER_ONE_GPIO_TYPE GPIO_PIN_TYPE_STD_WPD

#define NUM_BUT_POLLS 3

#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

/*
Given a button switch type object, the necessary peripheral, port and pin is initialised and enabled. 
*/
void
initButtonSwitchObj (buttonSwitch_t *but_sw_obj);

/* 
Initialises all of the globally defined buttons and switches for this module 
*/
void
initAllButtonSwitchObjs (void);


/*
 Returns the current button event status of the button object
 */
butStates_t
getButtonEventState (const buttonSwitch_t *but_obj);

/* Returns true if the current button event state is PUSHED, false otherwise
 */
bool
isButtonEventStatePushed (const buttonSwitch_t *but_obj);

/*
Determines whether a button event is a pushed one or a released event.
This is only called if the polls pass the debounce threshold
 */
butStates_t
updateButtonEventState (const buttonSwitch_t *but_obj);

/* Reads in the logic level of the button object passed in regardless of whether it's button or switch instance.
If it's a switch instance, only the logic level of the pin is read and updated. Otherwise, debouncing is performed on the button instance and the button event is updated if in fact it has been pressed for long enough or if it has been released */
void 
updateButtonSwitchObj (buttonSwitch_t *but_sw_obj);

/* Updates all of the switches and buttons as defined in this module */
void
updateAllButtonSwitchObjs (void);


extern uint32_t ButtonsSwitchTaskInit (void); 

#endif /*BUTTONS_SWITCH_TASK_H_*/
