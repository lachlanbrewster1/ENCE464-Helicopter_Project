/*
 * initialisers.h
 *
 *  Created on: 18/03/2018
 *  Last modified: 09/04/2018
 *      Authors: Jozef Crosland jrc149
 *               Alex Greer agr108
 */

#ifndef INITIALISERS_H_
#define INITIALISERS_H_

#include "sharedConstantsTypes.h"


//*********************************
//Initialises the OperatingData_t
//*********************************
//void initOperatingData (OperatingData_t* data, uint32_t altValue);

void initSSIGPIO (void);

//*****************************************************************************
// Initialises all GPIO ports (A-F), UART0 and ADC1.
//*****************************************************************************
//void
//initEnableProgPeripherals(void);


void 
peripheralReset(void);

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display.
//*****************************************************************************
void
initClock (void);

//*****************************************************************************
// Initialises the Analog-to-Digital Conversion.
//*****************************************************************************
//void
//initADC (void);

//*****************************************************************************
// Initialises the yaw measurements from the quadrature encoder.
//*****************************************************************************
//void
//initYaw(void);


//*****************************************************************************
// Initialises the GPIO Port A to trigger when the virtual reset is active
//*****************************************************************************
//void
//initProgReset(void);

//*****************************************************************************
// Initialises the GPIO Port A to trigger when the virtual reset is active
//*****************************************************************************
//void
//initRefYawChannel(void);


#endif /* INITIALISERS_H_ */
