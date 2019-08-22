/*
 * initialisers.h
 *
 *  Created on: 18/03/2018
 *  Last modified: 29/07/2019
 *      Authors: Jozef Crosland jrc149
 */

#ifndef INITIALISERS_H_
#define INITIALISERS_H_

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

#endif /* INITIALISERS_H_ */
