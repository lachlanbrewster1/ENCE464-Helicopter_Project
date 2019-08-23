/*
 * initialisers.h
 *
 *  Created on: 18/03/2018
 *  Last modified: 29/07/2019
 *      Authors: Jozef Crosland jrc149
 */

#ifndef INITIALISERS_H_
#define INITIALISERS_H_

// *****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, yaw, display
// and UART
// *****************************************************************************
void initSSIGPIO (void);

void 
peripheralReset(void);

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display.
//*****************************************************************************
void
initClock (void);


#endif /* INITIALISERS_H_ */
