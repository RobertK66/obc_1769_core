/*
 * thr.h
 *
 *  Created on: 01.03.2022
 *      Copy Paste by: Jevgeni
 */

//
#ifndef MOD_TIM_CLIMB_THR_H_
#define MOD_TIM_CLIMB_THR_H_

#include <chip.h>

// init data needed. Choose UARt and 2 GPIO Pins to be used
typedef struct {
	LPC_USART_T 	*pUart;			// default is 9600baud
	const PINMUX_GRP_T2   *pEnablePin;	// Output LPC -> GPS enable internal Voltage regulator -> we disable because we use SP Power
	const PINMUX_GRP_T2   *pPpsPin;		// Input  LPC <- GPS gives a 100ms pulse every second (if gps locked and synchronized only???).
} thr_initdata_t;

// API module functions
void thrInit (void *initData);
void thrMain (void);

void thrSendBytes(uint8_t *data, uint8_t len);


#endif /* MOD_TIM_CLIMB_GPS_H_ */
