/*
 * radtest.c
 *
 *  Created on: 08.08.2022
 *      Author: Robert
 */

#include "radtest.h"

#include "../mod/ado_timers.h"

//static LPC_TIMER_T* RtstTimerPtr;

// This is a temporary module to implement radiation test code for 2022-08-20 Tests.
// Will be removed again later.
void rtst_timer_IRQHandler(void) {

}


void rtst_init (void *initData) {
	//RtstTimerPtr = ((rtst_initdata_t*)initData)->RadTestTimerPtr;
	InitTimer(((rtst_initdata_t*)initData)->RadTestTimerPtr, 1000, rtst_timer_IRQHandler);

}

void rtst_main (void){

}

