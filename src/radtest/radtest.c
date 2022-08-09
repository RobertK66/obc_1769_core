/*
 * radtest.c
 *
 *  Created on: 08.08.2022
 *      Author: Robert
 */

#include "radtest.h"

#include "../mod/ado_timers.h"
#include <ado_modules.h>

#define 			RTST_MODNR					0x55			// ('U')  Module number for rad tests only
#define 			RTST_EVENTID_HEARTBEAT		0x55			// ('U')  Event ID for heartbeat event

#define 			RTST_TICK_MS				1000			// IRQ every second.
#define 			RTST_HEARTBEAT_TICKS		  10			// Heartbeat every 10 seconds
#define				RTST_MEMTST_TICKS			  16			// Memorytest all 15 seconds


static LPC_TIMER_T  *RtstTimerPtr = 0;
static bool 		RtstTick = false;
static uint16_t 	RtstTickCnt  = 0;


// This is a temporary module to implement radiation test code for 2022-08-20 Tests.
// Will be removed again later.

// Timer IRQ is called once every second.
void rtst_timer_IRQHandler(void) {
	uint32_t ir = RtstTimerPtr->IR;
	if (ir & 0x01) {
		// Match 0 detected
		RtstTick = true;
	}
	RtstTimerPtr->IR = ir;	// Clear all IRQ bits
}


void rtst_init (void *initData) {
	RtstTimerPtr = ((rtst_initdata_t*)initData)->RadTestTimerPtr;
	InitTimer(RtstTimerPtr, RTST_TICK_MS, rtst_timer_IRQHandler);
}

void rtst_main (void){
	if (RtstTick) {
		// Called once every second.
		RtstTick = false;
		RtstTickCnt++;

		if ((RtstTickCnt % RTST_HEARTBEAT_TICKS) == 0) {
			// Make heartbeat every 10 sec.
			SysEvent(RTST_MODNR, EVENT_WARNING, RTST_EVENTID_HEARTBEAT, "Supervision watchdog feed\n", 26);
		}
		if ((RtstTickCnt % RTST_MEMTST_TICKS) == 0) {
			// Memory tests TODO
		}
	}
}

