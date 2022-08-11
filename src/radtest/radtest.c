/*
 * radtest.c
 *
 *  Created on: 08.08.2022
 *      Author: Robert
 */

#include "radtest.h"

#include "../mod/ado_timers.h"
#include <ado_modules.h>
#include "../mod/l3_sensors.h"
#include "../mod/thr/thr.h"

#define 			RTST_MODNR					0x55			// ('U')  Module number for rad tests only
#define 			RTST_EVENTID_HEARTBEAT		0x55			// ('U')  Event ID for heartbeat event

#define 			RTST_TICK_MS				1000			// IRQ every second.
#define 			RTST_HEARTBEAT_TICKS		  10			// Heartbeat every 10 seconds
#define				RTST_MEMTST_TICKS			  16			// Memorytest all 15 seconds
#define				RTST_SENSOR_TICKS			  10		// I2C internal sensor ticks every 10 sec
#define				RTST_RS485_TICKS			  15		// I2C internal sensor ticks every 10 sec

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
			//SysEvent(RTST_MODNR, EVENT_WARNING, RTST_EVENTID_HEARTBEAT, "Supervision watchdog feed\n", 26);
			SysEvent_Debug("Supervision watchdog feed\n", 26)
		}
		if ((RtstTickCnt % RTST_MEMTST_TICKS) == 0) {
			// Memory tests TODO
		}
		if ((RtstTickCnt % RTST_SENSOR_TICKS) == 0) {
			SenReadAllValues();
			}

		if ((RtstTickCnt % RTST_RS485_TICKS) == 0) {

				// Send bytes over RS485

				uint8_t request[11];
				request[0]= 0x74;
				request[1]= 0x68;
				request[2]= 0x72;
				request[3]= 0x5f;
				request[4]= 0x68;
				request[5]= 0x65;
				request[6]= 0x6c;
				request[7]= 0x6c;
				request[8]= 0x6f;
				request[9]= 0x0a; // New line. This should be present
				request[10]= 0x0a; // New line. This should be present

				uint8_t len = sizeof(request);
				thrSendBytes(request, len);


		}
	}
}

