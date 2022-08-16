/*
 * radtest.c
 *
 *  Created on: 08.08.2022
 *      Author: Robert
 */

#include "radtest.h"

#include <stdio.h>
#include <ado_modules.h>

#include "../mod/ado_timers.h"
#include "../mod/l3_sensors.h"
#include "../mod/l2_debug_com.h"
#include "../mod/thr/thr.h"
#include "../mod/l7_climb_app.h"

#define 			RTST_MODNR					0x55			// ('U')  Module number for rad tests only
#define 			RTST_EVENTID_HEARTBEAT		0x55			// ('U')  Event ID for heartbeat event

#define 			RTST_TICK_MS				1000			// IRQ every second.
#define 			RTST_HEARTBEAT_TICKS		  10			// Heartbeat every 10 seconds
#define				RTST_MEMTST_TICKS			  16			// Memorytest all 15 seconds
#define				RTST_SENSOR_TICKS			  13			// I2C internal sensor ticks every 10 sec
#define				RTST_RS485_TICKS			  150			// ...

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

	// Output the headers of all data records used later;
	const char *header = "\nSensor; temp1; temp2; voltage; current; \n";
	uint8_t len = strlen(header);
	deb_print_pure_debug((uint8_t *)header, len);
//	header = "Init; resetCount; \n";
//	len = strlen(header);
//	deb_print_pure_debug((uint8_t *)header, len);
}

void rtst_eventoutput(event_t event) {
	// let's only send pure strings to debug UART and filter only wanted events....
	// In order to get sprintf to work with floats you have to omit the CR_INTEGER_PRINTF option in the setup
	// Later on we should consider to reintroduce it -> no floating points to string is needed for flight version and
	// this reduces memory (flash) footprint.
	char msg[100];

	if (event.id.moduleId == MODULE_ID_SENSORS) {
		if (event.id.eventId == EID_SEN_MEASSUREMENT) {
			// Convert Sensor Measurements to pure string result
			sensor_values_t *sensval = (sensor_values_t *)event.data;
			int len = snprintf(msg, 100, "Sensor; %.2f; %.2f; %.3f; %.3f; \n", sensval->TempSHT30, sensval->Temperature, sensval->SupplyVoltage, sensval->SupplyCurrentBoard);
			deb_print_pure_debug((uint8_t *)msg, len);
		}
	}
	if (event.id.moduleId == MODULE_ID_CLIMBAPP) {
		if (event.id.eventId == EID_APP_STRING) {
			deb_print_pure_debug((uint8_t *)event.data, event.byteCnt);
		}
	}

}


#define CTUT_2	// CTUT_1 CTUT_2  CTUT_3 replace as you wish to check different code options ;-)....
				// Robert: I would prefer CTUT_2 but this is up to taste (only the unreadable CTUT_1 version , I would object)

void rtst_main (void){
	if (RtstTick) {
		// Called once every second.
		RtstTick = false;
		RtstTickCnt++;

		if ((RtstTickCnt % RTST_HEARTBEAT_TICKS) == 0) {
			// Make heartbeat every 10 sec.
#ifdef CTUT_1
			// A short C-Tutorial Option 1 - your code with my comments
			uint8_t request[26];		// Compiler reserves space for 26 uint8_t variables on current stack (-> 26 bytes used on stack - not initilaized filled with random stuff)
										// Compiler gives you access to an (uint8_t *) - read "an uint8_t Pointer" - which is initialized to point to the first uint8t byte on the stack
			                            //          under the name 'request'
										// I think the "pointer request" also goes to the stack here (having 4 more bytes used for it) but I am not sure about this

			request[0]= 0x53;			// left side: compiler takes the content of "uint8_t Pointer named request" ...
										//			  adds 0 times the size of a uint8_t variable (1) -> which here does nothing to the pointer
										//            and stores the const from the right side 0x53 into the porition where the calculated pointer points to.
										// complicated way to say that the [x] operator after a pointer variable treats the memory beeing available as "Array of element types"
			                            //            where here elemnt type is uint8_t (needing 1 byte of memory space)
			request[1]= 0x75;
			request[2]= 0x70;
			request[3]= 0x65;

			// To make this 'meters of code' ;-) more readable you could write it like this:
			request[4]= 'r';	// -> depending of compiler warning levels this could lead to a warning here because 'x' is treated as type "char" and it has to be casted to uint8_t to fit your variable type.
								// My IDE here gives no warnings ->?
			// or if there is a warning and you want to avoid this:
			request[5]= (uint8_t)'v'; // with the 'cast' operator (uint8_t) you can tell the compiler that you know that this is what you intend here and it should not complain about it!
			request[6]= 'i'; //0x69;
			request[7]= 's'; //0x73;
			request[8]= 'i'; //0x69;
			request[9]= 'o'; // 0x6f;   ok I think you get it ;-) would be much more readable and checkable with char ' ' constants!!!!
			request[10]= 0x6e;
			request[11]= 0x20;
			request[12]= 0x77;
			request[13]= 0x61;
			request[14]= 0x74;
			request[15]= 0x63;
			request[16]= 0x68;
			request[17]= 0x64;
			request[18]= 0x6f;
			request[19]= 0x67;
			request[20]= 0x20;
			request[21]= 0x66;
			request[22]= 0x65;
			request[23]= 0x65;
			request[24]= 0x64;
			request[25]= 0x0a;
			uint8_t len = sizeof(request);		// This sizeof() is calculated by the compiler to be exactly the 26 you wrote in line 61. Nothing done at runtime here.
			   deb_print_pure_debug(request, len);
#endif
			
#ifdef CTUT_2
			const char *request = "Supervision watchdog feed\n";	// Here the compiler puts the string literal from the right side
																	// somewhere into read only memory including a 0x00 or '\0' char as
			                                                		// as end of string marker.
															// The (char *) variable 'request' is put on the stack memory and filled with the address of the literal
															// -> pointing to the first char.
			// So here we have similar status as in line 100 in CTUT_1 above!
			// Difference is we have (char *) and not a (uint8_t *) and the string is 'read only'

			// now how to we get length !?
			// it would be possible with:
			uint8_t len = strlen(request);		// This 'calculates' the char from pointer (base) up to the \0
												// I am pretty sure this is evaluated at compile time and not a call at runtime (but this is not 100% sure....)

			// In order to avoid the compiler warning here we have to explicitly cast the request variable beeing a (char *) to (uint8_t *)
			// This does nothing in code or at runtime. It only tells the compiler that he should shut up with his warning. We know what we do here ;-)!
			deb_print_pure_debug((uint8_t *)request, len);
#endif


#ifdef CTUT_3
			// If you want to avoid the strlen you could also use sizeof like this:
#define MY_OUTPUT_STRING "Supervision watchdog feed\n"		// We need the string literal 2 times now so use #define
			                                                // to make it unique and usable as often we want...

			const char *request =  MY_OUTPUT_STRING;
			uint8_t len = sizeof(MY_OUTPUT_STRING);
			   deb_print_pure_debug((uint8_t *)request, len);
#endif
			
			
		}
		if ((RtstTickCnt % RTST_MEMTST_TICKS) == 0) {
			// Memory tests TODO
		}
		if ((RtstTickCnt % RTST_SENSOR_TICKS) == 0) {
			SenReadAllValues();
			}

		if ((RtstTickCnt % RTST_RS485_TICKS) == 0) {

				// Send bytes over RS485

				uint8_t request[10];
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

				uint8_t len = sizeof(request);
				thrSendBytes(request, len);


		}
	}
}

