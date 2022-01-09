/*
===============================================================================
 Name        : obc_time.h
 Author      : Robert
 Created on	 : 12.09.2021
===============================================================================
*/

#ifndef MOD_TIM_OBC_TIME_H_
#define MOD_TIM_OBC_TIME_H_

#include <Chip.h>

void tim_init (void *dummy);
void tim_main (void);

#define MODULE_ID_TIME			0x01
#define EVENT_TIM_INITIALIZED	1
#define EVENT_TIM_XTALSTARTED	2				// This is only sent when XTAl Error from init 'disappears'.

typedef uint32_t    obc_timestamp;				// 32 bit counted in ms allows a epoch (period from one reset to next one) to be 49days17h02m !
typedef double      juliandayfraction;

typedef struct {
    uint16_t            year;
    juliandayfraction   dayOfYear;   		// This starts with 1.0 on 00:00:00.0000 on 1st January and counts up to 365/6.xxxxx on 31.Dec 23:59:59.9999
} obc_tim_tledatetime_t;

// OBC Time is defined in ms after each reset.
// The 'epochNumber' is increased with every reset (Reset Count read from persistence either RTC GPR or MRAM)
// If no valid persisted ResetCount can be found epochNumber=0;

// To get a valid UTC Time the OBC time system has to be synchronized.
// This can be done
// 	- after Reset if the RTC is still running and has a valid time set. (This can be kept for some minutes without Vpp by buffered Vbat on the RTC)
//  - with command (triggered from ground station or by a valid GPS-Lock on board)

typedef struct {
    uint32_t                epochNumber;
    obc_timestamp           msAfterStart;
    obc_tim_tledatetime_t   utcOffset;      	// year 0 meaning unknown/unsynced
} obc_tim_systemtime_t;


void tim_setEpochNumber(uint32_t resetCount);
uint32_t tim_getEpochNumber(void);

obc_tim_systemtime_t tim_getSystemTime(void);


#endif /* MOD_TIM_OBC_TIME_H_ */
