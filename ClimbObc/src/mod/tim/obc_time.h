/*
===============================================================================
 Name        : obc_time.h
 Author      : Robert
 Created on	 : 12.09.2021
===============================================================================
// OBC Time is defined in ms after each reset.
// The 'epochNumber' is increased with every reset (Reset Count read from persistence either RTC GPR or MRAM)
// If no valid persisted ResetCount is Available resetNumber -> 0;

// To get a valid UTC Time the OBC time system has to be synchronized.
// This can be done
// 	- after Reset if the RTC is still running and has a valid time set. (This can be kept for some minutes without Vpp by buffered Vbat on the RTC)
//  - with command (triggered from ground station or by a valid GPS-Lock on board)
*/

#ifndef MOD_TIM_OBC_TIME_H_
#define MOD_TIM_OBC_TIME_H_

#include <chip.h>

// API defines
// API structs & types
typedef uint32_t	obc_systime32_t;	// 32 bit counted in ms allows a epoch (period from one reset to next one) to be 49days17h02m !

typedef struct {
	uint32_t        resetNumber;		// current running 'reset-epoch'
	obc_systime32_t msAfterReset;   	// This ms counter is based on XTAL timer IRQ.
} obc_systime64_t;

typedef double      juliandayfraction;

typedef struct {
    uint16_t        	year;
    juliandayfraction	dayOfYear;   	// This starts with 1.0 on 00:00:00.0000 on 1st January and counts up to 365/6.xxxxx on 31.Dec 23:59:59.9999
} obc_tle_fulltime_t;

typedef struct {
	uint32_t	 		rtcDate;		// UTC Date in format YYYYMMDD	(from hardware RTC)
	uint32_t	 		rtcTime;		// UTC Time in format HHMMSS	(from hardware RTC)
	juliandayfraction	tleDay;			// UTC Time in TLE dayOfYear format (calculated from XTAL Systime and last UTC Sync command)
	double				currentDiff;    // in Seconds. The drift of XTAL Clock vs RTC since last Sync command.
} obc_utc_fulltime_t;

typedef union {
    struct {
        uint16_t reset: 		  6;
        uint16_t hwWatchdog : 	  1;
        uint16_t oddEven: 		  1;
        uint16_t rtcOscFail:	  1;
        uint16_t rtcOscError:	  1;
        uint16_t rtcSetDefault:	  1;
        uint16_t rtcCrcError:	  1;
        uint16_t rtcSynchronized: 1;
        uint16_t dummy:			  3;
    };
    struct {
    	uint16_t resetBits;
    	uint16_t rtcOscDelayMs;
    	uint32_t gprResetCount;
    };
} init_report_t;

// API module functions
void timInit (void *dummy);
void timMain (void);

void 	 			timSetResetNumber(uint32_t resetCount);
uint32_t 			timGetResetNumber(void);
obc_systime32_t		timGetSystime(void);

void 				TimSetUtc1(uint16_t year, uint8_t month, uint8_t dayOfMonth, uint8_t hour, uint8_t min, uint8_t sec, bool syncRTC, uint8_t syncSource);
void 				timSyncUtc(uint16_t year, obc_systime32_t systemTime, juliandayfraction utcDateTime, uint8_t syncSource);
obc_utc_fulltime_t 	timGetUTCTime(void);
uint64_t 			timGetUnixTime(void);

juliandayfraction 	timConvertUtcTimeToJdf(uint32_t gpsTime, uint16_t gpsMs);
juliandayfraction 	timConvertUtcDateToJdf(uint32_t gpsDate);



// Event defines (Internal defined event structs can be used on debug and com APIs as generic 'Event'.
#define MODULE_ID_TIME			0x01
#define EVENT_TIM_INITIALIZED	1
#define EVENT_TIM_XTALSTARTED	2				// This is only sent when XTAl Error from init 'disappears'.
#define EVENT_TIM_SYNCHRONIZED	3

#define TIM_SYNCSOURCE_GPS			1
#define TIM_SYNCSOURCE_DEBUGCMD		2
#define TIM_SYNCSOURCE_ONBOARDRTC 	3

#endif /* MOD_TIM_OBC_TIME_H_ */
