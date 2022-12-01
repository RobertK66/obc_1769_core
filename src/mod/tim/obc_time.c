/*
===============================================================================
 Name        : obc_time.c
 Author      : Robert
 Created on	 : 12.09.2021
===============================================================================
This module handles RTC time related functions and Controls the basic IRQs
to control epoch time, synchronizing epoch time and calibrating RTC to XTAL
timing.
One timer is used to count ms.
The RTC IRQ is used to count/check the RTC Seconds.
===============================================================================
*/
#include "obc_time.h"

#include <math.h>
#include <ado_modules.h>
#include <ado_crc.h>
#include "../modules_globals.h"


#define C_RESET_MS_OFFSET 22			// TODO: measure exact start offset

static const uint16_t timeMonthOffsetDays[2][13] = {
   {0,   0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},  // non leap year
   {0,   0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}   // leap year
};

// valid status has 4 higer equal to 4 lower bits.
typedef enum {
	RTC_STAT_SYNCHRONIZED	= 0x11,
	RTC_STAT_RUNNING		= 0x22,
	RTC_STAT_RESETDEFAULT	= 0x33,
	RTC_STAT_UNKNOWN		= 0xDD,
	RTC_STAT_XTAL_ERROR		= 0xE0,
} rtc_status_t;

// Usage of 19 bytes General Purpose Register
typedef enum {
	RTC_GPRIDX_STATUS = 0,
	RTC_GPRIDX_RESETCOUNTER32 = 4,		// must be on base of a uint32 register to be accessed with single read/write.
	RTC_GPRIDX_CRC8 = 19
} rtc_gpridx_t;

typedef struct {
	uint32_t	 rtcDate;
	uint32_t	 rtcTime;
} rtc_event_xtalstarted_t;

typedef struct {
    uint32_t                resetNumber;	// current running 'reset-epoch'
    obc_systime32_t         msAfterReset;	// as is named ;-)
    obc_tle_fulltime_t      utcOffset;      // defines the UTC time (in TLE format) when this reset epoch started.
    double					drift;			// current difference between XTAL(ms)-system based UTC time calculation and current time in hardware RTC.
} tim_synced_systime_t;

typedef struct __attribute__((packed)) {
	uint32_t			resetNumber;
	juliandayfraction 	oldOffset;
	juliandayfraction   newOffset;
	uint8_t				source;
} tim_syncdata_t;




// Prototypes
void timSetRtcDefaults(void);
void timBlockMs(uint16_t ms);
void RtcClearGpr();
uint8_t RtcReadGpr(rtc_gpridx_t idx);
bool RtcIsGprChecksumOk(void);
void RtcWriteGpr(rtc_gpridx_t idx, uint8_t byte);
uint32_t RtcReadGpr32(rtc_gpridx_t idx);
void RtcWriteGpr32(rtc_gpridx_t idx, uint32_t value);

// local used defines/inlines
#define TimConvertTimeToDoyf(hours, minutes, seconds) ((((seconds / 60.0) + minutes) / 60.0 + hours) / 24.0)

//static inline juliandayfraction TimConvertTimeToDoyf(uint32_t hours, uint32_t minutes, uint32_t seconds) {
//	return ((((seconds / 60.0) + minutes) / 60.0 + hours) / 24.0);
//}

static inline juliandayfraction TimConvertMsToDoyf(uint32_t ms) {
    return (((double)ms)/86400000.0);
}
static inline uint32_t rtc_get_time(void) {
	return (LPC_RTC->TIME[RTC_TIMETYPE_SECOND] + LPC_RTC->TIME[RTC_TIMETYPE_MINUTE] * 100 + LPC_RTC->TIME[RTC_TIMETYPE_HOUR] * 10000);
}
static inline uint32_t rtc_get_date(void) {
	return (LPC_RTC->TIME[RTC_TIMETYPE_DAYOFMONTH] + LPC_RTC->TIME[RTC_TIMETYPE_MONTH] * 100 + (LPC_RTC->TIME[RTC_TIMETYPE_YEAR]) * 10000);
}
static inline void timSetUtc2(RTC_TIME_T *fullTime, uint8_t syncSource) {
    TimSetUtc1(fullTime->time[RTC_TIMETYPE_YEAR],
               fullTime->time[RTC_TIMETYPE_MONTH],
               fullTime->time[RTC_TIMETYPE_DAYOFMONTH],
               fullTime->time[RTC_TIMETYPE_HOUR],
               fullTime->time[RTC_TIMETYPE_MINUTE],
               fullTime->time[RTC_TIMETYPE_SECOND],
			   false, syncSource);
}

// Module Variables
static tim_synced_systime_t ObcSystemTime;

// The RIT Timer IRQ counts the ms register
void RIT_IRQHandler(void) {
    LPC_RITIMER->CTRL |= 0x0001;                    // Clear the RITINT flag;
    ObcSystemTime.msAfterReset++;
    //Chip_GPIO_SetPinToggle(LPC_GPIO, 0, 27);
}

// The RTC Seconds Timer IRQ.
void RTC_IRQHandler(void)
{
	Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE);
	Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);

	// Every minute calculate Offset between RTX and XTAL (ms IRQ)
	if ((LPC_RTC->TIME[RTC_TIMETYPE_SECOND] == 0) || (ObcSystemTime.drift == 0.0)) {
		double curXtalTime =  ObcSystemTime.utcOffset.dayOfYear + TimConvertMsToDoyf(ObcSystemTime.msAfterReset);
		double curRtcTime = LPC_RTC->TIME[RTC_TIMETYPE_DAYOFYEAR];
		curRtcTime += TimConvertTimeToDoyf(LPC_RTC->TIME[RTC_TIMETYPE_HOUR], LPC_RTC->TIME[RTC_TIMETYPE_MINUTE], LPC_RTC->TIME[RTC_TIMETYPE_SECOND]);
		ObcSystemTime.drift = (curRtcTime - curXtalTime)*86400.0; // in [seconds]
	}

	/// Check RTC Clock recovery
	rtc_status_t status = RtcReadGpr(RTC_GPRIDX_STATUS);
	if (status == RTC_STAT_XTAL_ERROR) {
		// There was an error while init (no RTC Clock running). Now it seems to be ok (otherwise there would not be an IRQ!?)
		// clear the error bit now.
		if (LPC_RTC->RTC_AUX & RTC_AUX_RTC_OSCF)
		{
			LPC_RTC->RTC_AUX &= RTC_AUX_RTC_OSCF;	// Clear the error by writing to this bit now.
		}
		// TODO: Is this assumption here correct? We had an unknown time from init until now but it seems to run from here on.
		//       For sure it is not synchronized any more.....
		RtcWriteGpr(RTC_GPRIDX_STATUS, RTC_STAT_UNKNOWN);

		rtc_event_xtalstarted_t eventdata;
		eventdata.rtcDate = rtc_get_date();
		eventdata.rtcTime = rtc_get_time();
		SysEvent(MODULE_ID_TIME, EVENT_ERROR, EVENT_TIM_XTALSTARTED, &eventdata, sizeof(rtc_event_xtalstarted_t));
	}
}


void timInit (void *initData) {
	init_report_t *pInitReport = (init_report_t *)initData;

	// we use RIT IRQ to count ms after reset. It counts PCLK cycles
	// This Repetive Interrupt Timer is activated by default with reset.
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_RIT);
	Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_RIT, SYSCTL_CLKDIV_1);

	ObcSystemTime.msAfterReset = C_RESET_MS_OFFSET;
	//Chip_GPIO_SetPinToggle(LPC_GPIO, 0, 28);

	LPC_RITIMER->COMPVAL = (SystemCoreClock/1000) - 1;   					// We want an IRQ every 0,001 Seconds.
	LPC_RITIMER->COUNTER = 0;
	LPC_RITIMER->CTRL = RIT_CTRL_ENCLR | RIT_CTRL_ENBR | RIT_CTRL_TEN;
	NVIC_EnableIRQ(RITIMER_IRQn);

	// Initialize thr RTC Clock module
	// Check if the RTC was freshly started
	if (LPC_RTC->RTC_AUX & RTC_AUX_RTC_OSCF) {
		// This bit is set, if RTC was powerd on (for the first time) or there is/was a failure on the RTC Clock Oscillator
		pInitReport->rtcOscFail = true;

		Chip_RTC_Init(LPC_RTC);					// resets the calibration and all registers to 0. TODO: check to get calibration from memory....
		Chip_RTC_Enable(LPC_RTC, ENABLE);		// We have to enable the RTC now

		// Wait until RTC XTAL is running. Check by Clearing the OSC Error bit and see if it comes on again
		// We allow 400ms to stabilize. (On EM2 after deep power off it takes aprx. 300ms / now 90ms !?)
		uint32_t tryUntil = ObcSystemTime.msAfterReset + 400;
		while(tryUntil > ObcSystemTime.msAfterReset) {
			if (!(LPC_RTC->RTC_AUX & RTC_AUX_RTC_OSCF)) {
				// If there is no OSC error pending, we are ok to go
				pInitReport->rtcOscDelayMs = (uint16_t)(tryUntil - ObcSystemTime.msAfterReset);
				break;
			}
			LPC_RTC->RTC_AUX &= RTC_AUX_RTC_OSCF;	// Clear the OSC Error bit
			timBlockMs(5);							// and give it some time to re-appear....
		}

		// Now RTC should be up and running
		if (LPC_RTC->RTC_AUX & RTC_AUX_RTC_OSCF) {
			// If not (tested with defect OBC board!) we really have no RTC OSC running!??
			LPC_RTC->RTC_AUX &= RTC_AUX_RTC_OSCF;	// Clear the error  bit.
			pInitReport->rtcOscError = true;
			RtcWriteGpr(RTC_GPRIDX_STATUS, RTC_STAT_XTAL_ERROR);
			// No need to continue with RTC Init here.....
			return;
		} else {
			// RTC Oscillator started. Lets init the RTC content now.
			pInitReport->rtcSetDefault = true;
			timSetRtcDefaults();
		}
	} else {
		// We can assume the RTC to be running and ready to use....
		if (RtcIsGprChecksumOk()) {
			// We can rely on GPR holding correct status and Reset Count
			ObcSystemTime.resetNumber = RtcReadGpr32(RTC_GPRIDX_RESETCOUNTER32);
			pInitReport->gprResetCount = ObcSystemTime.resetNumber;
			// Increment the GPR Reset count and use as current Epoch Number
			ObcSystemTime.resetNumber++;
			RtcWriteGpr32(RTC_GPRIDX_RESETCOUNTER32, ObcSystemTime.resetNumber);

			if (RtcReadGpr(RTC_GPRIDX_STATUS) == RTC_STAT_SYNCHRONIZED) {
				// RTC was synchronized before Reset (and survived) so we can use it to synchronize again.
				RTC_TIME_T 	time;
				Chip_RTC_GetFullTime(LPC_RTC, &time);
				timSetUtc2(&time, TIM_SYNCSOURCE_ONBOARDRTC);
				pInitReport->rtcSynchronized = true;
			}
		} else {
			// RTC survived but GPR ram is defect !? Start with fresh RAM.
			ObcSystemTime.resetNumber = 0; 			// This will be overwritten Later by Persitence of Epoch Nr....
			pInitReport->rtcCrcError = true;
			pInitReport->rtcSetDefault = true;
			timSetRtcDefaults();
		}
	}

	// Enable the 1 Second IRQ
	Chip_RTC_CntIncrIntConfig(LPC_RTC, RTC_AMR_CIIR_IMSEC,  ENABLE);
	//NVIC_SetPriority(RTC_IRQn, RTC_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(RTC_IRQn); /* Enable interrupt */

}

void timSetRtcDefaults(void) {
	RTC_TIME_T 	time;

	RtcClearGpr();
	/* RTC module has been down, time and data invalid */
	/* Set to default values */
	time.time[RTC_TIMETYPE_SECOND] = 0;
	time.time[RTC_TIMETYPE_MINUTE] = 0;
	time.time[RTC_TIMETYPE_HOUR] = 0;
	time.time[RTC_TIMETYPE_DAYOFMONTH] = 1;
	time.time[RTC_TIMETYPE_DAYOFWEEK] = 1;
	time.time[RTC_TIMETYPE_DAYOFYEAR] = 1;
	time.time[RTC_TIMETYPE_MONTH] = 1;
	time.time[RTC_TIMETYPE_YEAR] = 1001;
	Chip_RTC_SetFullTime(LPC_RTC, &time);
	RtcWriteGpr(RTC_GPRIDX_STATUS, RTC_STAT_RESETDEFAULT);
}


void timMain (void) {
	// Nothing to do here (yet) ....
}

void RtcClearGpr() {
	for (int i=0;i<5;i++) {
		LPC_RTC->GPREG[i]= 0x00000000;
	}
}

// We use the 5 GPR registers as a byte store with 19 bytes + 1byte CRC8
void RtcWriteGpr(rtc_gpridx_t idx, uint8_t byte) {
	if ((idx < 19) && (idx >= 0)) {
		// GPREG only can be written as uint32/4byte at once!
		// A single uint8_t ptr does not work here :-( -> it writes 4 (same) bytes at once !?
		uint32_t *gprbase = (uint32_t *)(&(LPC_RTC->GPREG));
		uint32_t *ptr = gprbase;
		uint8_t channel = idx /4;
		uint8_t tmpBytes[4];

		ptr += channel;
		*((uint32_t *)tmpBytes) = *ptr;
		tmpBytes[idx % 4] = byte;
		*ptr = *((uint32_t *)tmpBytes);

		// last byte of word 4 is checksum
		ptr = gprbase + 4;
		*((uint32_t *)tmpBytes) = *ptr;
		tmpBytes[3] = CRC8((uint8_t*)&(LPC_RTC->GPREG), 19);
		*ptr = *((uint32_t *)tmpBytes);
	}
}

uint32_t RtcReadGpr32(rtc_gpridx_t idx) {
	return LPC_RTC->GPREG[idx>>2];
}

void RtcWriteGpr32(rtc_gpridx_t idx, uint32_t value) {
	uint32_t *gprbase = (uint32_t *)(&(LPC_RTC->GPREG));
	uint32_t *ptr = gprbase;
	uint8_t tmpBytes[4];

	LPC_RTC->GPREG[idx>>2] = value;
	// last byte of word 4 is checksum
	ptr = gprbase + 4;
	*((uint32_t *)tmpBytes) = *ptr;
	tmpBytes[3] = CRC8((uint8_t*)&(LPC_RTC->GPREG), 19);
	*ptr = *((uint32_t *)tmpBytes);
}

uint8_t RtcReadGpr(rtc_gpridx_t idx) {
	// Other than writing, for reading a uint8 ptr is good enough to get all bytes separately.
	uint8_t *gprbase = (uint8_t *)(&(LPC_RTC->GPREG));
	return gprbase[idx];
}

bool RtcIsGprChecksumOk(void) {
	uint8_t *gprbase;
	// The rtc does not change its GPR on itself, does it?
	// So no need to copy and/or disable the interrupts here !?
	gprbase = (uint8_t *) &(LPC_RTC->GPREG);
	uint8_t crc = CRC8(gprbase, 19);
	return (gprbase[19] == crc);
}

void timBlockMs(uint16_t ms) {
	uint32_t waituntil = ObcSystemTime.msAfterReset + ms;
	while (ObcSystemTime.msAfterReset<waituntil) {
	}
}

// conversion routines for different date/time formats
static inline uint16_t timGetDayOfYear(uint16_t year,uint8_t month,uint8_t dayOfMonth) {
	int leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
	return timeMonthOffsetDays[leap][month] + dayOfMonth;
}


// gpsTime: format uint value with hhmmss
juliandayfraction timConvertUtcTimeToJdf(uint32_t gpsTime, uint16_t gpsMs) {
	uint8_t  sec = (uint8_t)(gpsTime % 100);
	uint8_t  min = (uint8_t)((gpsTime / 100) % 100);
	uint8_t  hour = (uint8_t)(gpsTime / 10000);

	juliandayfraction timetoday;
	if ((sec < 60) & (min < 60) & (hour < 23) & (gpsMs < 1000)) {
		timetoday = (((sec / 60.0 + min) / 60.0 + hour) / 24.0 ) + (gpsMs/86400000.0);
	} else {
		timetoday = 0.0;
	}
	return timetoday;
}

// gpsDate: format uintvalue with ddmmyy
juliandayfraction timConvertUtcDateToJdf(uint32_t gpsDate) {
	uint16_t year = 2000 + (gpsDate % 100);
	uint8_t  month = (uint8_t)((gpsDate/100) % 100);
	uint8_t  dayOfMonth = (uint8_t)(gpsDate/10000);

	if ((month <= 12) && (dayOfMonth < 32)) {
		uint16_t dayOfYear = timGetDayOfYear(year, month, dayOfMonth);
		return (juliandayfraction)dayOfYear;
	} else {
		return 0.0;
	}
}


void timSyncUtc(uint16_t year, obc_systime32_t systemTime, juliandayfraction utcDateTime, uint8_t syncSource) {

	// Calculate and store the offset for this systime-utc time pair (stored some 'NMEA records ago!)
	// So this does not show current time here!
	tim_syncdata_t syncData;


	syncData.oldOffset = ObcSystemTime.utcOffset.dayOfYear;
	syncData.newOffset = utcDateTime - TimConvertMsToDoyf(systemTime);

	float diff = fabs(syncData.newOffset - syncData.oldOffset);
	if ( diff > 0.00005) {
		ObcSystemTime.utcOffset.dayOfYear = syncData.newOffset;

		// Now we (re)calculate current Time (from systime and offset) and set this to our RTC registers
		RTC_TIME_T 	time;

		juliandayfraction currenttime = ObcSystemTime.utcOffset.dayOfYear + TimConvertMsToDoyf(ObcSystemTime.msAfterReset);
		// Calculate month and day of month for this year
		uint16_t dayInYear = (uint16_t)currenttime;
		int leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
		uint8_t month;
		for (month = 1; month < 13; month++) {
			if (dayInYear <= timeMonthOffsetDays[leap][month]) {
				break;
			}
		}
		month--;
		uint8_t dayOfMonth = dayInYear - timeMonthOffsetDays[leap][month];

		time.time[RTC_TIMETYPE_YEAR] = year;

		time.time[RTC_TIMETYPE_DAYOFMONTH] = dayOfMonth;
		time.time[RTC_TIMETYPE_DAYOFWEEK] = 1;
		time.time[RTC_TIMETYPE_DAYOFYEAR] = dayInYear;
		time.time[RTC_TIMETYPE_MONTH] = month;


		currenttime -= dayInYear;	// This should give time (of day) only part.
		currenttime *= 24;
		uint8_t hours = (uint8_t)currenttime;
		currenttime -= hours;
		currenttime *= 60;
		uint8_t minutes = (uint8_t)currenttime;
		currenttime -= minutes;
		currenttime *= 60;
		uint8_t seconds = (uint8_t)currenttime;

		time.time[RTC_TIMETYPE_SECOND] = seconds;
		time.time[RTC_TIMETYPE_MINUTE] = minutes;
		time.time[RTC_TIMETYPE_HOUR] = hours;

		// TODO make this RTC setting somehow ms accurate !!!
		Chip_RTC_SetFullTime(LPC_RTC, &time);
		RtcWriteGpr(RTC_GPRIDX_STATUS, RTC_STAT_SYNCHRONIZED);

		syncData.resetNumber = ObcSystemTime.resetNumber;
		syncData.source = syncSource;
		SysEvent(MODULE_ID_TIME, EVENT_INFO, EVENT_TIM_SYNCHRONIZED, &syncData, sizeof(syncData));
	}
}

void TimSetUtc1(uint16_t year, uint8_t month, uint8_t dayOfMonth, uint8_t hour, uint8_t min, uint8_t sec, bool syncRTC,uint8_t syncSource) {
	tim_syncdata_t syncData;
	syncData.oldOffset = ObcSystemTime.utcOffset.dayOfYear;

    ObcSystemTime.utcOffset.year = year;

    // First we calculate the day depending of leap year
    juliandayfraction day = (juliandayfraction)timGetDayOfYear(year, month, dayOfMonth);

    // Then we add the time fraction
    day += ((sec / 60.0 + min) / 60.0 + hour) / 24.0;

    // To store the offset to this time we subtract current ms.
    ObcSystemTime.utcOffset.dayOfYear = day - TimConvertMsToDoyf(ObcSystemTime.msAfterReset);;
    syncData.newOffset = ObcSystemTime.utcOffset.dayOfYear;

    // Now we synchronize this UTC time into RTC
    if (syncRTC) {
    	RTC_TIME_T time;
    	time.time[RTC_TIMETYPE_SECOND] = sec;
    	time.time[RTC_TIMETYPE_MINUTE] = min;
    	time.time[RTC_TIMETYPE_HOUR] = hour;

    	time.time[RTC_TIMETYPE_DAYOFMONTH] = dayOfMonth;
    	time.time[RTC_TIMETYPE_DAYOFYEAR] = (uint32_t)day;
    	time.time[RTC_TIMETYPE_MONTH] = month;
    	time.time[RTC_TIMETYPE_YEAR] = year;

    	Chip_RTC_SetFullTime(LPC_RTC, &time);
    	Chip_RTC_ResetClockTickCounter(LPC_RTC);
    	RtcWriteGpr(RTC_GPRIDX_STATUS, RTC_STAT_SYNCHRONIZED);
    }

    syncData.resetNumber = ObcSystemTime.resetNumber;
    syncData.source = syncSource;
    SysEvent(MODULE_ID_TIME, EVENT_INFO, EVENT_TIM_SYNCHRONIZED, &syncData, sizeof(syncData));

    // and finally we reset the offset between XTAL and RTC -> this triggers immediate recalc with next sec RTC IRQ.
    ObcSystemTime.drift = 0.0; // in [seconds]

}

void timSetResetNumber(uint32_t resetCount) {
	ObcSystemTime.resetNumber = resetCount;
	RtcWriteGpr32(RTC_GPRIDX_RESETCOUNTER32, resetCount);
}

uint32_t timGetResetNumber(void) {
	return ObcSystemTime.resetNumber;
}

obc_utc_fulltime_t timGetUTCTime(void) {
	obc_utc_fulltime_t retVal;
	retVal.rtcDate = rtc_get_date();
	retVal.rtcTime = rtc_get_time();
	retVal.tleDay =  ObcSystemTime.utcOffset.dayOfYear + (ObcSystemTime.msAfterReset / 86400000.0);
	retVal.currentDiff = ObcSystemTime.drift;

	return retVal;
}

obc_systime32_t inline	timGetSystime(void) {
	return ObcSystemTime.msAfterReset;
}
