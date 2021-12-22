/*
===============================================================================
 Name        : obc_time.c
 Author      : Robert
 Created on	 : 12.09.2021
===============================================================================
This module handles RTC time related functions and Controls the basic IRQs
to control epoch time, synchronizing epoch time and calibrating RTC to XTAL
timing.
One timer is used to count ms and IRQ is triggered every second (controlled by XTAL)
The RTC IRQ is used to count/check the RTC Seconds.
===============================================================================
*/
#include "obc_time.h"
#include <ado_modules.h>
#include "../../ado/obc_checksums.h"

#define C_RESET_MS_OFFSET 22			// TODO: messure exact start offset

static const int timeMonthOffsetDays[2][13] = {
   {0,   0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},  // non leap year
   {0,   0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}   // leap year
};

static inline juliandayfraction TimeMsToDayFractions(uint32_t ms) {
    return (((double)ms)/86400000.0);
}


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
	uint32_t	 resetCount;
	uint32_t	 rtcDate;
	uint32_t	 rtcTime;
	bool		 crcError;
	rtc_status_t oldStatus;
	rtc_status_t newStatus;
	uint8_t		 oscDelay;
} rtc_event_init_t;

typedef struct {
	uint64_t	 rtcDateTime;
} rtc_event_xtalstarted_t;



// Prototypes
void RtcClearGpr();
uint8_t RtcReadGpr(rtc_gpridx_t idx);
bool RtcIsGprChecksumOk(void);
void RtcWriteGpr(rtc_gpridx_t idx, uint8_t byte);
uint32_t rtc_get_time(void);
uint32_t rtc_get_date(void);
uint64_t rtc_get_datetime(void);
void TimBlockMs(uint16_t ms);
uint32_t RtcReadGpr32(rtc_gpridx_t idx);
void RtcWriteGpr32(rtc_gpridx_t idx, uint32_t value);
void TimeSetUtc2(RTC_TIME_T *fullTime);
void TimeSetUtc1(uint16_t year, uint8_t month, uint8_t dayOfMonth, uint8_t hour, uint8_t min, uint8_t sec);

// Module Variables
static obc_tim_systemtime_t ObcSystemTime;

// The RIT Timer IRQ counts the ms register
void RIT_IRQHandler(void) {
    LPC_RITIMER->CTRL |= 0x0001;                    // Clear the RITINT flag;
    ObcSystemTime.msAfterStart++;
}

// The RTC Seconds Timer IRQ.
void RTC_IRQHandler(void)
{
	Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE);
	Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);

	// TODO: here we can synchronize RTC with 'XTAL msAfterReset time (counted by  RIT IRQ )
//	if (rtc_currentDayOfMonth == 0) {
//		rtc_currentDayOfMonth = LPC_RTC->TIME[RTC_TIMETYPE_DAYOFMONTH];
//	}
//	if (rtc_currentDayOfMonth !=  LPC_RTC->TIME[RTC_TIMETYPE_DAYOFMONTH]) {
//		// Day changed
//		rtc_currentDayOfMonth = LPC_RTC->TIME[RTC_TIMETYPE_DAYOFMONTH];
//		rtc_dayChangedAtSeconds = secondsAfterReset;
//		rtc_dayChanged = true;
//	}


	rtc_status_t status = RtcReadGpr(RTC_GPRIDX_STATUS);
	if (status == RTC_STAT_XTAL_ERROR) {
		// There was an error while init (no RTC Clock running). Now it seems to be ok (otherwise there would not be an IRQ)
		// clear the error bit now.
		if (LPC_RTC->RTC_AUX & RTC_AUX_RTC_OSCF)
		{
			LPC_RTC->RTC_AUX &= RTC_AUX_RTC_OSCF;	// Clear the error by writing to this bit now.
		}
		// TODO: Is this assumption here correct? We had an unknown time from init until now but it seems to run from here on.
		//       For sure it is not synchronized any more.....
		RtcWriteGpr(RTC_GPRIDX_STATUS, RTC_STAT_UNKNOWN);

		rtc_event_xtalstarted_t eventdata;
		eventdata.rtcDateTime = rtc_get_datetime();
		SysEvent(MODULE_ID_TIME, EVENT_ERROR, EVENT_TIM_XTALSTARTED, &eventdata, sizeof(rtc_event_xtalstarted_t));
	}
}

void tim_init (void *dummy) {
	// prepare the init sys event used later on.
	rtc_event_init_t initEventArgs;
	initEventArgs.newStatus = RTC_STAT_UNKNOWN;
	initEventArgs.oldStatus = RtcReadGpr(RTC_GPRIDX_STATUS);
	initEventArgs.crcError  = !RtcIsGprChecksumOk();
	initEventArgs.resetCount = 0;

	// we use RIT IRQ to count ms after reset. It counts PCLK cycles
	// This Repetive Interrupt Timer us activated by default with reset.
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_RIT);
	Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_RIT, SYSCTL_CLKDIV_1);

	ObcSystemTime.msAfterStart = C_RESET_MS_OFFSET;

	LPC_RITIMER->COMPVAL = (SystemCoreClock/1000) - 1;   					// We want an IRQ every 0,001 Seconds.
	LPC_RITIMER->COUNTER = 0;
	LPC_RITIMER->CTRL = RIT_CTRL_ENCLR | RIT_CTRL_ENBR | RIT_CTRL_TEN;
	NVIC_EnableIRQ(RITIMER_IRQn);

	// Initialize thr RTC Clock module
	// TODO: maybe we can avoid disable-enable by calling init here if we know it is already running !?
	Chip_RTC_Init(LPC_RTC);
	Chip_RTC_Enable(LPC_RTC, ENABLE);		// We have to enable the RTC

	// Wait until XTAL is running. Check by Clearing the OSC Error bit and see if it comes on again
	// We allow 400ms to stabilize. (On EM2 after deep power off it takes aprx. 300ms
	uint32_t tryUntil = ObcSystemTime.msAfterStart + 400;
	initEventArgs.oscDelay = 0;
	while(tryUntil > ObcSystemTime.msAfterStart) {
		if (!(LPC_RTC->RTC_AUX & RTC_AUX_RTC_OSCF)) {
			// If there is no OSC error pending, we are ok to go
			initEventArgs.oscDelay = ObcSystemTime.msAfterStart >> 1;
			break;
		}
		LPC_RTC->RTC_AUX &= RTC_AUX_RTC_OSCF;	// Clear the OSC Error bit
		TimBlockMs(5);							// and give it some time to re-appear....
	}

	// Now there shouldn't be a (new) error bit set here.
	if (LPC_RTC->RTC_AUX & RTC_AUX_RTC_OSCF)
	{
		// If so (tested with defect OBC board!) we really have no RTC OSC running!
		LPC_RTC->RTC_AUX &= RTC_AUX_RTC_OSCF;	// Clear the error  bit.
		initEventArgs.newStatus = RTC_STAT_XTAL_ERROR;
		RtcWriteGpr(RTC_GPRIDX_STATUS, initEventArgs.newStatus);
		// No need to continue with RTC Init here.....
		SysEvent(MODULE_ID_TIME, EVENT_FATAL, EVENT_TIM_INITIALIZED, &initEventArgs, sizeof(rtc_event_init_t));
		return;
	}

	// A valid status always has 4 higher bits equal to low higer bits
	// As Error uses 0xE0 this is not as valid state and we try to re-init everything.
	bool validStatus = ((initEventArgs.oldStatus & 0x0F) == ((initEventArgs.oldStatus >> 4) & 0x0F));

	// after deep power off (no bat left) we get random content in gpr
	RTC_TIME_T 	time;
	if (initEventArgs.crcError || !(validStatus)) {
		// crc wrong or status invalid -> we can init from scratch
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

		initEventArgs.newStatus = RTC_STAT_RESETDEFAULT;
		RtcWriteGpr(RTC_GPRIDX_STATUS, initEventArgs.newStatus);
	} else {
		// We can rely on GPR holding correct status and Reset Count
		ObcSystemTime.epochNumber = RtcReadGpr32(RTC_GPRIDX_RESETCOUNTER32);
		ObcSystemTime.epochNumber++;
		RtcWriteGpr32(RTC_GPRIDX_RESETCOUNTER32, ObcSystemTime.epochNumber);
		initEventArgs.resetCount = ObcSystemTime.epochNumber;
		if (initEventArgs.oldStatus == RTC_STAT_SYNCHRONIZED) {
			// RTC was synchronized before Reset so we can use it to synchronize again.
			Chip_RTC_GetFullTime(LPC_RTC, &time);
			TimeSetUtc2(&time);
		} else if (initEventArgs.oldStatus == RTC_STAT_RESETDEFAULT) {
			// Last epoch run with its defaults so from now on we only know that we 'continue' running.
			initEventArgs.newStatus = RTC_STAT_RUNNING;
			RtcWriteGpr(RTC_GPRIDX_STATUS, initEventArgs.newStatus);
		} else {
			initEventArgs.newStatus = initEventArgs.oldStatus;
		}
	}

	// Enable the 1 Second IRQ
	Chip_RTC_CntIncrIntConfig(LPC_RTC, RTC_AMR_CIIR_IMSEC,  ENABLE);
	//NVIC_SetPriority(RTC_IRQn, RTC_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(RTC_IRQn); /* Enable interrupt */

	// Send the Timer Init SysEvent with current RTC-timestamp.
	initEventArgs.rtcDate =  rtc_get_date();
	initEventArgs.rtcTime =  rtc_get_time();
	SysEvent(MODULE_ID_TIME, EVENT_INFO, EVENT_TIM_INITIALIZED, &initEventArgs, sizeof(rtc_event_init_t));
}

void tim_main (void) {
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
		tmpBytes[3] = CRC8((uint8_t*)&(LPC_RTC->GPREG), 19) + 1;
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
	tmpBytes[3] = CRC8((uint8_t*)&(LPC_RTC->GPREG), 19) + 1;
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
	uint8_t crc = CRC8(gprbase, 19) + 1;
	return (gprbase[19] == crc);
}


/* Returns the current RTC time according to UTC.
 * Parameters: 	none
 * Return value: time as uint32 with a decimal number formated as HHMMSS.
 */
uint32_t rtc_get_time(void)
{
	RTC_TIME_T tim;
	Chip_RTC_GetFullTime(LPC_RTC, &tim);

	return (tim.time[RTC_TIMETYPE_SECOND] + tim.time[RTC_TIMETYPE_MINUTE] * 100 + tim.time[RTC_TIMETYPE_HOUR] * 10000);
}

/* Returns the current RTC date
 * Parameters: 	none
 * Return value: date as uint32 with a decimal number formated as YYYYmmDD.
 */
uint32_t rtc_get_date(void)
{
	/* Returns the current RTC date according to UTC.
	 * Parameters: 	none
	 * Return value: date as uint32 with a decimal number formated as YYYYmmdd.
	 */
	RTC_TIME_T tim;
	Chip_RTC_GetFullTime(LPC_RTC, &tim);

	return (tim.time[RTC_TIMETYPE_DAYOFMONTH] + tim.time[RTC_TIMETYPE_MONTH] * 100 + (tim.time[RTC_TIMETYPE_YEAR]) * 10000);
}

/* Returns the current RTC date and time.
 * Parameters: 	none
 * Return value: date/tme as uint64 with a decimal number formated as YYYYmmDDHHMMSS.
 */
uint64_t rtc_get_datetime(void) {
	return ((uint64_t)rtc_get_date()) * 1000000 + (uint64_t)rtc_get_time();
}

void TimBlockMs(uint16_t ms) {
	uint32_t waituntil = ObcSystemTime.msAfterStart + ms;
	while (ObcSystemTime.msAfterStart<waituntil) {
	}
}

//
//void TimeGetCurrentSystemTime(ado_tim_systemtime_t *sysTime) {
//    sysTime->epochNumber = adoSystemTime.epochNumber;
//    sysTime->msAfterStart = adoSystemTime.msAfterStart;
//    sysTime->utcOffset.year = adoSystemTime.utcOffset.year;
//    sysTime->utcOffset.dayOfYear = adoSystemTime.utcOffset.dayOfYear;
//}
//

void TimeSetUtc1(uint16_t year, uint8_t month, uint8_t dayOfMonth, uint8_t hour, uint8_t min, uint8_t sec) {
    ObcSystemTime.utcOffset.year = year;

    // First we get the day# depending of leap year
    int leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    juliandayfraction day = timeMonthOffsetDays[leap][month] + dayOfMonth;
    // Then we add the time fraction
    day += ((sec / 60.0 + min) / 60.0 + hour) / 24.0;

    // To store the offset to this time we subtract current ms.
    day -= TimeMsToDayFractions(ObcSystemTime.msAfterStart);
    ObcSystemTime.utcOffset.dayOfYear = day;
}

inline void TimeSetUtc2(RTC_TIME_T *fullTime) {
    TimeSetUtc1(fullTime->time[RTC_TIMETYPE_YEAR],
                fullTime->time[RTC_TIMETYPE_MONTH],
                fullTime->time[RTC_TIMETYPE_DAYOFMONTH],
                fullTime->time[RTC_TIMETYPE_HOUR],
                fullTime->time[RTC_TIMETYPE_MINUTE],
                fullTime->time[RTC_TIMETYPE_SECOND]);
}

//void TimeGetCurrentUtcTime(RTC_TIME_T *fullTime) {
//    uint16_t year = adoSystemTime.utcOffset.year;
//    fullTime->time[RTC_TIMETYPE_YEAR] = year;
//    if (year == 0) {
//        fullTime->time[RTC_TIMETYPE_MONTH] = 0;
//        fullTime->time[RTC_TIMETYPE_DAYOFMONTH] = 0;
//        fullTime->time[RTC_TIMETYPE_HOUR] = 0;
//        fullTime->time[RTC_TIMETYPE_MINUTE] = 0;
//        fullTime->time[RTC_TIMETYPE_SECOND] = 0;
//    } else {
//        double currentTime =  TimeGetCurrentDayOfYear();
//        // Date conversionm
//        int day = (int)floor(currentTime);
//        int leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
//        for (uint8_t month = 1; month<=12; month++) {
//            if (timeMonthOffsetDays[leap][month] > day) {
//                fullTime->time[RTC_TIMETYPE_MONTH] = month - 1;
//                fullTime->time[RTC_TIMETYPE_DAYOFMONTH] = day - timeMonthOffsetDays[leap][month - 1];
//                break;
//            }
//        }
//        // Time conversion
//        double temp = (currentTime - day) * 24.0;
//        fullTime->time[RTC_TIMETYPE_HOUR]  = (uint32_t)floor(temp);
//        temp = (temp - fullTime->time[RTC_TIMETYPE_HOUR]) * 60.0;
//        fullTime->time[RTC_TIMETYPE_MINUTE]  = (uint32_t)floor(temp);
//        fullTime->time[RTC_TIMETYPE_SECOND]  = (temp - fullTime->time[RTC_TIMETYPE_MINUTE]) * 60.0;
//    }
//}
//
//juliandayfraction TimeGetCurrentDayOfYear() {
//    juliandayfraction currentTime = adoSystemTime.utcOffset.dayOfYear;
//    currentTime += TimeMsToDayFractions(adoSystemTime.msAfterStart);
//    return currentTime;
//}
//
//ado_timestamp TimeGetCurrentTimestamp() {
//    return adoSystemTime.msAfterStart;
//}

