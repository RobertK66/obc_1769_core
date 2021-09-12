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

#include "../../ado/obc_checksums.h"

typedef enum {
	RTC_STAT_SYNCHRONIZED	= 0x11,
	RTC_STAT_RUNNING		= 0x22,
	RTC_STAT_RESETDEFAULT	= 0x33,
	RTC_STAT_UNKNOWN		= 0xDD,
	RTC_STAT_XTAL_ERROR		= 0xEE,
} rtc_status_t;

// Usage of 19 bytes General Purpose Register
typedef enum {
	RTC_GPRIDX_STATUS = 0,

	RTC_GPRIDX_CRC8 = 19
} rtc_gpridx_t;

typedef struct {
	bool		 crcError;
	rtc_status_t oldStatus;
	rtc_status_t newStatus;
	uint64_t	 rtcDateTime;
} rtc_event_init_t;


// Prototypes
void RtcInitializeGpr();
uint8_t RtcReadGpr(rtc_gpridx_t idx);
bool RtcIsGprChecksumOk(void);
void RtcWriteGpr(rtc_gpridx_t idx, uint8_t byte);
uint32_t rtc_get_time(void);
uint32_t rtc_get_date(void);
uint64_t rtc_get_datetime(void);
void TimBlockMs(uint8_t ms);

// Variables
LPC_TIMER_T *timMsTimer = NULL;
uint32_t rtc_epoch_time;

void tim_init (void *dummy) {

	// We use timer0 fixed here for now TODO: make general timer irq register loike for UART in ado .....
	timMsTimer = LPC_TIMER0;
	uint32_t prescaler;
	prescaler = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_TIMER0) / 1000 - 1; 	/* 1 kHz timer frequency */

	/* Enable this timer counting milliseconds */
	Chip_TIMER_Init(timMsTimer);
	Chip_TIMER_Reset(timMsTimer);
	Chip_TIMER_MatchEnableInt(timMsTimer, 0);

	Chip_TIMER_PrescaleSet(timMsTimer, prescaler),
	Chip_TIMER_SetMatch(timMsTimer, 0, 999 );									// Count ms from 0 to 999
	Chip_TIMER_ResetOnMatchEnable(timMsTimer, 0);
	Chip_TIMER_Enable(timMsTimer);

	/* Enable timer interrupt */
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);

	// We use clockout but disable after reset.
//	Chip_Clock_DisableCLKOUT();

	// Initilaice thr RTC Clock module
	RTC_TIME_T 	tim;
	Chip_RTC_Init(LPC_RTC);

	rtc_event_init_t initEvent;
	initEvent.newStatus = RTC_STAT_UNKNOWN;
	initEvent.oldStatus = RtcReadGpr(RTC_GPRIDX_STATUS);
	initEvent.crcError  = !RtcIsGprChecksumOk();

	/* If the Gpr have content with correct crc we initialize from there  */
	if (initEvent.crcError || (initEvent.oldStatus == 0x61)) {
		RtcInitializeGpr();

		/* RTC module has been reset, time and data invalid */
		/* Set to default values */
		tim.time[RTC_TIMETYPE_SECOND] = 0;
		tim.time[RTC_TIMETYPE_MINUTE] = 0;
		tim.time[RTC_TIMETYPE_HOUR] = 0;
		tim.time[RTC_TIMETYPE_DAYOFMONTH] = 1;
		tim.time[RTC_TIMETYPE_DAYOFWEEK] = 1;
		tim.time[RTC_TIMETYPE_DAYOFYEAR] = 1;
		tim.time[RTC_TIMETYPE_MONTH] = 1;
		tim.time[RTC_TIMETYPE_YEAR] = 1001;
		Chip_RTC_SetFullTime(LPC_RTC, &tim);

		initEvent.newStatus = RTC_STAT_RESETDEFAULT;
		//rtc_backup_reg_reset(1);
		//rtc_backup_reg_set(1, 0x00);	// RTC is definitely not in sync
		//obc_status.rtc_synchronized = 0;
	}

	// Check if RTC XTAL is running.
	Chip_RTC_Enable(LPC_RTC, ENABLE);		// We have to enable the RTC
	TimBlockMs((uint8_t)400);				// This 400ms are needed to get a stable RTC OSC after Power on (tested with LPCX board).
	LPC_RTC->RTC_AUX &= RTC_AUX_RTC_OSCF;	// Now clear the OSC Error bit (its set to 1 on each RTC power on)
	TimBlockMs(5);							// and lets wait another short time.

	// Now there shouldn't be a (new) error bit set here.
	if (LPC_RTC->RTC_AUX & RTC_AUX_RTC_OSCF)
	{
		// If so (tested with defect OBC board!) we really have no RTC OSC running!
		LPC_RTC->RTC_AUX &= RTC_AUX_RTC_OSCF;	// Clear the error  bit.
		//printf("RTC: Oscillator error!\n");
		initEvent.newStatus = RTC_STAT_XTAL_ERROR;
	}

	//rtc_calculate_epoch_time();
	RtcWriteGpr(RTC_GPRIDX_STATUS, initEvent.newStatus);

	Chip_RTC_CntIncrIntConfig(LPC_RTC, RTC_AMR_CIIR_IMSEC,  ENABLE);

	//NVIC_SetPriority(RTC_IRQn, RTC_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(RTC_IRQn); /* Enable interrupt */

	initEvent.rtcDateTime =  rtc_get_datetime();
	SysEvent(&initEvent, sizeof(rtc_event_init_t));

}


uint32_t ms = 0;
uint32_t seconds = 0;

void tim_main (void) {
	if (ms > 0) {
		ms = 0;
	}
}


// This 'overwrites' the weak definition of this IRQ in cr_startup_lpc175x_6x.c
void TIMER0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(timMsTimer, 0)) {
		Chip_TIMER_ClearMatch(timMsTimer, 0);
		seconds++;
	}
}


void RTC_IRQHandler(void)
{

	ms = timMsTimer->TC;
//	timMsTimer->TC = 0; // Synchronize ms-timer to RTC seconds TODO....
	//ms = 0;


	Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE);
	Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);
//
//	if (rtc_currentDayOfMonth == 0) {
//		rtc_currentDayOfMonth = LPC_RTC->TIME[RTC_TIMETYPE_DAYOFMONTH];
//	}
//	if (rtc_currentDayOfMonth !=  LPC_RTC->TIME[RTC_TIMETYPE_DAYOFMONTH]) {
//		// Day changed
//		rtc_currentDayOfMonth = LPC_RTC->TIME[RTC_TIMETYPE_DAYOFMONTH];
//		rtc_dayChangedAtSeconds = secondsAfterReset;
//		rtc_dayChanged = true;
//	}

	rtc_epoch_time++; /* increment QB50 s epoch variable and calculate UTC time */	// TODO ...

	rtc_status_t status = RtcReadGpr(RTC_GPRIDX_STATUS);
	if (status == RTC_STAT_XTAL_ERROR) {
		// There was an error while init (no RTC Clock running). Now it seems to be ok (otherwise there would not be an IRQ)
		// clear the error bit now.
		if (LPC_RTC->RTC_AUX & RTC_AUX_RTC_OSCF)
		{
			LPC_RTC->RTC_AUX &= RTC_AUX_RTC_OSCF;	// Clear the error by writing to this bit now.
		}
		// TODO: is this assumption here correct. We had an unknown time from init until now but it seems to run from here on.
		//       For sure it is not synchronized any more.....
		RtcWriteGpr(RTC_GPRIDX_STATUS, RTC_STAT_UNKNOWN);
	}
	/* Do powersave modes or other things here */
	/*if (obc_status.obc_powersave)
	{
		// Reset watchdog regularly if OBC is in powersave
		WDT_Feed();
	}*/
}


void RtcInitializeGpr() {
	for (int i=0;i<19;i++) {
		RtcWriteGpr(i, 'a' + i);		// Some Recognizable pattern ;-).
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

uint8_t RtcReadGpr(rtc_gpridx_t idx) {
	// Other than writing, for reading a uint8 ptr is good enough to get all bytes separately.
	uint8_t *gprbase = (uint8_t *)(&(LPC_RTC->GPREG));
	if (idx>19) {
		//TODO: log an 'out of range' event here !!!
		idx = 19;
	} else if (idx < 0) {
		//TODO: log an 'out of range' event here !!!
		idx = 0;
	}
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
 * Return value: date as uint32 with a decimal number formated as HHMMSS.
 */
uint32_t rtc_get_time(void)
{
	RTC_TIME_T tim;
	Chip_RTC_GetFullTime(LPC_RTC, &tim);

	// TODO: tim0 is not used for ms counting yet.....
	return (tim.time[RTC_TIMETYPE_SECOND] + tim.time[RTC_TIMETYPE_MINUTE] * 100 + tim.time[RTC_TIMETYPE_HOUR] * 10000);
	//return (0 + tim.time[RTC_TIMETYPE_SECOND] * 1000 + tim.time[RTC_TIMETYPE_MINUTE] * 100000 + tim.time[RTC_TIMETYPE_HOUR] * 10000000);
	//return ((LPC_TIM0->TC) + tim.SEC * 1000 + tim.MIN * 100000 + tim.HOUR * 10000000);
}

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

/* Returns the current RTC date and time according to UTC.
 * Parameters: 	none
 * Return value: date as uint64 with a decimal number formated as YYYYmmddHHMMSS.
 */
uint64_t rtc_get_datetime(void) {
	return ((uint64_t)rtc_get_date()) * 1000000 + (uint64_t)rtc_get_time();
}

void TimBlockMs(uint8_t ms) {
	uint64_t loop = ms * 100000;
	while (loop>0) {
		loop--;
	}
//	uint32_t cr = Chip_Clock_GetPeripheralClockRate(MAINLOOP_TIMER_PCLK);
//	uint32_t cntToWait = ms * cr/1000 ;
//
//	LPC_TIMER_T *mlTimer = MAINLOOP_TIMER;
//	uint32_t currTimerReg = mlTimer->TC;
//
//	uint32_t waitFor = currTimerReg + cntToWait;
//	if (waitFor >= cr/1000 * TIM_MAIN_TICK_MS) {
//		// This is higher than the max value which TC gets before the match for the interrupt resets it to 0!
//		// so we correct the wait time for the time left after the 'overrun/reset'.
//		waitFor = cntToWait - (cr/1000 * TIM_MAIN_TICK_MS - currTimerReg);
//		// wait until TC gets reset
//		while (mlTimer->TC > currTimerReg);
//		// and then ...
//	} else if (waitFor < currTimerReg) {
//		// This only can happen when there is no Match reseting the TC. And the current TC + TimeToWait has an uint32 overrun.
//		// As coded now this will/should never happen (until somebody sets TIM_MAIN_TICK_MS to > 178000....)
//		// if it happens we first wait for the TC to overflow
//		while (mlTimer->TC > currTimerReg);
//		// and then ...
//	}
//
//
//	// ... lets wait until calculated time is reached.
//	while (mlTimer->TC < waitFor);
}
