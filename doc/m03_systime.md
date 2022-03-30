OBC Timestamps/RTC and related time formats
===========================================

systime
-------

is counted in ms after reset. This counter is counted by IRQ and never changed or adjusted by any other means. So it can be used as continuous timestamp with unsigned 32 bit size. 
This gives a timestamp which runs without overrun for almost 50 days of operation without reset. 

To get a unique mission timestamp which will 'never' repeat itself one have to additionally store the current reset counter (and somehow trigger a reset at least all 49 days).
The Reset counter is also a 32bit unsigned counter and gets persisted in both the RTC general purpose registers (this are battery backuped during resets)
and separately in all 6 MRAM chips available on the OBC board. With this we can guarantee that a systime together with the reset counter is a unique and non-repeatable timestamp.


UTC time
--------

In order to get a valid UTC time - needed for operation and orbit prediction - there has to be some synchronization from the OBC outside world.
Synchronization can be done by 
- GPS connected to OBC (by UART RX and/or TX + PPS signal)
- reception of a Sync command (either by radio transmission from ground station or from connected equipment on the debug/umbilical UART.

Another source of synchronization can be the battery buffered on board RTC of the OBC. This can survive up to 10 minutes and will be good enough to resynchronize the UTC time after short resets without longer power loss.


Precision and accuracy
-----------------------

systime is driven by the XTAL of the LPC1769.
onboard RTC is driven by separate RTC quartz.
GPS can be connected with additional PPS (pulse per second). This will be highest precision of synchronization, once this signal gets processed with a GPIO-Interrupt routine.

all accuracy and temp dependent drifts and offsets have to be investigated in more detail and incorporated in later software versions.



time date and timestamp formats
-------------------------------

obc_time module has the function timGetSystime() which delivers current ms after reset as obc_systime32_t (an alias for uint32_t).

To get the UTC time (if synchronized) the function timGetUTCTime() can be used. It gives a
obc_utc_fulltime_t 	structure containing following members:

	uint32_t	 		rtcDate;		// UTC Date in format YYYYMMDD	(direct from hardware RTC)
	uint32_t	 		rtcTime;		// UTC Time in format HHMMSS	(direct from hardware RTC)
	juliandayfraction	tleDay;			// UTC Time in TLE dayOfYear format (calculated from XTAL Systime and last UTC Sync command)
	double				currentDiff;    // in Seconds. The drift of XTAL Clock vs RTC since last Sync command. 


 juliandayfraction is a double which follows the 'EPOCH' definition of the 'Field 8 in Line 1' of a TLE orbital elements definition used by https://celestrak.com/ (see also: https://en.wikipedia.org/wiki/Two-line_element_set )
 The integer part is the day of the year and the remaining part is fractional portion of the day (0.5 -> 12:00 noon)     



 
