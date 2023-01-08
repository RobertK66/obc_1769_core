/*
 * climb_gps.c
 *
 *  Created on: 27.02.2022
 *      Author: Robert
 */
#include "climb_gps.h"

#include <string.h>
#include <stdlib.h>

#include <ado_uart.h>
#include <ado_modules.h>

#include "obc_time.h"
#include "../modules_globals.h"

// Event structs - used as API to ground station (or debug IF) only
//typedef struct __attribute__((packed)) {
//	char	talker;
//	uint8_t	msgNr;
//	uint8_t	msgCnt;
//	uint8_t	nrOfSatellites;
//} gps_gsvmsg_t;
typedef struct __attribute__((packed)) {
	uint8_t gpsSeen;
	uint8_t glonassSeen;
} gps_satcnt_t;

typedef struct __attribute__((packed)) {
	char		talker;
	char		fix;
	uint32_t	utcTimeSec;
	uint16_t	utcTimeMs;
	double lat; // decimal degree format
	double lon; // decimal degree
	double alt; // meters
} gps_ggamsg_t;

typedef struct __attribute__((packed)) {
	char	talker;
	char	mode;
	char	fixstatus;
} gps_gsamsg_t;


typedef struct __attribute__((packed)) {
	char	talker;
	char	posmode;
} gps_vtgmsg_t;

typedef struct __attribute__((packed)) {
	char		talker;
	char		valid;
	char		posmode;
	uint32_t	utcTimeSec;
	uint16_t	utcTimeMs;
} gps_rmcmsg_t;


// local/module variables

static gps_initdata_t *gpsInitData;

static uint8_t gpsGPSSatsInView = 0;
static uint8_t gpsGLONASSSatsInView= 0;

#define GPS_TIMESYNC_COUNT	3
typedef struct __attribute__((packed)) {
	uint16_t	year;
	juliandayfraction gpsUTC[GPS_TIMESYNC_COUNT];
	obc_systime32_t sysTime[GPS_TIMESYNC_COUNT];
} gps_syncdata_t;
static uint8_t gpsSyncIdx = 0;
static gps_syncdata_t gpsSyncData;

// local prototypes
void gpsUartIRQ(LPC_USART_T *pUART);
void gpsProcessRxByte(uint8_t rxByte);
bool gpsProcessNmeaMessage(int argc, char *argv[]);


// ************************** TX Circular byte Buffer Begin ********************
#define GPS_TX_BUFFERSIZE	200
static uint8_t 		   gpsTxBuffer[GPS_TX_BUFFERSIZE];
static uint8_t         gpsTxWriteIdx = 0;
static uint8_t         gpsTxReadIdx  = 0;
static bool		  	   gpsTxBufferFull = false;
static bool		  	   gpsFirstByteAfterReset = true;

static bool inline gpsTxBufferEmpty() {
	if (gpsTxBufferFull) {
		return false;
	} else {
		return (gpsTxReadIdx == gpsTxWriteIdx);
	}
}

static bool inline gpsTxAddByte(uint8_t b) {
	if (!gpsTxBufferFull) {
		gpsTxBuffer[gpsTxWriteIdx++] = b;
		if (gpsTxWriteIdx >= GPS_TX_BUFFERSIZE) {
			gpsTxWriteIdx = 0;
		}
		if (gpsTxWriteIdx == gpsTxReadIdx) {
			gpsTxBufferFull = true;
		}
		return true;
	} else {
		return false;
	}
}

static uint8_t inline gpsTxGetByte(void) {
	uint8_t retVal = gpsTxBuffer[gpsTxReadIdx++];
	if (gpsTxReadIdx >= GPS_TX_BUFFERSIZE) {
		gpsTxReadIdx = 0;
	}
	gpsTxBufferFull = false;
	return retVal;
}

// ************************** TX Circular byte Buffer End ********************

void gpsInit (void *initData) {
	gpsInitData = (gps_initdata_t*) initData;

	// Switch the 'enable' pin to low (no internal Volage regulator needed
	Chip_GPIO_SetPinOutLow(LPC_GPIO, gpsInitData->pEnablePin->pingrp, gpsInitData->pEnablePin->pinnum);

	// Init UART
	InitUart(gpsInitData->pUart, 9600, gpsUartIRQ);
	gpsFirstByteAfterReset = true;

}

void gpsMain (void) {
	// Uart Rx
	int32_t stat = Chip_UART_ReadLineStatus(gpsInitData->pUart);
	if (stat & UART_LSR_RDR) {
		// there is a byte available. Lets read and process it.
		uint8_t b = Chip_UART_ReadByte(gpsInitData->pUart);
		gpsProcessRxByte(b);
	}
}

void gpsUartIRQ(LPC_USART_T *pUART) {
	if (gpsInitData->pUart->IER & UART_IER_THREINT) {
		// Transmit register is empty now (byte was sent out)
		if (gpsTxBufferEmpty() == false) {
			// Send next byte
			uint8_t nextByte = gpsTxGetByte();
			Chip_UART_SendByte(gpsInitData->pUart, nextByte);
		} else {
			// No more bytes available -> stop the THRE IRQ.
			Chip_UART_IntDisable(gpsInitData->pUart, UART_IER_THREINT);
		}
	}
}



bool gpsProcessNmeaMessage(int argc, char *argv[]) {
	bool processed = false;
	char msg[8];
	strncpy(msg, argv[0], 8);

	if (strncmp(msg, "PMTK",4)==0) {
		// PNTK Messages are responses to PMTK commands (or sent on Power on)
		processed = true;
	} else if (strncmp(&msg[2], "GSV", 3)==0) {
		// GxGSV Message shows number of satellites in View.
		// We are only interested in total number of sats in view. Not in details and sat ids....
		uint8_t nrOfSatellites = atoi(argv[3]);
		// We have one value for GPS and one for GLONASS
		uint8_t *storedValuePtr;
		if (msg[1] == 'P') {
			processed = true;
			storedValuePtr = &gpsGPSSatsInView;
		} else if (msg[1] == 'L') {
			processed = true;
			storedValuePtr = &gpsGLONASSSatsInView;
		}
		if (processed) {
			if (*storedValuePtr != nrOfSatellites) {
				// Value changed. Store it and trigger event to send new numbers.
				*storedValuePtr = nrOfSatellites;
				gps_satcnt_t satcnt;
				satcnt.gpsSeen = gpsGPSSatsInView;
				satcnt.glonassSeen = gpsGLONASSSatsInView;
				SysEvent(MODULE_ID_GPS, EVENT_INFO, EID_GPS_SATELLITES_IN_SIGHT, &satcnt, sizeof(satcnt));
			}
		}
	} else if (strncmp(&msg[2], "GGA", 3)==0) {
		// xxGGA Message shows essential fix data
		processed = true;
		char temp_dd[2]; //string containing   DD part of latitude neglecting MM.MMM part in DDMM.MMM
		char temp_mmdotmmm[6]; // string containing   MM.MMM part of latitude neglecting DD part in DDMM.MMM
		double latlon_dd;
		double latlon_mm;


		gps_ggamsg_t ggamsg;
		ggamsg.talker = msg[1];					// 'P' for GPS, 'L' for GLONASS, 'N' for 'generic' method?
		ggamsg.fix = argv[6][0];				// '0' invalid, '1' GNSS fix, '2' DGPS fix, ...
		ggamsg.utcTimeSec = atoi(argv[1]);		// argv[1] is format 'hhmmss.sss' -> to int gives hhmmss as integer
		ggamsg.utcTimeMs = atoi(&(argv[1][7])); // argv[1] is format 'hhmmss.sss' -> to int from position [1][7] gives ms as integer

		// Parse latitude and longitude // Inpur of NMEA string is DDMM.MMM  format
		// Note ! Some GGA are DDMM.MMM others are DDMM.MMMMMMM
		// Since we dont have a GNSS device defined currently - I do parser for DDMM.MMM  which should work with DDMM.MMMMMMM also
		// parsing for additional .___MMMM can be easily added later if device will supply constant format.

		// example_nmea= "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0 0031*4F"

		temp_dd[0] = argv[2][0];
		temp_dd[1] = argv[2][1];

		latlon_dd = atof(temp_dd); // convert char to double


		temp_mmdotmmm[0] =argv[2][2];
		temp_mmdotmmm[1] =argv[2][3];
		temp_mmdotmmm[2] = (char)0x2e; // 0x2e ="."
		temp_mmdotmmm[3] = argv[2][5];
		temp_mmdotmmm[4] = argv[2][6];
		temp_mmdotmmm[5] = argv[2][7];

		latlon_mm = atof(temp_mmdotmmm); // convert char to double

		ggamsg.lat = latlon_dd + latlon_mm/60; // format is decimal degrees not dd mm ss

		if (strncmp(argv[3], "N", 1)==0){
			// if N - do nothing decimal degrees are already positive
		}
		if (strncmp(argv[3], "S", 1)==0){
			ggamsg.lat = (-1)*ggamsg.lat;
				}


		// Longitude parser // the same // assume DDMM.MMM format (which will work with DDMM.MMMMMMM messages also)

		temp_dd[0] = argv[4][0];
		temp_dd[1] = argv[4][1];



		temp_mmdotmmm[0] =argv[4][2];
		temp_mmdotmmm[1] =argv[4][3];
		temp_mmdotmmm[2] = (char)0x2e; // 0x2e ="."
		temp_mmdotmmm[3] = argv[4][5];
		temp_mmdotmmm[4] = argv[4][6];
		temp_mmdotmmm[5] = argv[4][7];

		latlon_dd = atof(temp_dd); // convert char to double
		latlon_mm = atof(temp_mmdotmmm); // convert char to double

		ggamsg.lon = latlon_dd + latlon_mm/60; // format is decimal degrees not dd mm ss

		if (strncmp(argv[5], "E", 1)==0){
			// if N - do nothing decimal degrees are already positive
		}
		if (strncmp(argv[5], "W", 1)==0){
			ggamsg.lon = (-1)*ggamsg.lon;
		}


		// Parse altitude
		ggamsg.alt = atof(argv[9]); // altitude above mean sea level
		if (strncmp(argv[10], "M", 1)==0){
			// unit is already in meters !
		}
		else{
			// WARNING ! Assume that unit output will change during on orbit operation
			// Is this possible that output unit can change ?
			// if so ! we have to convert unit to meters and store it.

			// Check which other units device can possibly output !
			// For each possible unit do the conversion to meters (or other unit that we would like to use)

		}

		SysEvent(MODULE_ID_GPS, EVENT_INFO, EID_GPS_NMEA_MSG_GGA, &ggamsg, sizeof(ggamsg));


	} else if (strncmp(&msg[2], "GSA", 3)==0) {
		// xxGSA Message shows fix details
		processed = true;
//		gps_gsamsg_t gsamsg;
//		gsamsg.talker = msg[1];					// 'P' for GPS, 'L' for GLONASS, 'N' for 'generic' method?
//		gsamsg.mode = argv[1][0];				// 'M' for manuell forced 2D/3D switch, 'A' allowed to switch 2D/3D automatically
//		gsamsg.fixstatus = argv[2][0];			// '1' No Fix, '2' 2D fix, '3' 3D fix
//		SysEvent(MODULE_ID_GPS, EVENT_INFO, EID_GPS_NMEA_MSG_GSA, &gsamsg, sizeof(gsamsg));
	} else if (strncmp(&msg[2], "VTG", 3)==0) {
		// xxVTG Message shows track mode and ground speed
		processed = true;
//		gps_vtgmsg_t vtgmsg;
//		vtgmsg.talker = msg[1];					// 'P' for GPS, 'L' for GLONASS, 'N' for 'generic' method?
//		vtgmsg.posmode = argv[9][0];			// 'N' no Fix, 'A' Autonomous fix, 'D' Differential Fix
//		SysEvent(MODULE_ID_GPS, EVENT_INFO, EID_GPS_NMEA_MSG_VTG, &vtgmsg, sizeof(vtgmsg));
	} else if (strncmp(&msg[2], "RMC", 3)==0) {
		// xxRMC Message shows minimum recommended position data
		processed = true;
		if (argv[2][0] == 'A') {
			// valid fix
			uint32_t time = atoi(argv[1]);		// argv[1] is format 'hhmmss.sss' -> to int gives hhmmss as integer
			uint16_t ms  = atoi(&(argv[1][7])); // argv[1] is format 'hhmmss.sss' -> to int from position [1][7] gives ms as integer
			uint32_t date = atoi(argv[9]);		// argv[9] is format 'ddMMyy'

			juliandayfraction day = timConvertUtcTimeToJdf(time, ms);
			if (day > 0.0) {
				day += timConvertUtcDateToJdf(date);
				if (day > 1.0) {
					// Seems to be a valid date/time reception -> store in Sync Structure
					if (gpsSyncIdx == 0) {
						gpsSyncData.year = 2000 + (date % 100);
					}
					gpsSyncData.gpsUTC[gpsSyncIdx] = day;
					gpsSyncData.sysTime[gpsSyncIdx] = timGetSystime();
					if (gpsSyncData.year != 2000 + (date % 100)) {
						// The year changed while sync record collection -> restart new (either it was corrupted message or really silvester -> taker next sequenxe of 3)
						gpsSyncIdx = 0;
					} else {
						gpsSyncIdx++;
					}
					if (gpsSyncIdx >= GPS_TIMESYNC_COUNT) {
						gpsSyncIdx = 0;
						// We got 3 timestamps. Unfortunately NMEA checksum does not really prevent (double) flipping numbers. So we make
						// a plausibility check here.
						bool ok = true;
						for (int i = 0; i < GPS_TIMESYNC_COUNT - 1 ; i++) {
							double diff = gpsSyncData.gpsUTC[i+1] - gpsSyncData.gpsUTC[i];
							if ((diff > 0.0000116) || (diff < 0.0000115)) {	// We assume a record exactly every second here !!
								ok = false;
								break;
							}
						}
						if (ok) {
							// Synchronize the RTC (if needed) with the (first) recorded gps timestamp.
							timSyncUtc(gpsSyncData.year, gpsSyncData.sysTime[0],gpsSyncData.gpsUTC[0], TIM_SYNCSOURCE_GPS);
						}
					}
				}
			}
		}
//		gps_rmcmsg_t rmcmsg;
//		rmcmsg.talker = msg[1];					// 'P' for GPS, 'N' for 'other'
//		rmcmsg.posmode = argv[12][0];			// 'N' no Fix, 'A' Autonomous fix, 'D' Differential Fix
//		rmcmsg.valid = argv[2][0];				// 'V' invalid, 'A' valid !
//		rmcmsg.utcTimeSec = atoi(argv[1]);		// argv[1] is format 'hhmmss.sss' -> to int gives hhmmss as integer
//		rmcmsg.utcTimeMs = atoi(&(argv[1][7])); // argv[1] is format 'hhmmss.sss' -> to int from position [1][7] gives ms as integer
//		SysEvent(MODULE_ID_GPS, EVENT_INFO, EID_GPS_NMEA_MSG_RMC, &rmcmsg, sizeof(rmcmsg));
	} else {
		//SysEvent(MODULE_ID_GPS, EVENT_INFO, EID_GPS_NMEA_MSG_RAW, gpsRxBuffer, gpsRxIdx);
	}
	return processed;
}


void gpsSendByte(uint8_t b) {
	// block irq while handling tx buffer
	Chip_UART_IntDisable(gpsInitData->pUart, UART_IER_THREINT);

	if (gpsFirstByteAfterReset) {
		// first time after reset the THRE IRQ will not be triggered by just enabling it here
		// So we have to really send the byte here and do not put this into buffer. From next byte on
		// we always put bytes to the TX buffer and enabling the IRQ will trigger it when THR (transmit hold register)
		// gets empty (or also if it was and is still empty!)
		// see UM10360 Datasheet rev 4.1  page 315 first paragraph for details of this behavior!
		gpsFirstByteAfterReset = false;
		Chip_UART_SendByte(gpsInitData->pUart, b);
	} else {
		// add byte to buffer if there is room left
		// and wait for IRQ to fetch it
		if (gpsTxAddByte(b) == false) {
			// Buffer Full Error. Byte is skipped -> count errors or signal event ?????
			// .....
		}
	}

	// enable irq after handling tx buffer.
	Chip_UART_IntEnable(gpsInitData->pUart, UART_IER_THREINT);
}

void gpsSendBytes(uint8_t *data, uint8_t len) {
	for (int i=0;i<len;i++) {
		gpsSendByte(data[i]);
	}
}

//*** GPS NMEA RX state machine begin
typedef enum {
	GPS_RX_IDLE,
	GPS_RX_DATA,
	GPS_RX_CHCKSUM1,
	GPS_RX_CHCKSUM2,
	GPS_RX_CR,
	GPS_RX_LF
} gps_rx_state;

#define GPS_NMEA_MAXBYTES		128
#define GPS_NMEA_MAXFIELDS		25

static uint8_t gpsRxChecksum;
static uint8_t gpsRxBuffer[GPS_NMEA_MAXBYTES];
static char* gpsNmeaMessage[GPS_NMEA_MAXFIELDS];

static uint8_t gpsRxIdx = 0;
static uint8_t gpsFieldIdx = 0;
static uint8_t gpsFieldCnt = 0;
static gps_rx_state gpsRxStatus = GPS_RX_IDLE;


void gpsProcessRxByte(uint8_t rxByte) {

	switch (gpsRxStatus) {
	case GPS_RX_IDLE:
		if (rxByte == '$') {
			gpsRxIdx = 0;
			gpsFieldIdx = 0;
			gpsNmeaMessage[0] = (char*)gpsRxBuffer;
			gpsFieldCnt = 1;
			gpsRxChecksum = 0x00;
			gpsRxStatus = GPS_RX_DATA;
		}
		break;

	case GPS_RX_DATA:
		if (rxByte == '*') {
			gpsRxStatus = GPS_RX_CHCKSUM1;
		} else {
			gpsRxChecksum ^= rxByte;
			if (rxByte == ',') {
				//replace field separator with \0 to generate the argc,argv structure for message processing
				rxByte = 0x00;
				gpsNmeaMessage[gpsFieldCnt++] = (char*)(&gpsRxBuffer[gpsRxIdx+1]);
				if (gpsFieldCnt>=GPS_NMEA_MAXFIELDS) {
					SysEvent(MODULE_ID_GPS, EVENT_ERROR, EID_GPS_RXFIELDBUFFERFULL, NULL, 0);
					gpsRxStatus = GPS_RX_IDLE;
				}
			}
			gpsRxBuffer[gpsRxIdx++] = rxByte;
			if (gpsRxIdx>=GPS_NMEA_MAXBYTES) {
				SysEvent(MODULE_ID_GPS, EVENT_ERROR, EID_GPS_RXBYTEBUFFERFULL, NULL, 0);
				gpsRxStatus = GPS_RX_IDLE;
			}
		}
		break;

	case GPS_RX_CHCKSUM1: {
		char chckSum1 = (char)(gpsRxChecksum >> 4);
		if (chckSum1 <= 9) {
			chckSum1 += '0';
		} else {
			chckSum1 = chckSum1 + 'A' - (char)10;
		}
		if (chckSum1 == rxByte) {
			gpsRxStatus = GPS_RX_CHCKSUM2;
		} else {
			SysEvent(MODULE_ID_GPS, EVENT_ERROR, EID_GPS_CRCERROR, NULL, 0);
			gpsRxStatus = GPS_RX_IDLE;
		}
		break;
	}

	case GPS_RX_CHCKSUM2: {
		char chckSum2 = (char)(gpsRxChecksum & 0x0F);
		if (chckSum2 <= 9) {
			chckSum2 += '0';
		} else {
			chckSum2 = chckSum2 + 'A' - (char)10;
		}
		if (chckSum2 == rxByte) {
			gpsRxStatus = GPS_RX_CR;
		} else {
			SysEvent(MODULE_ID_GPS, EVENT_ERROR, EID_GPS_CRCERROR, NULL, 0);
			gpsRxStatus = GPS_RX_IDLE;
		}
		break;
	}

	case GPS_RX_CR:
		if (rxByte == 0x0d) {
			gpsRxStatus = GPS_RX_LF;
		} else {
			SysEvent(MODULE_ID_GPS, EVENT_ERROR, EID_GPS_MSGERROR, NULL, 0);
			gpsRxStatus = GPS_RX_IDLE;
		}
		break;

	case GPS_RX_LF:
		if (rxByte == 0x0a) {
			// ok the message is finally good here.
			if (!gpsProcessNmeaMessage(gpsFieldCnt, gpsNmeaMessage)) {
				SysEvent(MODULE_ID_GPS, EVENT_INFO, EID_GPS_NMEA_MSG_RAW, gpsRxBuffer, gpsRxIdx);
			}
			gpsRxStatus = GPS_RX_IDLE;
		} else {
			SysEvent(MODULE_ID_GPS, EVENT_ERROR, EID_GPS_MSGERROR, NULL, 0);
			gpsRxStatus = GPS_RX_IDLE;
		}
		break;

	default:
		gpsRxStatus = GPS_RX_IDLE;
		break;
	}
}

//*** GPS NMEA RX state machine end

