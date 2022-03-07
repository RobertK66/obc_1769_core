/*
 * climb_gps.c
 *
 *  Created on: 27.02.2022
 *      Author: Robert
 */
#include "climb_gps.h"

#include <string.h>
#include <ado_uart.h>
#include <ado_modules.h>

// prototypes
void gpsUartIRQ(LPC_USART_T *pUART);
void gpsProcessRxByte(uint8_t rxByte);
void gpsProcessNmeaMessage(int argc, char *argv[]);

// local/module variables
static gps_initdata_t *gpsInitData;

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



void gpsProcessNmeaMessage(int argc, char *argv[]) {

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
static char*  gpsNmeaMessage[GPS_NMEA_MAXFIELDS];
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
				gpsNmeaMessage[gpsFieldCnt++] = (char*)(&gpsRxBuffer[gpsRxIdx]);
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
			SysEvent(MODULE_ID_GPS, EVENT_INFO, EID_GPS_NMEA_MSG_RAW, gpsNmeaMessage[0], gpsRxIdx);
			gpsProcessNmeaMessage(gpsFieldCnt, gpsNmeaMessage);
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
