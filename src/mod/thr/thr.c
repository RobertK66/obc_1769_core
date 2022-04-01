/*
 * based on climb_gps.c
 *
 *  Created on: 27.02.2022
 *      Copy paste by: Jevgeni
 */
#include "thr.h"
#include <ado_uart.h>
#include "../l7_climb_app.h"
#include "../l2_debug_com.h"
// prototypes
void thrUartIRQ(LPC_USART_T *pUART);
void thrProcessRxByte(uint8_t rxByte);

// local/module variables
static thr_initdata_t *thrInitData;

// ************************** TX Circular byte Buffer Begin ********************
#define THR_TX_BUFFERSIZE	200
static uint8_t 		   thrTxBuffer[THR_TX_BUFFERSIZE];
static uint8_t         thrTxWriteIdx = 0;
static uint8_t         thrTxReadIdx  = 0;
static bool		  	   thrTxBufferFull = false;
static bool				thrFirstByteAfterReset=true;

static bool inline thrTxBufferEmpty() {
	if (thrTxBufferFull) {
		return false;
	} else {
		return (thrTxReadIdx == thrTxWriteIdx);
	}
}

static bool inline thrTxAddByte(uint8_t b) {
	if (!thrTxBufferFull) {
		thrTxBuffer[thrTxWriteIdx++] = b;
		if (thrTxWriteIdx >= THR_TX_BUFFERSIZE) {
			thrTxWriteIdx = 0;
		}
		if (thrTxWriteIdx == thrTxReadIdx) {
			thrTxBufferFull = true;
		}
		return true;
	} else {
		return false;
	}
}

static uint8_t inline thrTxGetByte(void) {
	uint8_t retVal = thrTxBuffer[thrTxReadIdx++];
	if (thrTxReadIdx >= THR_TX_BUFFERSIZE) {
		thrTxReadIdx = 0;
	}
	thrTxBufferFull = false;
	return retVal;
}

// ************************** TX Circular byte Buffer End ********************


void thrInit (void *initData) {
	thrInitData = (thr_initdata_t*) initData;

	// Switch the 'enable' pin to low (no internal Volage regulator needed
	Chip_GPIO_SetPinOutLow(LPC_GPIO, thrInitData->pEnablePin->pingrp, thrInitData->pEnablePin->pinnum);

	Chip_GPIO_SetPinOutLow(LPC_GPIO, 2, 5); //  This is PINIDX_RS485_TX_RX

	// Init UART
	InitUart(thrInitData->pUart, 9600, thrUartIRQ);
	thrFirstByteAfterReset = true;
}

void thrMain (void) {
	// Uart Rx
	int32_t stat = Chip_UART_ReadLineStatus(thrInitData->pUart);
	if (stat & UART_LSR_RDR) {
		// there is a byte available. Lets read and process it.
		uint8_t b = Chip_UART_ReadByte(thrInitData->pUart);
		thrProcessRxByte(b);
	}
}

void thrUartIRQ(LPC_USART_T *pUART) {
	if (thrInitData->pUart->IER & UART_IER_THREINT) {
		// Transmit register is empty now (byte was sent out)
		if (thrTxBufferEmpty() == false) {
			// Send next byte
			uint8_t nextByte = thrTxGetByte();
			Chip_UART_SendByte(thrInitData->pUart, nextByte);
		} else {
			// No more bytes available -> stop the THRE IRQ.
			Chip_UART_IntDisable(thrInitData->pUart, UART_IER_THREINT);
		}
	}
}

void thrSendByte(uint8_t b) {

	Chip_GPIO_SetPinOutLow(LPC_GPIO, 1, 19); //  This is PINIDX_RS485_TX_RX  SET LOW MEANS SWTCH TO TX
	// block irq while handling tx buffer
	Chip_UART_IntDisable(thrInitData->pUart, UART_IER_THREINT);



	//if (thrTxBufferEmpty()) {
		// First Byte: Store in Buffer and initiate TX
		//thrTxAddByte(b);
		// Put this byte on the UART Line.
	if(thrTxBufferEmpty()){
				// first time after reset the THRE IRQ will not be triggered by just enabling it here
				// So we have to really send the byte here and do not put this into buffer. From next byte on
				// we always put bytes to the TX buffer and enabling the IRQ will trigger it when THR (transmit hold register)
				// gets empty (or also if it was and is still empty!)
				// see UM10360 Datasheet rev 4.1  page 315 first paragraph for details of this behavior!
				thrFirstByteAfterReset = false;
				Chip_UART_SendByte(thrInitData->pUart, b);
	} else {
		// add byte to buffer if there is room left
			// and wait for IRQ to fetch it
		if (thrTxAddByte(b) == false) {
			// Buffer Full Error. Byte is skipped -> count errors or signal event ?????
			// .....
		}
	}

	// enable irq after handling tx buffer.
	Chip_UART_IntEnable(thrInitData->pUart, UART_IER_THREINT);

	Chip_GPIO_SetPinOutHigh(LPC_GPIO, 1, 19); //  This is PINIDX_RS485_TX_RXP  SET HIGH - back to receive
}

void thrSendBytes(uint8_t *data, uint8_t len) {
	for (int i=0;i<len;i++) {
		thrSendByte(data[i]);
	}

}


void thrProcessRxByte(uint8_t rxByte) {
	// do your processing of RX here....

}
