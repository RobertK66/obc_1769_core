/*
 * based on climb_gps.c
 *
 *  Created on: 27.02.2022
 *      Copy paste by: Jevgeni
 */
#include "thr.h"
#include <ado_uart.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "../l7_climb_app.h"
#include "../l2_debug_com.h"
#include "../l4_thruster.h" // with that we include variable  l4_thr_ExpectedReceiveBuffer  which defines expected RX buffer length
#include "../modules_globals.h"

//uint16_t LAST_STARTED_MODULE;


// prototypes
void thrUartIRQ(LPC_USART_T *pUART);
void thrProcessRxByte(uint8_t rxByte);
void thr_debugPrintBuffer(uint8_t *buffer,int bufferlen);

static thr_initdata_t *thrInitData;

// ************************** TX Circular byte Buffer Begin ********************
#define THR_TX_BUFFERSIZE	200
static uint8_t 		   thrTxBuffer[THR_TX_BUFFERSIZE];
static uint8_t         thrTxWriteIdx = 0;
static uint8_t         thrTxReadIdx  = 0;
static bool		  	   thrTxBufferFull = false;
static bool				thrFirstByteAfterReset=true;


#define THR_BUFFERLEN 700 // maximum buffer length
//int thr_counter = 0; would be defined in l4_thruster
char thr_receiveBuffer[THR_BUFFERLEN] ="";
//l4_thr_counter = 0; // set received bytes counter to 0 initialy // defined in l4_thruster.c


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
	//Chip_GPIO_SetPinOutLow(LPC_GPIO, thrInitData->pEnablePin->pingrp, thrInitData->pEnablePin->pinnum);
	//// IMPORTANT PINIDX_RS485_TX_RX   HIGH - TRANSMIT              LOW - RECEIVE
	// SET TO RECEIVE UPON INITIALIZATION
	//Chip_GPIO_SetPinOutLow(LPC_GPIO, 2, 5); //  This is PINIDX_RS485_TX_RX RECEIVE
	//Chip_GPIO_SetPinOutHigh(LPC_GPIO, 2, 5); //  This is PINIDX_RS485_TX_RX TRANSMIT

	// Init UART
	InitUart(thrInitData->pUart, 115200, thrUartIRQ);
	thrFirstByteAfterReset = true;

	// Enable dirrection controll
	thrInitData->pUart->RS485CTRL |= UART_RS485CTRL_DCTRL_EN; // Enable Auto Direction Control

	// If direction control is enabled (bit DCTRL = 1), pin DTR is used for direction control
	thrInitData->pUart->RS485CTRL |= UART_RS485CTRL_SEL_DTR;

	//This bit reverses the polarity of the direction control signal on the RTS (or DTR) pin. The direction control pin
	 //will be driven to logic "1" when the transmitter has data to be sent
	thrInitData->pUart->RS485CTRL |= UART_RS485CTRL_OINV_1;



}

void thrMain (void) {
	LAST_STARTED_MODULE=10;
	// Uart Rx
	int32_t stat = Chip_UART_ReadLineStatus(thrInitData->pUart);
	if (stat & UART_LSR_RDR) {
		// there is a byte available. Lets read and process it.


		uint8_t b = Chip_UART_ReadByte(thrInitData->pUart);
		thrProcessRxByte(b);
	}
}

void thrUartIRQ(LPC_USART_T *pUART) {
	LAST_STARTED_MODULE=1001;
	if (thrInitData->pUart->IER & UART_IER_THREINT) {
		// Transmit register is empty now (byte was sent out)
		if (thrTxBufferEmpty() == false) {
			// Send next byte
			uint8_t nextByte = thrTxGetByte();
			Chip_UART_SendByte(thrInitData->pUart, nextByte);
		} else {
			// No more bytes available -> stop the THRE IRQ.
			Chip_UART_IntDisable(thrInitData->pUart, UART_IER_THREINT);

			// switch back to receive when all bytes are transmited
			Chip_GPIO_SetPinOutLow(LPC_GPIO, 2, 5); //  This is PINIDX_RS485_TX_RX RECEIVE
		}
	}
}

void thrSendByte(uint8_t b) {
	LAST_STARTED_MODULE=1002;

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

	// enable irq after handling tx buffer
	Chip_UART_IntEnable(thrInitData->pUart, UART_IER_THREINT);

}

void thrSendBytes(uint8_t *data, uint8_t len) {
	LAST_STARTED_MODULE=1003;
	//set to transmit when sending data package
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, 2, 5); //  This is PINIDX_RS485_TX_RX TRANSMIT
	for (int i=0;i<len;i++) {
		thrSendByte(data[i]);
	}

}


void thrProcessRxByte(uint8_t rxByte) {
	LAST_STARTED_MODULE=1004;
	// do your processing of RX here....

	if (l4_thr_counter< l4_thr_ExpectedReceiveBuffer){ // change it to expected buffer length SET by REQUEST functions

		thr_receiveBuffer[l4_thr_counter]=(char) rxByte;
		//Chip_UART_SendByte(LPC_UART2, rxByte); // print received byte
		l4_thr_counter++;
	}
	if (l4_thr_counter== l4_thr_ExpectedReceiveBuffer) {

		thr_receiveBuffer[l4_thr_counter]=(char) rxByte;
		//Chip_UART_SendByte(LPC_UART2, rxByte); // print received byte

		l4_thr_counter =0;
		ParseReadRequest((uint8_t*)&thr_receiveBuffer,l4_thr_ExpectedReceiveBuffer);

		char print_str[200];
		sprintf(print_str, "\n Thruster Reply : \n");
		int len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		deb_print_pure_debug((uint8_t*)&thr_receiveBuffer,l4_thr_ExpectedReceiveBuffer);

		sprintf(print_str, "\n");
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);


		// TODO : TEST ParseReadRequest. Make request to read all registers.
		// Parse reply with ParseReadRequest and verify that array of data values are stored correctly

	}

}




