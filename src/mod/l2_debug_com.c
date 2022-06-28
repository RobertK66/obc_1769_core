/*
===============================================================================
 Name        : l2_debug_com.c
 Author      : Robert
 Created on	 : 07.09.2021
===============================================================================
 *** Layer2 module for Debug Interface via external Serial Communication ***

Rx part waits for ASCII char terminated by a CR or LF char. The received Line
is then translated into argc argv and is available for higher layers module
as "Received Command".

Tx can buffer up to TX_L2_FRAME_SIZE Bytes. A TX Frame can send any binary
data. Frame starts and ends with 0x7E. To Escape a flag in date we use 0x7D.
The escaped data is Xored by 0x20.
	Example:   01 02 7e 03 -> 01 02 7d 5e 03

===============================================================================
*/
#include "l2_debug_com.h"
#include "l7_climb_app.h"

#include <string.h>
#include <ado_uart.h>

#define DEB_L2_TX_FRAMESTARTSTOP	0x7E
#define DEB_L2_TX_FRAMEESCAPE		0x7D
#define DEB_L2_TX_BUFFER_SIZE		 600

#define DEB_EOL_CHAR				0x0a

typedef struct {
	uint8_t  txData[DEB_L2_TX_BUFFER_SIZE];
	uint16_t currentTxByteIdx;
	uint16_t headTxByteIdx;
} DEB_L2_TXFRAME_QUEUE_T ;

typedef enum  {
	DEB_IDLE,
	DEB_TX
} DEB_TX_STATE_T ;

// Module Variables
LPC_USART_T *	deb_Uart;
char	 		deb_InputLine[DEB_L2_MAX_CHARPERLINE] 	= "";
char	 		deb_CommandLine[DEB_L2_MAX_CHARPERLINE] = "";
uint8_t			deb_RxIdx = 0;
bool			deb_CommandAvailable = false;

DEB_TX_STATE_T	 		deb_txState = DEB_IDLE;
DEB_L2_TXFRAME_QUEUE_T  deb_txFrames;

// Prototypes
void deb_uartIRQ(LPC_USART_T *pUAR);
void deb_processLine(void);

inline void deb_init(void *pUart) {
	_deb_init ((LPC_USART_T *)pUart);
}

void _deb_init (LPC_USART_T *pUart) {
	//LPC_USART_T *pUart
	InitUart(pUart, 9600, deb_uartIRQ);
	deb_Uart = pUart;

	deb_txFrames.currentTxByteIdx = 0;
	deb_txFrames.headTxByteIdx = 0;
}

void deb_main (void) {

	char* debug_print;
	uint8_t request[3];
	request[0]= 0xFF;
	request[1]= 0x33;
	request[2]= 0xFF;
	debug_print = (char*)request;

	int32_t stat = Chip_UART_ReadLineStatus(deb_Uart);
	if (stat & UART_LSR_RDR) {
		int ch = (int)Chip_UART_ReadByte(deb_Uart);
		if (ch != 0x0a &&
			ch != 0x0d) {
			deb_InputLine[deb_RxIdx++] = (char)(ch);
			//SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, (char)(ch), strlen((char)(ch))); ///print if something received
		}

		if ((deb_RxIdx >= DEB_L2_MAX_CHARPERLINE - 1) ||
			 ch == DEB_EOL_CHAR ) 	{
			//SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, request, strlen(request)); ///print if something received
			deb_InputLine[deb_RxIdx] = 0x00;
			deb_processLine();
			deb_RxIdx = 0;
		}
	}
}

void deb_processLine(void) {
	if (deb_CommandAvailable) {
		// Polling error. Nobody got the previous Command out of buffer yet!
		// Signal 'Command Overrun'-Event
		// TODO:

		// ?? throw away new command or overwrite old one !?
		//SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, deb_InputLine, strlen(deb_InputLine)); ///print if something received
	} else {
		// copy the received line to our command line buffer in order to get room for next RX bytes.
		strcpy(deb_CommandLine, deb_InputLine);
		deb_CommandAvailable = true;

		//investigation of why DEBUG UART works unreliable
		// print received line
		//SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, deb_InputLine, strlen(deb_InputLine));
		//char* debug_print = "debug print";
		//SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, debug_print, strlen(debug_print));
		//SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, deb_InputLine, strlen(deb_InputLine)); ///print if something received
	}
}


bool deb_getCommandIfAvailable(DEB_L2_CMD_T *pRetVal){
	bool cmdAvailable = false;
	if (deb_CommandAvailable) {
		// copy the CommandLine into struct. This is for data to stay available with polling Caller as long as he wants to use it !!!!
		strcpy(pRetVal->cmdLine, deb_CommandLine);
		size_t len = strlen(pRetVal->cmdLine);
		memset(&pRetVal->cmdLine[len+1], 0x20, DEB_L2_MAX_CHARPERLINE - len - 1);
		deb_CommandAvailable = false;

		// Lets split the text line in separate strings for parameters
		pRetVal->parCnt = 1;
		pRetVal->pars[0] = pRetVal->cmdLine;
		for (int i = 1; i < DEB_L2_MAX_PARAMS; i++) {
			pRetVal->pars[i] = NULL;
		}

		for (int i = 0, p=1; pRetVal->cmdLine[i] != 0x00; i++) {
			if ( (pRetVal->cmdLine[i] == ' ') &&
				 (p < DEB_L2_MAX_PARAMS) )	{
				pRetVal->cmdLine[i] = 0x00;
				pRetVal->pars[p++] = (&pRetVal->cmdLine[i]) + 1;
				pRetVal->parCnt++;
			}
		}
		cmdAvailable = true;
	}
	return cmdAvailable;
}

#define IncHeadTxIdx() { \
	deb_txFrames.headTxByteIdx++; \
	if (deb_txFrames.headTxByteIdx >= DEB_L2_TX_BUFFER_SIZE) { \
		deb_txFrames.headTxByteIdx = 0; \
	} }

#define IncCurrentTxIdx() { \
	deb_txFrames.currentTxByteIdx++; \
	if (deb_txFrames.currentTxByteIdx >= DEB_L2_TX_BUFFER_SIZE) { \
		deb_txFrames.currentTxByteIdx = 0; \
	} }




void deb_CopyAndEscapeData(uint8_t *data, uint16_t len) {
	int i = 0;
	bool inEscape = false;
	while (deb_txFrames.headTxByteIdx != deb_txFrames.currentTxByteIdx) {
		if (! inEscape) {
			if (    (data[i] == DEB_L2_TX_FRAMESTARTSTOP)
				 || (data[i] == DEB_L2_TX_FRAMEESCAPE) )	{
				inEscape = true;
				deb_txFrames.txData[deb_txFrames.headTxByteIdx] = DEB_L2_TX_FRAMEESCAPE;
			} else {
				deb_txFrames.txData[deb_txFrames.headTxByteIdx] = data[i];
				i++;
			}
		} else {
			inEscape = false;
			deb_txFrames.txData[deb_txFrames.headTxByteIdx] = data[i] ^ 0x20;
			i++;
		}
		IncHeadTxIdx();
		if (i>=len) {
			break;
		}
	}
}



bool deb_sendEventFrame(event_id_t eventId, uint8_t *data, uint16_t len) {
	bool ok;
	// block irq while handling buffers
	Chip_UART_IntDisable(deb_Uart, UART_IER_THREINT);
	uint16_t oldHead = deb_txFrames.headTxByteIdx;

	deb_txFrames.txData[deb_txFrames.headTxByteIdx] = DEB_L2_TX_FRAMESTARTSTOP;
	IncHeadTxIdx();

	deb_CopyAndEscapeData((uint8_t *)&eventId, sizeof(event_id_t));
	deb_CopyAndEscapeData(data, len);

	// Check if end Token fits into buffer
	ok = (deb_txFrames.headTxByteIdx != deb_txFrames.currentTxByteIdx);
	if (ok) {
		deb_txFrames.txData[deb_txFrames.headTxByteIdx] = DEB_L2_TX_FRAMESTARTSTOP;
		IncHeadTxIdx();		// Could be filled up now. But if so thats ok here!
	}

	if (ok) {
		if (deb_txState == DEB_IDLE) {
			// Start TX and let IRQ do the rest
			deb_txState = DEB_TX;
			Chip_UART_SendByte(deb_Uart, deb_txFrames.txData[deb_txFrames.currentTxByteIdx]);
		}
	} else {
		// Reset buffer to previous state - frame discarded!
		deb_txFrames.headTxByteIdx = oldHead;
		// TODO: Signal frame buffer error
	}

	// (Re-)Enable IRQ if needed
	if (deb_txState != DEB_IDLE) {
		Chip_UART_IntEnable(deb_Uart, UART_IER_THREINT);
	}
	return ok;

}



//bool deb_sendFrame(uint8_t *data, uint16_t len) {
//	bool ok;
//
//	// block irq while handling buffers
//	Chip_UART_IntDisable(deb_Uart, UART_IER_THREINT);
//
//	uint16_t oldHead = deb_txFrames.headTxByteIdx;
//
//	// Copy the next frame on end of TXBuffer.
//	deb_txFrames.txData[deb_txFrames.headTxByteIdx] = DEB_L2_TX_FRAMESTARTSTOP;
//	IncHeadTxIdx();
//
//	deb_CopyAndEscapeData(data, len);
//
////	//ok = IncDataIdx(&deb_txFrames.headTxByteIdx);
////	int i = 0;
////	bool inEscape = false;
////	while (deb_txFrames.headTxByteIdx != deb_txFrames.currentTxByteIdx) {
////		if (! inEscape) {
////			if (    (data[i] == DEB_L2_TX_FRAMESTARTSTOP)
////				 || (data[i] == DEB_L2_TX_FRAMEESCAPE) )	{
////				inEscape = true;
////				deb_txFrames.txData[deb_txFrames.headTxByteIdx] = DEB_L2_TX_FRAMEESCAPE;
////			} else {
////				deb_txFrames.txData[deb_txFrames.headTxByteIdx] = data[i];
////				i++;
////			}
////		} else {
////			inEscape = false;
////			deb_txFrames.txData[deb_txFrames.headTxByteIdx] = data[i] ^ 0x20;
////			i++;
////		}
////		IncHeadTxIdx();
////		if (i>=len) {
////			break;
////		}
////	}
//
//	// Check if end Token fits into buffer
//	ok = (deb_txFrames.headTxByteIdx != deb_txFrames.currentTxByteIdx);
//	if (ok) {
//		deb_txFrames.txData[deb_txFrames.headTxByteIdx] = DEB_L2_TX_FRAMESTARTSTOP;
//		IncHeadTxIdx();		// Could be filled up now. But if so thats ok here!
//	}
//
//	if (ok) {
//		if (deb_txState == DEB_IDLE) {
//			// Start TX and let IRQ do the rest
//			deb_txState = DEB_TX;
//			Chip_UART_SendByte(deb_Uart, deb_txFrames.txData[deb_txFrames.currentTxByteIdx]);
//		}
//	} else {
//		// Reset buffer to previous state - frame discarded!
//		deb_txFrames.headTxByteIdx = oldHead;
//		// TODO: Signal frame buffer error
//	}
//
//	// (Re-)Enable IRQ if needed
//	if (deb_txState != DEB_IDLE) {
//		Chip_UART_IntEnable(deb_Uart, UART_IER_THREINT);
//	}
//	return ok;
//}
//
//


void deb_uartIRQ(LPC_USART_T *pUART) {
	if (pUART->IER & UART_IER_THREINT) {
		int nextChar;
		switch (deb_txState) {
			case DEB_TX:
				// Send next Byte
				IncCurrentTxIdx();
				// Last one reached ?
				if (deb_txFrames.currentTxByteIdx == deb_txFrames.headTxByteIdx) {
					// Yes buffer empty: stop TX IRQ
					deb_txState = DEB_IDLE;
					Chip_UART_IntDisable(deb_Uart, UART_IER_THREINT);
				} else {
					nextChar = deb_txFrames.txData[deb_txFrames.currentTxByteIdx];
					Chip_UART_SendByte(pUART, nextChar);
				}
				break;

			case DEB_IDLE:
				// If not already done disable IRQs here.
				Chip_UART_IntDisable(deb_Uart, UART_IER_THREINT);
				break;
		}
	}
}
