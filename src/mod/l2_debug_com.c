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

#include <string.h>
#include <ado_uart.h>

#define DEB_L2_TX_FRAMESTARTSTOP	0x7E
#define DEB_L2_TX_FRAMEESCAPE		0x7D
#define DEB_L2_TX_BUFFER_SIZE		 500
#define DEB_L2_MAX_TXFRAMES		      20

#define DEB_EOL_CHAR				0x0d

typedef struct {
	uint8_t txData[DEB_L2_TX_BUFFER_SIZE];
	uint8_t frameBaseIdx[DEB_L2_MAX_TXFRAMES];
	uint8_t frameLen[DEB_L2_MAX_TXFRAMES];
	uint8_t currentFrameIdx;
	uint8_t currentByteIdx;
	uint8_t pendingFrames;
	uint8_t pendingBytes;
} DEB_L2_TXFRAME_QUEUE_T ;

typedef enum  {
	IDLE,
	STARTFRAME,
	STOPFRAME,
	ESCAPECHAR
} DEB_TX_STATE_T ;


// Module Variables
LPC_USART_T *	deb_Uart;
char	 		deb_InputLine[DEB_L2_MAX_CHARPERLINE] 	= "";
char	 		deb_CommandLine[DEB_L2_MAX_CHARPERLINE] = "";
uint8_t			deb_RxIdx = 0;
bool			deb_CommandAvailable = false;

DEB_L2_TXFRAME_QUEUE_T	deb_txFrames;
DEB_TX_STATE_T	 		deb_txState = IDLE;

// Prototypes
void deb_uartIRQ(LPC_USART_T *pUAR);
void deb_processLine(void);

void deb_init (LPC_USART_T *pUart) {
	InitUart(pUart, 9600, deb_uartIRQ);
	deb_Uart = pUart;
	deb_txFrames.pendingFrames = 0;
	deb_txFrames.currentFrameIdx = 0;
	deb_txFrames.pendingBytes = 0;
	deb_txFrames.currentByteIdx = 0;
}

void deb_main (void) {
	int32_t stat = Chip_UART_ReadLineStatus(deb_Uart);
	if (stat & UART_LSR_RDR) {
		int ch = (int)Chip_UART_ReadByte(deb_Uart);
		if (ch != 0x0a &&
			ch != 0x0d) {
			deb_InputLine[deb_RxIdx++] = (char)(ch);
		}

		if ((deb_RxIdx >= DEB_L2_MAX_CHARPERLINE - 1) ||
			 ch == DEB_EOL_CHAR ) 	{
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
	} else {
		// copy the received line to our command line buffer in order to get room for next RX bytes.
		strcpy(deb_CommandLine, deb_InputLine);
		deb_CommandAvailable = true;
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

bool deb_sendFrame(uint8_t *data, uint8_t len) {
	bool ok = true;
	uint8_t newByteBaseIdx = 0;
	uint8_t frameIdx = 0;

	// TODO: stop IRQ here !!???
	Chip_UART_IntDisable(deb_Uart, UART_IER_THREINT);

	// Check for available Frame Entry
	if (deb_txFrames.pendingFrames >= DEB_L2_MAX_TXFRAMES) {
		//Buffer full -> error signal and/or return value false
		ok = false;
	} else {
		// Framepointer left, check for Bytes available
		frameIdx = deb_txFrames.currentFrameIdx + deb_txFrames.pendingFrames;
		newByteBaseIdx = deb_txFrames.frameBaseIdx[frameIdx] + deb_txFrames.frameLen[frameIdx];
		frameIdx++;
		if (frameIdx > DEB_L2_MAX_TXFRAMES) {
			frameIdx = 0;
		}
		if (newByteBaseIdx + len > DEB_L2_TX_BUFFER_SIZE) {
			// Wrap around. For now just Skip this left over memory....
			newByteBaseIdx = 0;
			// Check for available Bytes up to current frame
			if (len > deb_txFrames.frameBaseIdx[deb_txFrames.currentFrameIdx]) {
				//Buffer full -> error signal and/or return value false
				ok = false;
			}
		}
	}

	if (ok) {
		deb_txFrames.frameBaseIdx[frameIdx] = newByteBaseIdx;
		deb_txFrames.frameLen[frameIdx] = len;
		deb_txFrames.pendingFrames++;
		if (deb_txFrames.pendingFrames == 1) {
			// Start TX and let IRQ do the rest
			deb_txFrames.currentFrameIdx = frameIdx;
			deb_txState = STARTFRAME;
			Chip_UART_IntEnable(deb_Uart, UART_IER_THREINT);
			Chip_UART_SendByte(deb_Uart, DEB_L2_TX_FRAMESTARTSTOP);
		}
	}

	if (deb_txState != IDLE) {
		Chip_UART_IntEnable(deb_Uart, UART_IER_THREINT);
	}
	return ok;
}


void deb_uartIRQ(LPC_USART_T *pUART) {
	if (pUART->IER & UART_IER_THREINT) {
		char nextChar;
		switch (deb_txState) {
			case STARTFRAME:
				// Send first Byte
				Chip_UART_SendByte(pUART, nextChar);
				deb_txState = IDLE;
				break;
			case IDLE:
				Chip_UART_IntDisable(deb_Uart, UART_IER_THREINT);
				break;
		}
	}
}
