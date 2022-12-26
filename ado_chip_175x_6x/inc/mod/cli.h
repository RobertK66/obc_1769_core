/*
 * cli.h
 *
 *  Created on: 02.11.2019
 *      Author: Robert
 */

#ifndef MOD_CLI_CLI_H_
#define MOD_CLI_CLI_H_

#include <chip.h>

// Defaults used by CliInit() macro;
// Do not change the default values here. Use the 'overloaded' CliInitN() macros
// or make global definitions of the corresponding CLI_CFG_xyz values.
#define CLI_DEFAULT_BAUD			115200
#define CLI_DEFAULT_UART			LPC_UART0

#if CLI_CFG_TXBUFFER_SIZE
	#define CLI_TXBUFFER_SIZE	((uint16_t)CLI_CFG_TXBUFFER_SIZE)
#else
	#define CLI_TXBUFFER_SIZE	((uint16_t)1024)
#endif

#if CLI_CFG_ENDOFLINE_CHAR
	#define CLI_ENDOFLINE_CHAR CLI_CFG_ENDOFLINE_CHAR
#else
	#define CLI_ENDOFLINE_CHAR ((int)'\n')
#endif

#if CLI_CFG_SWO_MAX_BUFFERWAITPERMAINLOOP
	#define CLI_SWO_MAX_BUFFERWAITPERMAINLOOP CLI_CFG_SWO_MAX_BUFFERWAITPERMAINLOOP
#else
	#define CLI_SWO_MAX_BUFFERWAITPERMAINLOOP 16
#endif

#define CliInitSWO() 									_CliInit(0, 0 , 0, CLI_TXBUFFER_SIZE)
#define CliInitSWO1(pTxBuffer, txBufferSize) 			_CliInit(0, 0 , pTxBuffer, txBufferSize)

#define CliInit() 										_CliInit(CLI_DEFAULT_UART, CLI_DEFAULT_BAUD , 0,  CLI_TXBUFFER_SIZE )
#define CliInit1(pUart) 								_CliInit(pUart, CLI_DEFAULT_BAUD, 0, CLI_TXBUFFER_SIZE )
#define CliInit2(pUart,baud) 							_CliInit(pUart, baud,  0, CLI_TXBUFFER_SIZE)
#define CliInit4(pUart,baud, pTxBuffer, txBufferSize) 	_CliInit(pUart, baud,  pTxBuffer, txBufferSize)

// Module API Prototypes
void _CliInit(LPC_USART_T *pUart, int baud, char *pTxBuffer, uint16_t txBufferSize);	// module init to be called once prior mainloop. (Use macros for calling with various par lists.)
void CliMain(void);								// module routine should participate in regular mainloop calls.

void CliRegisterCommand(char* cmdStr, void (*callback)(int argc, char *argv[]));	// Assign Callback for your custom commands.

#endif /* MOD_CLI_CLI_H_ */
