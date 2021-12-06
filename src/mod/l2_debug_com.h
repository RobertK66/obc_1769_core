/*
===============================================================================
 Name        : l2_debug_com.h
 Author      : Robert
 Created on	 : 07.09.2021
===============================================================================
*/

#ifndef MOD_L2_DEBUG_COM_H_
#define MOD_L2_DEBUG_COM_H_

#include <chip.h>
#include <string.h>
#include "../temp-Base.h"

// RX definitions
#define DEB_L2_MAX_CHARPERLINE		100
#define DEB_L2_MAX_PARAMS			 10
typedef struct {
	char	cmdLine[DEB_L2_MAX_CHARPERLINE];
	int 	parCnt;
	char* 	pars[DEB_L2_MAX_PARAMS];
} DEB_L2_CMD_T;

// TX definitions
//#define DEB_L2_MAX_TX_FRAMESIZE		 32


// Module Header
void deb_init (LPC_USART_T *pUart);
void deb_main (void);

static const MODULE_DEF_T debModuleDesc = {
		//0,
		(void*)deb_init,
		deb_main
};

#define DebInitModule(pUart) {	\
	debModuleDesc.init(pUart); 	\
}

#define DebMain() { 		\
	debModuleDesc.main(); 	\
}


// Module API
bool	deb_getCommandIfAvailable(DEB_L2_CMD_T *pCmd);
bool    deb_sendFrame(uint8_t *data, uint8_t len);

// API defines
#define deb_sendString(s) {deb_sendFrame((uint8_t*)(s), strlen(s));}

#endif /* MOD_L2_DEBUG_COM_H_ */
