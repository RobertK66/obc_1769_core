/*
===============================================================================
 Name        : l7_climb_app.c
 Author      : Robert
 Created on	 : 07.09.2021
===============================================================================
*/
#include "l7_climb_app.h"

#include <string.h>
#include <stdlib.h>
#include "l2_debug_com.h"
#include "mem/ado_mram.h"


// Prototypes
void app_processCmd(int argc, char *argv[]);
void ReadMramFinished (mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
void WriteMramFinished (mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);

void app_init (void *dummy) {

}

void app_main (void) {
	// Debug Command Polling (direct from L2 CLI Module)
	DEB_L2_CMD_T cmd;
	if ( deb_getCommandIfAvailable(&cmd) ) {
		app_processCmd(cmd.parCnt, cmd.pars);
	}
	// handle event - queue ....

}

uint8_t  tempData[MRAM_MAX_WRITE_SIZE];

void app_processCmd(int argc, char *argv[]) {
	char* cmd = argv[0];

	switch (cmd[0]) {
	case '0':
		deb_sendFrame((uint8_t*)"Hi from cmd 0",14);
		break;
	case '1':
		if (argc != 4) {
			deb_sendFrame((uint8_t*)"uasge: 1 <chipIdx> <adr> <len>", 25);
		} else {
			// CLI params to binary params
			uint8_t  idx = atoi(argv[1]);
		    uint32_t adr = atoi(argv[2]);
		    uint32_t len = atoi(argv[3]);
		    if (len > MRAM_MAX_READ_SIZE) {
		        len = MRAM_MAX_READ_SIZE;
		    }
		    if (idx >= MRAM_CHIP_CNT) {
		       idx = 0;
		    }
		    ReadMramAsync(idx, adr, tempData, len, ReadMramFinished);
		 }
		break;
	case '2':
		  if (argc != 5) {
			  deb_sendFrame((uint8_t*)"uasge: cmd <chipidx> <adr> <databyte> <len>",30 );
		  } else {
		    // CLI params to binary params
		    uint8_t  idx = atoi(argv[1]);
		    uint32_t adr = atoi(argv[2]);
		    uint8_t byte = atoi(argv[3]);
		    uint32_t len = atoi(argv[4]);
		    if (len > MRAM_MAX_WRITE_SIZE) {
		        len = MRAM_MAX_WRITE_SIZE;
		    }
		    if (idx > MRAM_CHIP_CNT) {
		        idx = 0;
		    }

		    for (int i=0;i<len;i++){
		    	tempData[i] = byte;
		    }

		    // Binary Command
		    WriteMramAsync(idx, adr, tempData, len,  WriteMramFinished);
		}
		break;
	}
	// Dummy processing. Send pars in reverse order to L2 Debug (one frame per par)
//	for (int i = argc - 1; i>=0; i--) {
//		if (!deb_sendFrame((uint8_t *)argv[i], strlen(argv[i]))) {
//			// Error !? buffer full or something else
//			// Signal event here !? Or rely on event from lower layer itself !?
//			break;
//		}
//	}
}

void ReadMramFinished (mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
    if (result == MRAM_RES_SUCCESS) {
    	deb_sendFrame((uint8_t*)data, len);
    } else {
    	deb_sendFrame((uint8_t*)"ERROR  !!!", 10);
    }
}

void WriteMramFinished (mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	if (result == MRAM_RES_SUCCESS) {
		deb_sendFrame((uint8_t*)"SUCCESS", 10);
	} else {
    	deb_sendFrame((uint8_t*)"Write ERROR!!!", 12);
	}
}


void _SysEvent(uint8_t *data, uint16_t byteCnt) {
	deb_sendFrame(data, byteCnt);
}
