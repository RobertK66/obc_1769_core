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
#include "mem/ado_sdcard.h"

#include "hw_check.h"

typedef struct {
	uint8_t	cmdId;
	void 	(*command_function)(int argc, char *argv[]);
} app_command_t;


// Prototypes
void app_processCmd(int argc, char *argv[]);
void ReadMramCmd(int argc, char *argv[]);
void ReadMramFinished (mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
void WriteMramCmd(int argc, char *argv[]);
void WriteMramFinished (mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
void ReadSdcardCmd(int argc, char *argv[]);
void ReadSdcardFinished (sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len);
void WriteSdcardCmd(int argc, char *argv[]);
//void WriteSdcardFinished (mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
void HwcSetOutputCmd(int argc, char *argv[]);
void HwcMirrorInputCmd(int argc, char *argv[]);

extern void *sdCard;

static const app_command_t Commands[] = {
		{ 'h' , HwcSetOutputCmd },
		{ 'm' , HwcMirrorInputCmd },
		{ 'r' , ReadMramCmd },
		{ 'w' , WriteMramCmd },
		{ 'R' , ReadSdcardCmd },
		{ 'W' , WriteSdcardCmd },
};
#define APP_CMD_CNT	(sizeof(Commands)/sizeof(app_command_t))

void app_init (void *dummy) {
	SdcCardinitialize(0);
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

	for (int i=0; i<APP_CMD_CNT; i++) {
		if (cmd[0] == Commands[i].cmdId) {
			Commands[i].command_function(argc, argv);
			break;
		}
	}


}

uint8_t tempBlockData[2000];


void ReadSdcardCmd(int argc, char *argv[]) {
	if (argc != 2) {
		deb_sendString("uasge: R <blockNr>");
	} else {
		// CLI params to binary params
		uint8_t  block = atoi(argv[1]);
		SdcReadBlockAsync(0, block, tempBlockData, ReadSdcardFinished);
	 }
}

void ReadSdcardFinished (sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len) {
    if (result == SDC_RES_SUCCESS) {
    	deb_sendFrame((uint8_t*)data, len);
    } else {
    	deb_sendString("ERROR  !!!");
    }
}

void WriteSdcardCmd(int argc, char *argv[]) {

}


void ReadMramCmd(int argc, char *argv[]) {
	if (argc != 4) {
		deb_sendString("uasge: r <chipIdx> <adr> <len>");
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
}

void ReadMramFinished (mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
    if (result == MRAM_RES_SUCCESS) {
    	deb_sendFrame((uint8_t*)data, len);
    } else {
    	deb_sendString("ERROR  !!!");
    }
}

void WriteMramCmd(int argc, char *argv[]) {
	if (argc != 5) {
		deb_sendString("uasge: w <chipidx> <adr> <databyte> <len>");
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
}


void WriteMramFinished (mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	if (result == MRAM_RES_SUCCESS) {
		deb_sendFrame((uint8_t*)"SUCCESS", 10);
	} else {
    	deb_sendString("Write ERROR!!!");
	}
}

void _SysEvent(uint8_t *data, uint16_t byteCnt) {
	deb_sendFrame(data, byteCnt);
}

void HwcSetOutputCmd(int argc, char *argv[]) {
	hwc_OutStatus stat = HWC_Signal_Slow;
	uint8_t idx = 0;
	if (argc > 1) {
		idx = atoi(argv[1]);
	}
	if (argc > 2) {
		stat = (hwc_OutStatus)atoi(argv[2]);
	}

	HwcSetOutput(idx, stat);
}

void HwcMirrorInputCmd(int argc, char *argv[]) {
	uint8_t idxIn  = 50;	//RBF
	uint8_t idxOut = 44;	//LED
	if (argc > 1) {
		idxIn = atoi(argv[1]);
	}
	if (argc > 2) {
		idxOut = atoi(argv[2]);
	}

	HwcMirrorInput(idxIn, idxOut);
}


