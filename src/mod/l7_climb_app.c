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

#include "l3_sensors.h"
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
void ReadAllSensorsCmd(int argc, char *argv[]);
void SpPowerCmd(int argc, char *argv[]);
void CardPowerOnCmd(int argc, char *argv[]);
void CardPowerOffCmd(int argc, char *argv[]);


//extern void *sdCard;

static const app_command_t Commands[] = {
		{ 'h' , HwcSetOutputCmd },
		{ 'm' , HwcMirrorInputCmd },
		{ 'r' , ReadMramCmd },
		{ 'w' , WriteMramCmd },
		{ 'R' , ReadSdcardCmd },
		{ 'C' , CardPowerOnCmd },
		{ 'c' , CardPowerOffCmd },
		//{ 'W' , WriteSdcardCmd },
		{ 's' , ReadAllSensorsCmd },
		{ 'p' , SpPowerCmd },
};
#define APP_CMD_CNT	(sizeof(Commands)/sizeof(app_command_t))

#define SysEventString(str) { \
		SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, str, strlen(str)); \
}

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


void _SysEvent(event_t event) {
	//deb_sendFrame(event.data, event.byteCnt);
	deb_sendEventFrame(event.id, event.data, event.byteCnt);
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

static bool spOn[4]={false,false,false,false};
void SpPowerSwitch(char sp) {
	uint8_t pinIdx = PINIDX_SP3_VCC_EN;
	bool *flag;
	if ((sp=='a')||(sp=='A')) {
		pinIdx = PINIDX_SP1_VCC_EN;
		flag = &spOn[0];
	} else if ((sp=='b')||(sp=='B')) {
		pinIdx = PINIDX_SP2_VCC_EN;
		flag = &spOn[1];
	} else if ((sp=='c')||(sp=='C')) {
		pinIdx = PINIDX_SP3_VCC_EN;
		flag = &spOn[2];
	} else if ((sp=='d')||(sp=='D')) {
		pinIdx = PINIDX_SP4_VCC_EN;
		flag = &spOn[3];
	}
	hwc_OutStatus pinStat = HWC_Low;
	*flag = true;
	if ((sp=='a')||(sp=='b')||(sp=='c')||(sp=='d')) {
		pinStat = HWC_High;
		*flag = false;
	}
	HwcSetOutput(pinIdx, pinStat);
	if (spOn[0]||spOn[1]||spOn[2]||spOn[3]) {
		HwcMirrorInput(PINIDX_SP_VCC_FAULT, PINIDX_LED);		// this uses idx in pinmuxing2 s5tructure. Mirror VPP_FAULT (55) -> LED (44)
	} else {
		HwcMirrorInput(200, 200);	// Mirror off
		HwcSetOutput(PINIDX_LED, HWC_Low);	// Led Off
	}
}

void SpPowerCmd(int argc, char *argv[]) {
	if (argc != 2) {
		SysEventString("uasge: p <a|A/b|B/c|C/d|D>");
	} else {
		int i = strlen(argv[1]);
		if (i > 4) {
			i = 4;
		}
		for (int x= 0; x<i;x++) {
			char sp = argv[1][x];
			SpPowerSwitch(sp);
		}
	}
}

void CardPowerOnCmd(int argc, char *argv[]) {
	HwcSetOutput(PINIDX_SD_VCC_EN, HWC_Low);	// Sd Card Power On
	SdcCardinitialize(0);		// initialize Card[0]
}

void CardPowerOffCmd(int argc, char *argv[]) {
	HwcSetOutput(PINIDX_SD_VCC_EN, HWC_High);
}


void ReadAllSensorsCmd(int argc, char *argv[]) {
	sensor_values_t values = SenReadAllValues();
	SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_SENSORVALUES, &values, sizeof(sensor_values_t));
}

uint8_t tempBlockData[2000];
void ReadSdcardCmd(int argc, char *argv[]) {
	if (argc != 2) {
		SysEventString("uasge: R <blockNr>")
	} else {
		// CLI params to binary params
		uint8_t  block = atoi(argv[1]);
		SdcReadBlockAsync(0, block, tempBlockData, ReadSdcardFinished);
	 }
}

void ReadSdcardFinished (sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len) {
    if (result == SDC_RES_SUCCESS) {
    	SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_RAWDATA, data, len);
    	//deb_sendFrame((uint8_t*)data, len);
    } else {
    	SysEventString("ERROR  !!!");
    }
}

void WriteSdcardCmd(int argc, char *argv[]) {

}


void ReadMramCmd(int argc, char *argv[]) {
	if (argc != 4) {
		SysEventString("uasge: r <chipIdx> <adr> <len>");
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
    	SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_RAWDATA, data, len);
    	//deb_sendFrame((uint8_t*)data, len);
    } else {
    	char *str="ERROR  !!!";
    	SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, str,strlen(str));
    }
}

void WriteMramCmd(int argc, char *argv[]) {
	if (argc != 5) {
		char *str="uasge: w <chipidx> <adr> <databyte> <len>";
		SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, str,strlen(str));
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
		SysEventString("SUCCESS");
	} else {
		SysEventString("Write ERROR!!!");
	}
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
	uint8_t idxIn  = PINIDX_RBF;
	uint8_t idxOut = PINIDX_LED;	//LED
	if (argc > 1) {
		idxIn = atoi(argv[1]);
	}
	if (argc > 2) {
		idxOut = atoi(argv[2]);
	}

	HwcMirrorInput(idxIn, idxOut);
}

