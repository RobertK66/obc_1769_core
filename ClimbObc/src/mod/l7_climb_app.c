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
#include <stdio.h>
#include "l2_debug_com.h"
#include <mod/ado_mram.h>
#include <mod/ado_sdcard.h>
#include <ado_crc.h>

#include "mem/obc_memory.h"
#include "tim/obc_time.h"
#include "l3_sensors.h"
#include "hw_check.h"
#include "tim/climb_gps.h"
#include "ai2c/obc_i2c.h"

#include "thr/thr.h"
//#include "crc/obc_checksums.h"
#include "l4_thruster.h"

#include "srs/radsensor.h"

#include "../radtest/radtest.h"

#include "modules_globals.h"

typedef struct {
	uint8_t	cmdId;
	void 	(*command_function)(int argc, char *argv[]);
} app_command_t;


typedef struct {
	uint32_t 				SerialShort;
	char					InstanceName[16];
	char					CardName[16];
//	tim_synced_systime_t 	CurrentTime;
	mem_status_t			MemoryStatus;
	uint32_t				SdCardBlock0Number;
	uint32_t				SdCardSize;
	uint32_t				SdCardUsed;
	uint32_t				SystemCommandCounter;
	uint32_t				SystemErrorCounter;
	char					SwRelease[16];
} app_systeminfo_t;

static uint32_t climbCmdCounter = 0;
static uint32_t climbErrorCounter = 0;


// Prototypes
void app_processCmd(int argc, char *argv[]);
void ReadMramCmd(int argc, char *argv[]);
void ReadMramFinished (uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
void WriteMramCmd(int argc, char *argv[]);
void WriteMramFinished (uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
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
void SetObcNameCmd(int argc, char *argv[]);
void ReadStatusMramCmd(int argc, char *argv[]);
void GetSystemInfoCmd(int argc, char *argv[]);
void SetSdCardNameCmd(int argc, char *argv[]);
void TriggerWatchdogCmd(int argc, char *argv[]);
void SetUtcDateTimeCmd(int argc, char *argv[]);
void GetFullTimeCmd(int argc, char *argv[]);
void SendToGpsUartCmd(int argc, char *argv[]);
void i2c_test_cmd(int argc, char *argv[]);
void main_showruntimes_cmd(int argc, char *argv[]);
void PSU_datavector_request(int argc, char *argv[]);
void old_pegasys_PSU_request_cmd(int argc, char *argv[]);

//extern void *sdCard;

static const app_command_t Commands[] = {
		{ 'h' , HwcSetOutputCmd },
		{ 'm' , HwcMirrorInputCmd },
		{ 'r' , ReadMramCmd },
		{ 'w' , WriteMramCmd }, // do not write at 0 .. 100 "Page0" is controlled by memory module !!!
		{ 'R' , ReadSdcardCmd },
		{ 'C' , CardPowerOnCmd },
		{ 'c' , CardPowerOffCmd },
		{ 's' , ReadAllSensorsCmd },
		{ 'p' , SpPowerCmd },
		{ 'O' , SetObcNameCmd },
		{ 'N' , SetSdCardNameCmd },
		//{ 'd' , TriggerWatchdogCmd },
		{ 't' , SetUtcDateTimeCmd },
		{ 'T' , GetFullTimeCmd },
		{ 'g' , SendToGpsUartCmd },
		{ '5' , ReadAllRegisters },
		{ '6' , GeneralReadRequest },
		{ '7' , GeneralSetRequest },
		{ '8' , thr_execute_sequence_cmd }, // 1st argument - thrust duration in ms
        { 'k' , PSU_datavector_request },
        { 'b' , old_pegasys_PSU_request_cmd },
		{ '9' , mem_write_cmd },
		{ 'Q' , srs_test_cmd },
		{ 'a' , main_showruntimes_cmd}

};


#define APP_CMD_CNT	(sizeof(Commands)/sizeof(app_command_t))

#define SysEventString(str) { \
	SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, str, strlen(str)); \
}


//static bool ictReadinProgress = false;
//static I2C_Data ictReadJob;
//static uint8_t ictReadTx[20];
//static uint8_t ictReadRx[20];
//
//void i2c_test_cmd(int argc, char *argv[]) {
//	if (ictReadinProgress) {
//			return;
//	}
//	ictReadinProgress = true;
//
//	ictReadJob.device = LPC_I2C0;
//	ictReadJob.tx_size = 1;
//	ictReadJob.tx_data = ictReadTx;
//	ictReadJob.rx_size = 0x10;
//	ictReadJob.rx_data = ictReadRx;
//	ictReadJob.adress = 0x20;
//	ictReadTx[0] = 0x00;		// adr
//	//ictReadTx[1] = 0x55;		// len
//
//
//	i2c_add_job(&ictReadJob);
//	return;
//}
//
//

void app_init (void *dummy) {
	//SdcCardinitialize(0);
	char ver[40] = "SW-Ver: ";
	ver[39] = 0;
	strncpy(&ver[8], BUILD_SWVERSION, 32);
	int pos = 8+strlen(BUILD_SWVERSION);
	if (pos < 39) {
		ver[pos] = ' ';
		strncpy(&ver[pos+1], BA_BOARD_REL, 38 - pos);
	}
	SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, ver, strlen(ver));
}

void app_main (void) {
	LAST_STARTED_MODULE = 9;
	// Debug Command Polling (direct from L2 CLI Module)
	DEB_L2_CMD_T cmd;
	if ( deb_getCommandIfAvailable(&cmd) ) {
		app_processCmd(cmd.parCnt, cmd.pars);
	}
	// handle event - queue ....

//	if (ictReadinProgress) {
//if (ictReadJob.job_done == 1) {
//			ictReadinProgress = false;
//			SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_RAWDATA, ictReadRx, 0x10);
//		}
//	}

}

// Defining this function here (or somewhere) is overwriting the week (and empty!) implementation from ado_modules.h(!)
// _SysEvent is the global event handler where all events raised by all modules (SYS_EVENT(...)) will arrive.
// The only thing available is the eventId and the raw data. To interpret the contents you have to know (by knowing the hopefully
// unique ID) who created this event and what structure it has.

// Because we are in Application layer(L7) here, it would be possible to include any module<xy>.h files and cast the event back to its original
// structure. At this moment we do not do this here. Instead we do send all events out to the debug UART.

// In order to be able to transmit any data structures (which could potentially contain any data bytes 0x00 ... 0xFF) to the debug UART there has to be
// a 'Layer3' protocol to handle this (At this moment there is a simple protocol used which has a Start-EndFrame (0x7E) token and escapes any occurrences of
// 0x7E and 0x7D in the data by using 0x7D as Escape token).
void _SysEvent(event_t event) {
	LAST_STARTED_MODULE = 901;
	// send all events to umbilical UART as debug frames
	deb_sendEventFrame(event.id, event.data, event.byteCnt);

	if ( (event.id.severity == EVENT_ERROR) || (event.id.severity == EVENT_FATAL)) {
		climbErrorCounter++;
	}
}

//void _SysEvent_Debug(event_t event) {
//	deb_sendEventFrame_Debug(event.data, event.byteCnt);
//
//}


uint8_t  tempData[MRAM_MAX_WRITE_SIZE];

void app_processCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 902;
	char* cmd = argv[0];

	for (int i=0; i<APP_CMD_CNT; i++) {
		if (cmd[0] == Commands[i].cmdId) {
			Commands[i].command_function(argc, argv);
			climbCmdCounter++;
			break;
		}
	}
}

static bool spOn[4]={false,false,false,false};
void SpPowerSwitch(char sp) {
	LAST_STARTED_MODULE = 903;
	uint8_t pinIdx = PINIDX_SP3_VCC_EN;
	bool *flag  = 0;
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

void SendToGpsUartCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 904;
	gpsSendBytes((uint8_t *)"1U2", 3);
}


void SpPowerCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 905;
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
	LAST_STARTED_MODULE = 906;
	memCardOn();
//	HwcSetOutput(PINIDX_SD_VCC_EN, HWC_Low);	// Sd Card Power On
//	SdcCardinitialize(0);		// initialize Card[0]
}

void CardPowerOffCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 907;
	memCardOff();
//	HwcSetOutput(PINIDX_SD_VCC_EN, HWC_High);
}

void TriggerWatchdogCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 908;
	while(true);
}


void ReadAllSensorsCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 909;
	SenReadAllValues();
	//SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_SENSORVALUES, &values, sizeof(sensor_values_t));
}

uint8_t tempBlockData[2000];
void ReadSdcardCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 910;
	if (argc != 2) {
		SysEventString("uasge: R <blockNr>")
	} else {
		// CLI params to binary params
		uint32_t  block = atoi(argv[1]);
		// memReadObcBlockAsync(block, tempBlockData, ReadSdcardFinished );
		SdcReadBlockAsync(0, block, tempBlockData, ReadSdcardFinished);
	 }
}

void ReadSdcardFinished (sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len) {
	LAST_STARTED_MODULE = 911;
    if (result == SDC_RES_SUCCESS) {
    	SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_RAWDATA, data, len);
    	//deb_sendFrame((uint8_t*)data, len);
    } else {
    	SysEventString("ERROR  !!!");
    }
}

void WriteSdcardCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 912;

}


void ReadMramCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 913;
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
		MramReadAsync(idx, adr, tempData, len, ReadMramFinished);
	 }
}

void ReadMramFinished (uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	LAST_STARTED_MODULE = 914;
    if (result == MRAM_RES_SUCCESS) {
    	SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_RAWDATA, data, len);
    	//deb_sendFrame((uint8_t*)data, len);
    } else {
    	char *str="ERROR  !!!";
    	SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, str,strlen(str));
    }
}



void WriteMramCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 915;
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
		MramWriteAsync(idx, adr, tempData, len,  WriteMramFinished);
	}
}


void WriteMramFinished (uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	LAST_STARTED_MODULE = 916;
	if (result == MRAM_RES_SUCCESS) {
		SysEventString("SUCCESS");
	} else {
		SysEventString("Write ERROR!!!");
	}
}


void HwcSetOutputCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 917;
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
	LAST_STARTED_MODULE = 918;
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

void SetObcNameCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 919;
	if (argc != 2) {
		SysEventString("uasge: O <instanceName>");
	} else {
		memChangeInstanceName(argv[1]);
	 }
}


void SetSdCardNameCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 920;
	if (argc != 2) {
		SysEventString("uasge: N <cardName>");
	} else {
		memChangeCardName(argv[1]);
	 }
}


void GetSystemInfoCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 921;
	app_systeminfo_t info;
	//info.CurrentTime = tim_getSystemTime();
	memGetInstanceName(info.InstanceName,16);
	memGetCardName(info.CardName, 20);

	memset(info.SwRelease, 0, 16);
	strncpy(info.SwRelease, BUILD_SWVERSION , 16);

	info.MemoryStatus = memGetStatus();
	mem_sdcobcdataarea_t *obcarea = memGetObcArea();
	info.SdCardBlock0Number = obcarea->basisBlockNumber;
	info.SdCardSize = obcarea->blocksAvailable;
	info.SdCardUsed = obcarea->blocksUsed;
	info.SystemCommandCounter = climbCmdCounter;
	info.SystemErrorCounter = climbErrorCounter;
	info.SerialShort = memGetSerialNumber(1);

	SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_SYSTEMSTATUS, &info, sizeof(info));
}

void SetUtcDateTimeCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 922;
	// setTime <year><Month><day><hours><minutes><seconds> as single uint32
	uint16_t year = 0;
	uint8_t month = 0;
	uint8_t dayOfMonth = 0;
	uint8_t sec = 0;
	uint8_t min = 0;
	uint8_t hrs = 0;

	uint32_t date;
	uint32_t time;

	if (argc != 3) {
		SysEventString("uasge: t <date> <time>");
	} else {
		date = atol(argv[1]);
		time = atol(argv[2]);

		year = date / 10000;
		date %= 10000;

		month = date / 100;
		dayOfMonth = date % 100;

		hrs = time / 10000;
		time %= 10000;

		min = time / 100;
		sec = time % 100;

		// binary cmd
		TimSetUtc1(year, month, dayOfMonth, hrs, min, sec, true, TIM_SYNCSOURCE_DEBUGCMD);
	}
}


void GetFullTimeCmd(int argc, char *argv[]) {
	LAST_STARTED_MODULE = 923;
	obc_utc_fulltime_t ft = timGetUTCTime();
	SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_FULLTIMEINFO, &ft, sizeof(ft));
}

