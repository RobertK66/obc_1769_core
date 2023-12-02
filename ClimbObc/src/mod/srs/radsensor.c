/*
 * radsensor.c
 *
 *  Created on: 19.05.2023
 *      Author: Robert
 */

#include "radsensor.h"
#include <stdlib.h>

#include <ado_modules.h>
#include "../ai2c/obc_i2c.h"

#define SRS_POWERBUS_ADDR		0x47
#define SRS_POWERBUS_ENABLECMD	0x1F
#define SRS_POWERBUS_DISABLECMD	0x17

#define SRS_CTRL_ADDR			0x0B

#define SRS_CTRLCMD_SETSYNCTIME		0x0D
#define SRS_CMDEXEC_SETSYNCTIME		0x00000001

#define SRS_CTRLCMD_REQSTATUS		0x0A
#define SRS_STATUSTYPE_COMMON		0x01
#define SRS_STATUSTYPE_EXTENDED		0x02
#define SRS_STATUSTYPE_BOTH			0x03
#define SRS_CMDEXEC_REQSTATUS_C		0x00000002
#define SRS_CMDEXEC_REQSTATUS_E		0x00000004
#define SRS_CMDEXEC_REQSTATUS_B		0x00000008


#define SRS_CTRLCMD_SETINTERVALS	0x0B
#define SRS_STATUSTYPE_FGDOS		0x01
#define SRS_STATUSTYPE_RADFET		0x02
#define SRS_STATUSTYPE_SRAM			0x03
#define SRS_CMDEXEC_SETINTV_F		0x00000010
#define SRS_CMDEXEC_SETINTV_R		0x00000020
#define SRS_CMDEXEC_SETINTV_S		0x00000040

#define SRS_CTRLCMD_REQINTERVALS	0x0C
#define SRS_CMDEXEC_REQINTERVALS	0x00000080

#define SRS_CTRLCMD_REQSYNCTIME		0x0E
#define SRS_CMDEXEC_REQSYNCTIME		0x00000100

#define SRS_CTRLCMD_SHUTDOWN		0x0F
#define SRS_CMDEXEC_SHUTDOWN		0x00000200

#define SRS_CTRLCMD_REQDATAADDR		0x10
#define SRS_CMDEXEC_REQDATAADDR		0x00000400

#define SRS_CTRLCMD_REQDATA			0x11
#define SRS_DATACMD_INIT			0x00
#define SRS_DATACMD_TRANSFER		0x01
#define SRS_DATACMD_RESEND			0x02
#define SRS_DATACMD_STOP			0x03

#define SRS_DATARESP_BLOCKSIZEERROR		0x15
#define SRS_DATARESP_TRANSFERROR		0x14	// 'Not Initialized Error'
#define SRS_DATARESP_NOMMOREDATA		0x16
#define SRS_CMDEXEC_REQDATA			0x00000800		// Only one bit is enough here because ctrl of data transfer is done in other structures....


static uint8_t PowerOnCmd[2] = {0x00, SRS_POWERBUS_ENABLECMD};
static uint8_t PowerOffCmd[2] = {0x00, SRS_POWERBUS_DISABLECMD};

static srs_initdata_t* srs;

static bool srsPowerOn = false;
static bool srsRequestedPowerOn = false;

static bool srsJobInProgress = false;
static bool srsIsPowerJob = false;
static I2C_Data srsJob;


static uint32_t	srsCmdExecutes		= 0;
static uint32_t	srsPendingCmdExec 	= 0;
//static bool srsCmdSynctime = false;
static uint8_t srsTx[20];
static uint8_t srsRx[20];


// local module prototypes
uint8_t srs_crc(uint8_t *data, int len);

void srsExecuteSetSyncTime(void);


// Module API
// ----------
void srs_init(void *initdata) {
	srs = (srs_initdata_t*)initdata;
}

void srs_main() {
	if (srsJobInProgress) {
		// Wait for job finishing
		if (srsJob.job_done == 1) {
			srsJobInProgress = false;
			if (srsIsPowerJob) {
				srsIsPowerJob = false;
				// PowerJob finished.
				if (srsJob.tx_data[1] == SRS_POWERBUS_ENABLECMD) {
					srsPowerOn = true;
					SysEvent(MODULE_ID_RADSENSOR, EVENT_INFO, EID_SRS_POWERON, srsJob.tx_data, 2);
				} else {
					srsPowerOn = false;
					SysEvent(MODULE_ID_RADSENSOR, EVENT_INFO, EID_SRS_POWEROFF, srsJob.tx_data, 2);
				}
			} else {
				// Controller Job finished.
				if (srsJob.rx_size >= 3) {
					// Check for errors
					if (srsJob.rx_data[0] == 0xFF) {
						srsPendingCmdExec = 0;		// No valid answer to process
						SysEvent(MODULE_ID_RADSENSOR, EVENT_ERROR, EID_SRS_ERROR, &srsJob.rx_data[1], 1);
					}
				}
				switch (srsPendingCmdExec) {
					case SRS_CMDEXEC_SETSYNCTIME:
						SysEvent(MODULE_ID_RADSENSOR, EVENT_INFO, EID_SRS_TIME, srsJob.rx_data, srsJob.rx_size);
						break;
					default:
						break;
				}

//				if (srsJob.rx_size > 0) {
//					SysEvent(MODULE_ID_RADSENSOR, EVENT_INFO, EID_SRS_DATARX, srsJob.rx_data, srsJob.rx_size);
//				} else {
//					SysEvent(MODULE_ID_RADSENSOR, EVENT_INFO, EID_SRS_DATATX, srsJob.tx_data, srsJob.tx_size);
//				}
			}
		}
	} else {
		// Check for requested Power ON/OFF
		if (srsPowerOn != srsRequestedPowerOn) {
			srsJob.device = srs->pI2C;			// Redundant but lets do in sake of memory flips.....
			srsJob.adress = SRS_POWERBUS_ADDR;
			srsJob.tx_size = 2;
			srsJob.rx_size = 0;
			srsJob.rx_data = 0;
			if (srsRequestedPowerOn) {
				srsJob.tx_data = PowerOnCmd;
			} else {
				srsJob.tx_data = PowerOffCmd;
			}
			// Send Command to Power Bus Switch
			srsJobInProgress = true;
			srsIsPowerJob = true;
			i2c_add_job(&srsJob);
		} else if (srsCmdExecutes>0) {
			// There is/are cmd(s) requested
			// TODO: Priority not yet designed. Whatever sequence we should keep when more than one cmdexec is reuested can be handled here!
			if (srsCmdExecutes & SRS_CMDEXEC_SETSYNCTIME) {
				// Syncing the SRS module to current OBC timestamp.
				srsCmdExecutes &= (~SRS_CMDEXEC_SETSYNCTIME);
				srsPendingCmdExec = SRS_CMDEXEC_SETSYNCTIME;
				srsExecuteSetSyncTime();
			}
//			else if (false) {
//				// ....
//			}



		}
	}
}


void srs_enable(void) {
	srsRequestedPowerOn = true;
}

void srs_disable(void) {
	srsRequestedPowerOn = false;
}

void srs_synctime(void) {
	srsCmdExecutes |= SRS_CMDEXEC_SETSYNCTIME;
}


// Module local functions
// ----------------------
uint8_t srs_crc(uint8_t *data, int len) {
	uint8_t ret = 0x00;
	while (len>=0) {
		ret ^= data[len--];
	}
	return ret;
}


void srsExecuteSetSyncTime(void) {
	srsJob.device = srs->pI2C;			// Redundant but lets do in sake of memory flips.....
	srsJob.adress = SRS_CTRL_ADDR;
	uint64_t unixtime = timGetUnixTime();
	srsTx[0] = SRS_CTRLCMD_SETSYNCTIME;
	srsTx[1] =  *((uint8_t*)(&unixtime) + 0);
	srsTx[2] =  *((uint8_t*)(&unixtime) + 1);
	srsTx[3] =  *((uint8_t*)(&unixtime) + 2);
	srsTx[4] =  *((uint8_t*)(&unixtime) + 3);
	srsTx[5] =  *((uint8_t*)(&unixtime) + 4);
	srsTx[6] =  *((uint8_t*)(&unixtime) + 5);
	srsTx[7] =  *((uint8_t*)(&unixtime) + 6);
	srsTx[8] =  *((uint8_t*)(&unixtime) + 7);
	srsTx[9] =  srs_crc(srsTx, 8);
	srsJob.tx_size = 10;
	srsJob.tx_data = srsTx;
	srsJob.rx_size = 10;
	srsJob.rx_data = srsRx;
	srsJobInProgress = true;
	i2c_add_job(&srsJob);
}


// Module L7 API - TODO: move to higher level commanding layer (Scripting!)
// -------------
void srs_cmd(int argc, char *argv[]) {
	if (argc > 1) {
		char cmd = argv[1][0];
		switch(cmd) {
		case 'p':
			srs_disable();
			break;
		case 'P':
			srs_enable();
			break;
		case 't':
			srs_synctime();
			break;

		default:
			break;
		}
	}
}



void srs_testwrite_cmd(int argc, char *argv[]) {
	uint8_t writeAdr = 0;
	uint8_t writeLen = 10;

	if (srsJobInProgress) {
			return;
	}
	srsJobInProgress = true;

	if (argc > 1) {
		writeAdr = atol(argv[1]);
	}
	if (argc > 2) {
		writeLen = strlen(argv[2]);
		if (writeLen>10) {
			writeLen=10;
		}
		strncpy(&srsTx[1],argv[2],writeLen);
	}

	srsTx[0] = writeAdr;				// data write adr

	srsJob.device = srs->pI2C;
	srsJob.tx_size = writeLen + 1;
	srsJob.tx_data = srsTx;
	srsJob.rx_size = 0;
	srsJob.rx_data = 0;
	srsJob.adress = SRS_CTRL_ADDR;

	i2c_add_job(&srsJob);
	return;
}



void srs_testread_cmd(int argc, char *argv[]) {
	uint8_t readAdr = 0;
	uint8_t readLen = 10;

	if (srsJobInProgress) {
			return;
	}
	srsJobInProgress = true;

	if (argc > 1) {
		readAdr = atol(argv[1]);
	}
	if (argc > 2) {
		readLen = atol(argv[2]);
		if (readLen>20) {
			readLen=20;
		}
	}

	srsJob.device = srs->pI2C;
	srsJob.tx_size = 1;
	srsJob.tx_data = srsTx;
	srsJob.rx_size = readLen;
	srsJob.rx_data = srsRx;
	srsJob.adress = SRS_CTRL_ADDR;
	srsTx[0] = readAdr;				// data read adr

	i2c_add_job(&srsJob);
	return;
}
