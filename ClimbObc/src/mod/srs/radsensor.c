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




static srs_initdata_t* srs;

static bool srsReadinProgress = false;
static I2C_Data srsJob;
static uint8_t srsTx[20];
static uint8_t srsRx[20];

void srs_init(void *initdata) {
	srs = (srs_initdata_t*)initdata;
}

void srs_main() {
	if (srsReadinProgress) {
		if (srsJob.job_done == 1) {
			srsReadinProgress = false;
				SysEvent(MODULE_ID_RADSENSOR, EVENT_INFO, EID_SRS_DATARX, srsRx, srsJob.rx_size);
			}
		}

}

void srs_test_cmd(int argc, char *argv[]) {
	uint8_t readAdr = 0;
	uint8_t readLen = 10;

	if (srsReadinProgress) {
			return;
	}
	srsReadinProgress = true;

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
	srsJob.adress = srs->i2cAdr;
	srsTx[0] = readAdr;				// data read adr

	i2c_add_job(&srsJob);
	return;
}
