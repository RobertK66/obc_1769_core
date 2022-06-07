/*
===============================================================================
 Copy paste from l3_sensor by       : Jevgeni Potulov
 Created on	 : 07.06.2022
===============================================================================
*/

#include "i2c_arduino.h"
#include <math.h>
#include <float.h>
#include <chip.h>
#include <ado_crc.h>
#include <ado_adc.h>

#include <ado_modules.h>

#include "../ai2c/obc_i2c.h"
#include "../l7_climb_app.h"
#include "../l2_debug_com.h"



static bool readInProgress = false;
static I2C_Data readJob;
static uint8_t readRx[8];

static i2c_arduino_initdata_t *i2c_arduino_InitData;



void i2c_arduino_init (void *initData) {
	i2c_arduino_InitData = (i2c_arduino_initdata_t*) initData;



	// Init I2C
	init_i2c(i2c_arduino_InitData->pI2C, i2c_arduino_InitData->frequency);		// 100 kHz


	// send test request with initialization

	bool init_test = i2cArduino_SendReadRequest();

}

void i2c_arduino_main() { // in main we check for active read jobs
	if (readInProgress) {
		if (readJob.job_done == 1) {
			readInProgress = false;

			if (readJob.error == I2C_ERROR_NO_ERROR) {
				// if no errors

				// parse data
				//uint8_t byte0 = readRx[0];
				//uint8_t byte1 = readRx[1];
				//uint8_t byte2 = readRx[2];
				//uint8_t byte3 = readRx[3];

				//  lets just print the received buffer string
				SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, readRx, strlen(readRx));

			} // end if no errors

		} //end if job done
	}//end if read in progress
}






bool i2cArduino_SendReadRequest() {  //when send i2c read request we add READ JOB expecting replly bytes
	if (readInProgress) {
		return false;
	}
	readInProgress = true;

	///////////////request that is intender to be send
	uint8_t read_request[2];
	read_request[0] = 0xFF;
	read_request[1] = 0x00;
	///////////////////////////


	readJob.device = LPC_I2C1;
	readJob.tx_size = sizeof(read_request)/sizeof(uint8_t); // number of entries in read request array
	readJob.tx_data = read_request;
	readJob.rx_size = 6;
	readJob.rx_data = readRx;
	readJob.adress = 4;

	i2c_add_job(&readJob);
	return true;
}

