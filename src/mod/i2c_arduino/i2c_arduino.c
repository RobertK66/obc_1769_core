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
#include "../../ClimbObc.h"


static bool readInProgress = false;
static I2C_Data readJob;

uint8_t readRx[6];
uint8_t read2[6];

static i2c_arduino_initdata_t *i2c_arduino_InitData;

int i2c_delayCounter = 0;



void i2c_arduino_init (void *initData) {
	i2c_arduino_InitData = (i2c_arduino_initdata_t*) initData;

	// ENABLE A and B

	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_A_EN), PINNR_FROM_IDX(PINIDX_I2C_A_EN)); //ENABLE I2C on X+ side
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_B_EN), PINNR_FROM_IDX(PINIDX_I2C_B_EN)); //ENABLE I2C on  Y- side

	Chip_GPIO_SetPinOutLow(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_C_EN), PINNR_FROM_IDX(PINIDX_I2C_C_EN)); // ENABLE I2C on X- side
	Chip_GPIO_SetPinOutLow(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_D_EN), PINNR_FROM_IDX(PINIDX_I2C_D_EN)); //ENABLE I2C on Y+ side

 /*
	read2[0]= 1;
	read2[1]= 2;
	read2[2]= 3;
	read2[3]= 4;
	read2[4]= 5;
	read2[5]= 6;

	i2c_debugPrintBuffer(&read2,6);
	*/




 /*
	if(i2c_arduino_InitData->pI2C == LPC_I2C0){

		//I2C0 is on C/D sides  (X-/ Y+)

		//enable C and D
		Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_C_EN), PINNR_FROM_IDX(PINIDX_I2C_C_EN));
		Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_D_EN), PINNR_FROM_IDX(PINIDX_I2C_D_EN));


	}

	if(i2c_arduino_InitData->pI2C == LPC_I2C2){
		//I2C2 is on A/B sides  (X+/ Y-)

		//enable A and B
		Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_A_EN), PINNR_FROM_IDX(PINIDX_I2C_A_EN));
		Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_B_EN), PINNR_FROM_IDX(PINIDX_I2C_B_EN));


		}

*/


	// Init I2C
	init_i2c(i2c_arduino_InitData->pI2C, i2c_arduino_InitData->frequency);		// 100 kHz


	// send test request with initialization

	bool init_test = i2cArduino_SendReadRequest();


	////////////////// FROM ADO LIB
		 //Chip_I2C_Init(I2C2);
	 	 //uint8_t *i2c_send_buff = "hello";
	 	 //uint8_t i2c_buffLen = sizeof(i2c_send_buff)/sizeof(uint8_t);
		//int i2c_test = Chip_I2C_MasterSend(I2C2, 4, &i2c_send_buff, i2c_buffLen);

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
				//i2c_debugPrintBuffer(&read2,6);
				i2c_debugPrintBuffer(&readRx,6);

			} // end if no errors

		} //end if job done
	}//end if read in progress


	///////// SEND I2C bytes every once in a while
	i2c_delayCounter++;
	if(i2c_delayCounter ==100000){
		i2c_delayCounter=0;
		i2cArduino_SendReadRequest();
		//i2cArduino_Read();

	}// end if delay



}// end main






bool i2cArduino_SendReadRequest() {  //when send i2c read request we add READ JOB expecting replly bytes
	if (readInProgress) {
		return false;
	}
	readInProgress = true;

	///////////////request that is intender to be send//////////////////


	uint8_t read_request[9];
	read_request[0] = 0x4f;
	read_request[1] = 0x42;
	read_request[2] = 0x43;
	read_request[3] = 0x5f;
	read_request[4] = 0x68;
	read_request[5] = 0x65;
	read_request[6] = 0x6c;
	read_request[7] = 0x6c;
	read_request[8] = 0x6f;

	///////////////////////////


	readJob.device = LPC_I2C2;
	readJob.tx_size = sizeof(read_request)/sizeof(uint8_t); // number of entries in read request array
	//readJob.tx_size = 9; // number of entries in read request array
	readJob.tx_data = read_request;
	readJob.rx_size = sizeof(readRx)/sizeof(uint8_t);
	readJob.rx_data = readRx;
	readJob.adress = 57;

	i2c_add_job(&readJob);
	return true;
}



bool i2cArduino_Read() {  //when send i2c read request we add READ JOB expecting replly bytes
	if (readInProgress) {
		return false;
	}
	readInProgress = true;


	readJob.device = LPC_I2C2;
	//readJob.tx_size = sizeof(read_request)/sizeof(uint8_t); // number of entries in read request array
	readJob.tx_size = 0; // number of entries in read request array
	//readJob.tx_data = read_request;
	readJob.rx_size = sizeof(read2)/sizeof(uint8_t);
	readJob.rx_data = read2;
	readJob.adress = 57;

	i2c_add_job(&readJob);
	return true;
}


void i2c_debugPrintBuffer(uint8_t *buffer,int bufferlen){

	//LPC_UART2 is debug UART

	for (int i=0;i<bufferlen;i++){
		Chip_UART_SendByte(LPC_UART2, buffer[i]);

	}

}
