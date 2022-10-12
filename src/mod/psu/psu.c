/*
===============================================================================
 Copy paste from l3_sensor by       : Jevgeni Potulov
 Created on	 : 07.06.2022
===============================================================================
*/

#include "psu.h"
#include <math.h>
#include <float.h>
#include <chip.h>
#include <ado_crc.h>
#include <ado_adc.h>

#include <ado_modules.h>



//I2C receive buffer
static I2C_Data i2c_message; // create structure that will contain i2c message
static I2C_Data PSU_transmit_register_msg; // create structure that will contain i2c message
uint8_t PSU_receive_buff[3]; // Buffer that will keep datavector replied by PSU. Length of buffer defines amount of received bytes. MAX =86
uint8_t PSU_register_request[1]; // PSU starting access register This should be GLOBAL variable




static i2c_arduino_initdata_t *i2c_arduino_InitData;





void i2c_arduino_init (void *initData) {
	i2c_arduino_InitData = (i2c_arduino_initdata_t*) initData;

	// ENABLE A and B and C and D

	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_A_EN), PINNR_FROM_IDX(PINIDX_I2C_A_EN)); //ENABLE I2C on X+ side
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_B_EN), PINNR_FROM_IDX(PINIDX_I2C_B_EN)); //ENABLE I2C on  Y- side

	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_C_EN), PINNR_FROM_IDX(PINIDX_I2C_C_EN)); // ENABLE I2C on X- side
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_D_EN), PINNR_FROM_IDX(PINIDX_I2C_D_EN)); //ENABLE I2C on Y+ side




	// Init I2C
	init_i2c(i2c_arduino_InitData->pI2C, i2c_arduino_InitData->frequency);		// 100 kHz





}

void i2c_arduino_main() { // in main we check for active read jobs



	i2c_Proccess_Received_Buffer(i2c_message, PSU_receive_buff,sizeof(PSU_receive_buff)/sizeof(uint8_t)); // proccess buffer in the main of module








}// end main










void i2c_Proccess_Received_Buffer(I2C_Data i2cJob, uint8_t *i2c_buffer,uint8_t i2c_buffer_len){


	if (readInProgress) {
			if (i2cJob.job_done == 1) {
				readInProgress = false;

				if (i2cJob.error == I2C_ERROR_NO_ERROR) {

					// do stuff with received buffer
					//print received buffer
					//i2c_debugPrintBuffer(i2c_buffer,i2c_buffer_len);
					deb_print_pure_debug(i2c_buffer, i2c_buffer_len);

				} // end if no errors

			} //end if job done




		}//end if read in progress



}


void PSU_datavector_request(int argc, char *argv[]){

	if (readInProgress) {
			return;
		}
		readInProgress = true;


		PSU_register_request[0] = 3;

		PSU_transmit_register_msg.tx_data=PSU_register_request;
		PSU_transmit_register_msg.tx_size = sizeof(PSU_register_request)/sizeof(uint8_t);
		PSU_transmit_register_msg.tx_size = 1;
		PSU_transmit_register_msg.adress = 0b1010101; // address of device to which we send data
		PSU_transmit_register_msg.device = LPC_I2C2;// which I2C on which side

		i2c_add_job(&PSU_transmit_register_msg); // add job ??? and message is transmitted ? // unused wariable warning


		///////// PURE READ REQUEST


		i2c_message.adress = 0b1010101; // address of device to which we send data
		i2c_message.device = LPC_I2C2;// which I2C on which side

		i2c_message.rx_size = sizeof(PSU_receive_buff)/sizeof(uint8_t);
		i2c_message.rx_data = PSU_receive_buff;

		i2c_add_job(&i2c_message); // add job ??? and message is transmitted ? // unused wariable warning


}
