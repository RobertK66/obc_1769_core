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
#include "../tim/obc_time.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define PSU_BUFFER_LENGTH 70 // Total size of PSU reply

//I2C receive buffer
static I2C_Data i2c_message; // create structure that will contain i2c message
static I2C_Data PSU_transmit_register_msg; // create structure that will contain i2c message
uint8_t PSU_receive_buff[PSU_BUFFER_LENGTH]; // Buffer that will keep datavector replied by PSU. Length of buffer defines amount of received bytes. MAX =86
uint8_t PSU_register_request[1]; // PSU starting access register This should be GLOBAL variable

/*
typedef struct {

	// 5V voltage and current
	uint16_t v5_i1;
	uint16_t v5_i2;
	uint16_t v5_v1;
	uint16_t v5_v2;

	//3V voltage and current
	uint16_t v3_i1;
	uint16_t v3_i2;
	uint16_t v3_v1;
	uint16_t v3_v2;

	// Edge Temperature
	uint16_t tEdge_temp;
	//Center Temperature
	uint16_t tCenter_temp;

	// HV VI data
	uint16_t hv_i1;
	uint16_t hv_i2;

	uint16_t hv_v1;
	uint16_t hv_v2;

	// Battery 1V data
	uint16_t viBat1_i1;
	uint16_t viBat1_i2;

	// I dont like it ....
	// If PSU team change names, positions, etc..
	// Then OBC team would need to redesign whole structure again

	// And as far as I know - They will redesign PSU ...


} psu_data_t;

*/


// So instead of structure with defined names of variables - lets use
// THe same approach as for THRUSTER registers

uint8_t PSU_VALUE_UINT8;
uint16_t PSU_VALUE_UINT16;
double PSU_ACTUAL_VALUE;

double PSU_REGISTER_DATA[70];
										//0 1 2 3 4

										//1/2/3/4/5
const uint8_t PSU_REGISTER_LENGTH[70] = {0,0,2,0,2, //5
										 0,2,0,2,0, //10
										 2,0,2,0,2, //15
										 0,2,0,2,0, //20
										 2,0,2,0,2, //25
										 0,2,0,2,0, //30
										 2,0,2,0,2, // 35
										 0,2,0,2,0, //40
										 2,0,2,0,2, //45
										 0,2,0,2,0, //50

										 // Status registers ?
										 // I have no idea what are length of those registers ...
										 // POOR PSU DOCU !!
										 1,1,1,1,1, // 55
										 1,1,1,1,1, // 60
										 1,1,1,1,1, // 65

										 // datavector[68] - CHECKSUM
										 1,1,1,1,1, // 70
};


const double PSU_CONVERSION_DOUBLE[70] = { //1  / 2  / 3  / 4   /5
											1.0, 1.0, 1.0, 1.0, 1.0, //5
											1.0, 1.0, 1.0, 1.0, 1.0, //10
											1.0, 1.0, 1.0, 1.0, 1.0, //15
											1.0, 1.0, 1.0, 1.0, 1.0, //20
											1.0, 1.0, 1.0, 1.0, 1.0, //25
											1.0, 1.0, 1.0, 1.0, 1.0, //30
											1.0, 1.0, 1.0, 1.0, 1.0, //35
											1.0, 1.0, 1.0, 1.0, 1.0, //40
											1.0, 1.0, 1.0, 1.0, 1.0, //45
											1.0, 1.0, 1.0, 1.0, 1.0, //50
											1.0, 1.0, 1.0, 1.0, 1.0, //55
											1.0, 1.0, 1.0, 1.0, 1.0, //60
											1.0, 1.0, 1.0, 1.0, 1.0, //65
											1.0, 1.0, 1.0, 1.0, 1.0 //70

};


static psu_i2c_initdata_t *psu_InitData;





void PSU_ParseDataVector(uint8_t* received_data,uint16_t uint16_payload_length){

	char print_str[200];
	int len_print;
	double multiplier;
	uint8_t calculated_checksum = CRC8(received_data, uint16_payload_length); // length of an actual message is header (6) + payload length
	uint8_t received_checksum = received_data[68];


	if (calculated_checksum == received_checksum){

		// ok // do nothing

	}
	else{
		// incorrect checksumm - message should not be proccessed
		sprintf(print_str, "\n Incorrect checksum Received = %d  Calculated = %d \n",received_checksum,calculated_checksum );
		len_print = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len_print);
		//return; // checksum incorrect calculation on PSU side

	}






		uint8_t next_register_index = 2; // First usable index of datavector ! Note it is important
		uint8_t length_of_next_register = PSU_REGISTER_LENGTH[next_register_index];
		multiplier = PSU_CONVERSION_DOUBLE[next_register_index];

		//sprintf(print_str, "\n Start RI= %d_RI_len=%d  Conversion_multiplier = %.2f \n",next_register_index,length_of_next_register,multiplier  );
		//len_print = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len_print);

		int i = 0; // i referres to position in received_datavector, not register_index. Starting i is always 0.
		uint32_t parse_start_time = (uint32_t)timGetSystime();
		uint32_t current_time;
		while( next_register_index<uint16_payload_length ){

			//sprintf(print_str, "\nParse Iteration = %d length_of_next_register = %d \n", i,length_of_next_register);
			//len_print = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len_print);

			//sprintf(print_str, "\n Next RI= %d Next_RI_len=%d  Conversion_multiplier = %.2f \n",next_register_index,length_of_next_register,multiplier  );
			//len_print = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len_print);
			current_time = (uint32_t)timGetSystime();

			if((current_time - parse_start_time)> 100){
				/*
				 *
				 * This is exit condition to prevent infinite loop.
				 * In case thruster replies less bytes then expected - index will never reach exit condition.
				 * To prevent this we introduce break after 100ms delay
				 *
				 * This is not the most elegant solution. But it is assumed that ReadAllRegisters will request amount of bytes
				 * enough to cover all REGISTER_DATA. Thus in normal operation proccess time exceeded limit warning should never occure
				 *
				 */

				sprintf(print_str, "\nParse WARNING: Process time exceeded limit   next_register_index = %d  \n",next_register_index  );
				len_print = strlen(print_str);
				deb_print_pure_debug((uint8_t *)print_str, len_print);
				current_time = (uint32_t)timGetSystime();



				break;
			}


			if(length_of_next_register ==1){
				PSU_VALUE_UINT8 = received_data[next_register_index];
				multiplier = PSU_CONVERSION_DOUBLE[next_register_index];
				PSU_ACTUAL_VALUE = (double)PSU_VALUE_UINT8 / multiplier;
				PSU_REGISTER_DATA[next_register_index]=PSU_ACTUAL_VALUE;

				//sprintf(print_str, "\n [%d] ACTUAL_VALUE= %.6f Conversion=%.6f  uint8_value =%d  \n",next_register_index,PSU_ACTUAL_VALUE,multiplier,PSU_VALUE_UINT8  );
				//len_print = strlen(print_str);
				//deb_print_pure_debug((uint8_t *)print_str, len_print);

				next_register_index = next_register_index+1;

			}

			if(length_of_next_register==2){
				PSU_VALUE_UINT16 = (received_data[next_register_index+1] <<8 )| received_data[next_register_index]; // here problem / Compiles value out of wrong bytes
				multiplier = PSU_CONVERSION_DOUBLE[next_register_index];
				PSU_ACTUAL_VALUE = (double)PSU_VALUE_UINT16 / multiplier;
				PSU_REGISTER_DATA[next_register_index]=PSU_ACTUAL_VALUE;

				//sprintf(print_str, "\n [%d] ACTUAL_VALUE= %.6f Conversion=%.6f  uint16_value =%d  \n",next_register_index,PSU_ACTUAL_VALUE,multiplier,PSU_VALUE_UINT16  );
				//len_print = strlen(print_str);
				//deb_print_pure_debug((uint8_t *)print_str, len_print);

				next_register_index = next_register_index+2;


			}

			if(length_of_next_register==4){

				//sprintf(print_str, "\n [%d] Skip Fuse \n",next_register_index  );
				//len_print = strlen(print_str);
				//deb_print_pure_debug((uint8_t *)print_str, len_print);

				next_register_index = next_register_index+4;

			}
			length_of_next_register = PSU_REGISTER_LENGTH[next_register_index];
			multiplier = PSU_CONVERSION_DOUBLE[next_register_index];

			//sprintf(print_str, "\n Parse Iteration End: len_next_RI=%d   next_RI=[%d]  \n",length_of_next_register,next_register_index );
			//len_print = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len_print);

			i = i+1;

		}






}




void psu_init (void *initData) {
	psu_InitData = (psu_i2c_initdata_t*) initData;

	// ENABLE A and B and C and D

	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_A_EN), PINNR_FROM_IDX(PINIDX_I2C_A_EN)); //ENABLE I2C on X+ side
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_B_EN), PINNR_FROM_IDX(PINIDX_I2C_B_EN)); //ENABLE I2C on  Y- side

	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_C_EN), PINNR_FROM_IDX(PINIDX_I2C_C_EN)); // ENABLE I2C on X- side
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_D_EN), PINNR_FROM_IDX(PINIDX_I2C_D_EN)); //ENABLE I2C on Y+ side




	// Init I2C
	init_i2c(psu_InitData->pI2C, psu_InitData->frequency);		// 100 kHz





}

void psu_main() { // in main we check for active read jobs



	//i2c_Proccess_Received_Buffer(i2c_message, PSU_receive_buff,sizeof(PSU_receive_buff)/sizeof(uint8_t)); // proccess buffer in the main of module
	i2c_Proccess_Received_Buffer(i2c_message, PSU_receive_buff,PSU_BUFFER_LENGTH); // proccess buffer in the main of module







}// end main










void i2c_Proccess_Received_Buffer(I2C_Data i2cJob, uint8_t *i2c_buffer,uint8_t i2c_buffer_len){


	if (readInProgress) {
			if (i2cJob.job_done == 1) {
				readInProgress = false;

				if (i2cJob.error == I2C_ERROR_NO_ERROR) {

					// do stuff with received buffer
					//print received buffer
					deb_print_pure_debug(i2c_buffer, i2c_buffer_len);

					PSU_ParseDataVector(i2c_buffer,i2c_buffer_len);

				} // end if no errors

			} //end if job done




		}//end if read in progress



}


void PSU_datavector_request(int argc, char *argv[]){

	// Request full data vector at once
	// Starting register index =0
	// Length of i2c_receive buffer should be equal to length of datavector reply

	if (readInProgress) {
			return;
		}
		readInProgress = true;


		PSU_register_request[0] = 0; // index of STARTING register at PSU datavector that is desired to be read from

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
