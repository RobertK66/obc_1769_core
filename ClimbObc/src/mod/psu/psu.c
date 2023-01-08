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

//#include <ado_modules.h>
#include "../tim/obc_time.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "../libfixmath/fix16.h"

//#include "../../ClimbObc.h"


#define PSU_BUFFER_LENGTH 70 // Total size of PSU reply

//I2C receive buffer
static I2C_Data i2c_message; // create structure that will contain i2c message
static I2C_Data PSU_transmit_register_msg; // create structure that will contain i2c message
uint8_t PSU_receive_buff[PSU_BUFFER_LENGTH]; // Buffer that will keep datavector replied by PSU. Length of buffer defines amount of received bytes. MAX =86
uint8_t PSU_register_request[1]; // PSU starting access register This should be GLOBAL variable

eps_hk_data_t HK_DATA_PSU;
eps_hk_data_t eps_hk_data;

eps_settings_t eps_settings_data;

static bool readInProgress = false;

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
	// Moved layer 1 init to ClimbOBC.c
//	psu_InitData = (psu_i2c_initdata_t*) initData;

	// ENABLE A and B and C and D
//  (done in pinmuxing2[] of pinning_climbobc.h (init value)
//	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_A_EN), PINNR_FROM_IDX(PINIDX_I2C_A_EN)); //ENABLE I2C on X+ side
//	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_B_EN), PINNR_FROM_IDX(PINIDX_I2C_B_EN)); //ENABLE I2C on  Y- side

//	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_C_EN), PINNR_FROM_IDX(PINIDX_I2C_C_EN)); // ENABLE I2C on X- side
//	Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_I2C_D_EN), PINNR_FROM_IDX(PINIDX_I2C_D_EN)); //ENABLE I2C on Y+ side
//



	// Init I2C
//	init_i2c(psu_InitData->pI2C, psu_InitData->frequency);		// 100 kHz





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
					//i2c_buffer = &(HK_DATA_PSU.i_pv2_5v);

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


		PSU_register_request[0] = 2; // index of STARTING register at PSU datavector that is desired to be read from

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
		//or
		//i2c_message.rx_data = (uint8_t *) &(HK_DATA_PSU.i_pv2_5v);


		i2c_add_job(&i2c_message);


		//*************** Now we  try pegasys request code ********************

		eps_housekeeping_data_read(1);// this will add another job which will read houskeeping data from block 1


}



void old_pegasys_PSU_request_cmd(int argc, char *argv[]){

	/*
	 *
	 * Sends I2C read request to PSU using old pegasys function
	 * usage : b <block_nr>
	 * block_nr is {1,2,3,4}
	 *
	 * Blocks are defined in psu.h typedef struct eps_hk_data_s
	 */

	uint8_t block_nr = atoi(argv[1]);
	eps_housekeeping_data_read(block_nr);

}


//void eps_housekeeping_data_read(uint8_t cc, uint8_t block)
void eps_housekeeping_data_read( uint8_t block)
{
	/* Return all housekeeping data of the EPS. */
	static I2C_Data job_tx =
	{ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };
	static I2C_Data job_rx =
	{ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };
	static uint8_t tx[1];

	if (job_rx.job_done != 1)
	{
		/* Old job not finished - prevent job data from being  - please slow down
		if (cc == CC1)
		{
			obc_error_counters.i2c0_error_counter++;
		}
		else
		{
			obc_error_counters.i2c2_error_counter++;
		}*/

		eps_hk_data.data_valid = 0;
		job_rx.job_done = 1; // Set for next call
		job_rx.error = I2C_ERROR_JOB_NOT_FINISHED; // Set error to mark values old
		return;
	}

	if (job_rx.error != I2C_ERROR_NO_ERROR || job_tx.error != I2C_ERROR_NO_ERROR)
	{
		/* Transmission error */
		eps_hk_data.data_valid = 0;
	}
	else
	{
		eps_hk_data.data_valid = 1;
	}

	/*
	if (cc == CC1)
	{
		job_tx.device = LPC_I2C0;
		job_rx.device = LPC_I2C0;
	}
	else
	{
		job_tx.device = LPC_I2C2;
		job_rx.device = LPC_I2C2;
	}
	*/
	job_tx.device = LPC_I2C2;
	job_rx.device = LPC_I2C2;

	switch (block)
	{
	case 1:
		tx[0] = 0;
		job_rx.rx_data = (uint8_t *) &(eps_hk_data.i_pv2_5v);
		break;

	case 2:
		tx[0] = 16;
		job_rx.rx_data = (uint8_t *) &(eps_hk_data.temp_bat1_sw);
		break;

	case 3:
		tx[0] = 32;
		job_rx.rx_data = (uint8_t *) &(eps_hk_data.v_5v_out);
		break;

	case 4:
		tx[0] = 48;
		job_rx.rx_data = &(eps_hk_data.status_1);
		break;

	default:
		return;
	}
	job_tx.adress = I2C_ADR_EPS;
	job_tx.tx_data = tx;
	job_tx.tx_size = 1;
	job_tx.rx_size = 0;

	i2c_add_job(&job_tx);

	job_rx.adress = I2C_ADR_EPS;
	job_rx.tx_data = NULL;
	job_rx.tx_size = 0;
	job_rx.rx_size = 16; // block size is always 16 bytes

	if (i2c_add_job(&job_rx))
	{
		/*
		if (cc == CC1)
		{
			obc_error_counters.i2c0_error_counter++;
		}
		else
		{
			obc_error_counters.i2c2_error_counter++;
		}
		*/
	}
}



void eps_settings_read_all(uint8_t cc)
{

	// sentds i2c request for PSU settings
	// Loads data into eps_settings_data upon reception
	static I2C_Data job_tx =
	{ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };
	static I2C_Data job_rx =
	{ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };
	static uint8_t tx[1];

	if (job_rx.job_done != 1)
	{
		/* Old job not finished - prevent job data from being  - please slow down
		if (cc == CC1)
		{
			obc_error_counters.i2c0_error_counter++;
		}
		else
		{
			obc_error_counters.i2c2_error_counter++;
		}
		*/

		eps_settings_data.data_valid = 0;
		job_rx.job_done = 1; // Set for next call
		job_rx.error = I2C_ERROR_JOB_NOT_FINISHED; // Set error to mark values old
		return;
	}

	if (job_rx.error != I2C_ERROR_NO_ERROR || job_tx.error != I2C_ERROR_NO_ERROR)
	{
		/* Transmission error */
		eps_settings_data.data_valid = 0;
	}
	else
	{
		eps_settings_data.data_valid = 1;
	}

	/*
	if (cc == CC1)
	{
		job_tx.device = LPC_I2C0;
		job_rx.device = LPC_I2C0;
	}
	else
	{
		job_tx.device = LPC_I2C2;
		job_rx.device = LPC_I2C2;
	}
	*/
	job_rx.device = LPC_I2C2;
	job_rx.device = LPC_I2C2;

	tx[0] = 64; /* Current Limit HV ... Nr. 64 */

	job_tx.adress = I2C_ADR_EPS;
	job_tx.tx_data = tx;
	job_tx.tx_size = 1;
	job_tx.rx_data = NULL;
	job_tx.rx_size = 0;

	i2c_add_job(&job_tx);

	job_rx.adress = I2C_ADR_EPS;
	job_rx.tx_data = NULL;
	job_rx.tx_size = 0;
	job_rx.rx_data = &(eps_settings_data.current_lim_hv);
	job_rx.rx_size = 24;

	if (i2c_add_job(&job_rx))
	{
		/*
		if (cc == CC1)
		{
			obc_error_counters.i2c0_error_counter++;
		}
		else
		{
			obc_error_counters.i2c2_error_counter++;
		}
		*/
	}
}





/// fix conversion functions

//to convert from any format to fix16_t    which is q15_1
fix16_t q2_13_to_fix16(int16_t val)
{
	return ((((uint32_t) val) * 8)); // shift to  3
}

fix16_t uq3_13_to_fix16(uint16_t val)
{
	return ((((uint32_t) val) * 8));
}

fix16_t q7_8_to_fix16(int16_t val)
{
	return ((((uint32_t) val) * 256)); // shift to 8
}

fix16_t uq8_8_to_fix16(uint16_t val)
{
	return ((((uint32_t) val) * 256)); // shift to 8
}

fix16_t uq3_5_to_fix16(uint8_t val)
{
	return (((uint32_t) val) * 2048); // shift to 11
}

fix16_t uq3_4_to_fix16(uint8_t val)
{
	return (((uint32_t) val) * 4096); //shift to 12
}


void eps_rate_val(mval_t * mval, uint32_t id)
{
	if (mval == NULL)
	{
		/* Invalid pointer */
		return;
	}

	mval->val = 0;
	mval->quality = MV_VALUE_CRITICAL;

	switch (id)
	{
	/* Housekeeping data */
	case EPS_MVAL_HK_I_PV2_5V:
		mval->val = q2_13_to_fix16(eps_hk_data.i_pv2_5v);
		break;
	case EPS_MVAL_HK_I_PV1_5V:
		mval->val = q2_13_to_fix16(eps_hk_data.i_pv1_5v);
		break;
	case EPS_MVAL_HK_V_PV2:
		mval->val = uq3_13_to_fix16(eps_hk_data.v_pv2);
		break;
	case EPS_MVAL_HK_V_5V_IN:
		mval->val = uq3_13_to_fix16(eps_hk_data.v_5v_in);
		break;
	case EPS_MVAL_HK_I_PV1_3V3:
		mval->val = q2_13_to_fix16(eps_hk_data.i_pv1_3v3);//
		break;
	case EPS_MVAL_HK_I_PV2_3V3:
		mval->val = q2_13_to_fix16(eps_hk_data.i_pv2_3v3);//
		break;
	case EPS_MVAL_HK_V_PV1:
		mval->val = uq3_13_to_fix16(eps_hk_data.v_pv1);//
		break;
	case EPS_MVAL_HK_V_3V3_IN:
		mval->val = uq3_13_to_fix16(eps_hk_data.v_3v3_in);//
		break;
	case EPS_MVAL_HK_TEMP_BAT1SW:
		mval->val = q7_8_to_fix16(eps_hk_data.temp_bat1_sw);
		break;
	case EPS_MVAL_HK_TEMP_5V:
		mval->val = q7_8_to_fix16(eps_hk_data.temp_5v);
		break;
	case EPS_MVAL_HK_I_PV1_HV:
		mval->val = q2_13_to_fix16(eps_hk_data.i_pv1_hv);
		break;
	case EPS_MVAL_HK_I_PV2_HV:
		mval->val = q2_13_to_fix16(eps_hk_data.i_pv2_hv);
		break;
	case EPS_MVAL_HK_V_3V3_OUT:
		mval->val = uq3_13_to_fix16(eps_hk_data.v_3v3_out);
		break;
	case EPS_MVAL_HK_V_HV:
		mval->val = uq3_13_to_fix16(eps_hk_data.v_hv);
		break;
	case EPS_MVAL_HK_I_PV2_BAT1:
		mval->val = q2_13_to_fix16(eps_hk_data.i_pv2_bat1);
		break;
	case EPS_MVAL_HK_I_PV1_BAT1:
		mval->val = q2_13_to_fix16(eps_hk_data.i_pv1_bat1);
		break;
	case EPS_MVAL_HK_V_5V_OUT:
		mval->val = uq3_13_to_fix16(eps_hk_data.v_5v_out);
		break;
	case EPS_MVAL_HK_V_BAT1:
		mval->val = uq3_13_to_fix16(eps_hk_data.v_bat1);
		break;
	case EPS_MVAL_HK_I_PV2_BAT2:
		mval->val = q2_13_to_fix16(eps_hk_data.i_pv2_bat2);
		break;
	case EPS_MVAL_HK_I_PV1_BAT2:
		mval->val = q2_13_to_fix16(eps_hk_data.i_pv1_bat2);
		break;
	case EPS_MVAL_HK_VCC_MC:
		mval->val = uq3_5_to_fix16(eps_hk_data.vcc_mc);
		break;
	case EPS_MVAL_HK_TEMP_MC:
		mval->val = fix16_from_int((int) eps_hk_data.temp_mc);
		break;
	case EPS_MVAL_HK_V_BAT2:
		mval->val = uq3_13_to_fix16(eps_hk_data.v_bat2);
		break;
	case EPS_MVAL_HK_TEMP_BAT1:
		mval->val = q7_8_to_fix16(eps_hk_data.temp_bat1);
		break;
	case EPS_MVAL_HK_TEMP_BAT2:
		mval->val = q7_8_to_fix16(eps_hk_data.temp_bat2);
		break;
	case EPS_MVAL_HK_STATUS_1:
		mval->val = eps_hk_data.status_1;
		break;
	case EPS_MVAL_HK_STATUS_2:
		mval->val = eps_hk_data.status_2;
		break;
	case EPS_MVAL_HK_STATUS_3:
		mval->val = eps_hk_data.status_3;
		break;
	case EPS_MVAL_HK_STATUS_BAT1:
		mval->val = fix16_from_int((int) eps_hk_data.status_bat1);
		break;
	case EPS_MVAL_HK_STATUS_BAT2:
		mval->val = fix16_from_int((int) eps_hk_data.status_bat2);
		break;
	case EPS_MVAL_HK_REBOOT_MC:
		mval->val = fix16_from_int((int) eps_hk_data.reboot_mc);
		break;
	case EPS_MVAL_HK_REBOOT_CC1:
		mval->val = fix16_from_int((int) eps_hk_data.reboot_cc1);
		break;
	case EPS_MVAL_HK_REBOOT_CC2:
		mval->val = fix16_from_int((int) eps_hk_data.reboot_cc2);
		break;
	case EPS_MVAL_HK_VCC_CC1:
		mval->val = uq3_5_to_fix16(eps_hk_data.vcc_cc1);
		break;
	case EPS_MVAL_HK_TEMP_CC1:
		mval->val = fix16_from_int((int) eps_hk_data.temp_cc1);
		break;
	case EPS_MVAL_HK_VCC_CC2:
		mval->val = uq3_5_to_fix16(eps_hk_data.vcc_cc2);
		break;
	case EPS_MVAL_HK_TEMP_CC2:
		mval->val = fix16_from_int((int) eps_hk_data.temp_cc2);
		break;
	case EPS_MVAL_HK_STATUS_CC1:
		mval->val = eps_hk_data.status_cc1;
		break;
	case EPS_MVAL_HK_STATUS_CC2:
		mval->val = eps_hk_data.status_cc2;
		break;
	case EPS_MVAL_HK_CC_ID:
		mval->val = eps_hk_data.cc_id;
		break;
	case EPS_MVAL_HK_TBD:
		mval->val = eps_hk_data.empty2;
		break;
	//case EPS_MVAL_EPS_SOFTWARE_VERSION:
	//	mval->val = eps_hk_data.status_bat2;
	//	break;

		/* EPS settings */
	case EPS_MVAL_SETTING_CURRENT_LIMIT_HV:
		mval->val = uq3_4_to_fix16(eps_settings_data.current_lim_hv >> 1); /* Shift parity */
		break;
	case EPS_MVAL_SETTING_CURRENT_LIMIT_3V3_1:
		mval->val = uq3_4_to_fix16(eps_settings_data.current_lim_3v3_1 >> 1); /* Shift parity */
		break;
	case EPS_MVAL_SETTING_CURRENT_LIMIT_3V3_2:
		mval->val = uq3_4_to_fix16(eps_settings_data.current_lim_3v3_2 >> 1); /* Shift parity */
		break;
	case EPS_MVAL_SETTING_CURRENT_LIMIT_3V3_3:
		mval->val = uq3_4_to_fix16(eps_settings_data.current_lim_3v3_3 >> 1); /* Shift parity */
		break;
	case EPS_MVAL_SETTING_CURRENT_LIMIT_3V3_BACKUP:
		mval->val = uq3_4_to_fix16(eps_settings_data.current_lim_3v3_backup >> 1); /* Shift parity */
		break;
	case EPS_MVAL_SETTING_CURRENT_LIMIT_5V_1:
		mval->val = uq3_4_to_fix16(eps_settings_data.current_lim_1 >> 1); /* Shift parity */
		break;
	case EPS_MVAL_SETTING_CURRENT_LIMIT_5V_2:
		mval->val = uq3_4_to_fix16(eps_settings_data.current_lim_2 >> 1); /* Shift parity */
		break;
	case EPS_MVAL_SETTING_CURRENT_LIMIT_5V_3:
		mval->val = uq3_4_to_fix16(eps_settings_data.current_lim_3 >> 1); /* Shift parity */
		break;
	case EPS_MVAL_SETTING_CURRENT_LIMIT_5V_4:
		mval->val = uq3_4_to_fix16(eps_settings_data.current_lim_4 >> 1); /* Shift parity */
		break;
	case EPS_MVAL_SETTING_MC_FORCE_OUTPUT_VALUE_REGISTER_1:
		mval->val = (uint8_t)(eps_settings_data.mc_force_out_val_1 & 0xFF);
		break;
	case EPS_MVAL_SETTING_MC_FORCE_OUTPUT_VALUE_REGISTER_2:
		mval->val = (uint8_t)(eps_settings_data.mc_force_out_val_2 & 0xFF);
		break;
	case EPS_MVAL_SETTING_MC_FORCE_OUTPUT_VALUE_REGISTER_3:
		mval->val = (uint8_t)(eps_settings_data.mc_force_out_val_3 & 0xFF);
		break;
	case EPS_MVAL_SETTING_MC_OUTPUT_VALUE_REGISTER_1:
		mval->val = (uint8_t)(eps_settings_data.mc_out_val_1 & 0xFF);
		break;
	case EPS_MVAL_SETTING_MC_OUTPUT_VALUE_REGISTER_2:
		mval->val = (uint8_t)(eps_settings_data.mc_out_val_2 & 0xFF);
		break;
	case EPS_MVAL_SETTING_MC_OUTPUT_VALUE_REGISTER_3:
		mval->val = (uint8_t)(eps_settings_data.mc_out_val_3 & 0xFF);
		break;
	case EPS_MVAL_SETTING_CC1_FORCE_OUTPUT_VALUE_REGISTER:
		mval->val = (uint8_t)(eps_settings_data.cc1_force_out_val & 0xFF);
		break;
	case EPS_MVAL_SETTING_CC1_OUTPUT_VALUE_REGISTER:
		mval->val = (uint8_t)(eps_settings_data.cc1_output & 0xFF);
		break;
	case EPS_MVAL_SETTING_CC2_FORCE_OUTPUT_VALUE_REGISTER:
		mval->val = (uint8_t)(eps_settings_data.cc2_force_out_val & 0xFF);
		break;
	case EPS_MVAL_SETTING_CC2_OUTPUT_VALUE_REGISTER:
		mval->val = (uint8_t)(eps_settings_data.cc2_output & 0xFF);
		break;
	//case EPS_MVAL_SETTING_OBC_WATCHDOG:
	//	mval->val = (eps_settings_data.obc_watchdog & 0xFF);
	//	break;

	default:
		mval->val = 0;
		mval->quality = MV_VALUE_CRITICAL;
		return;
	}

	if (eps_hk_data.cc_id == 0xAB || eps_hk_data.cc_id == 0xAC)
	{
		mval->quality = MV_VALUE_GOOD;
	}
	else
	{
		mval->quality = MV_VALUE_CRITICAL;
	}

	if (id < EPS_MVAL_SETTING_CURRENT_LIMIT_HV)
	{
		/* Housekeeping data */
		if (eps_hk_data.data_valid == 0)
		{
			mval->quality = MV_VALUE_CRITICAL;
		}
	}
	else
	{
		/* Settings data */
		if (eps_settings_data.data_valid == 0)
		{
			mval->quality = MV_VALUE_CRITICAL;
		}
	}
}

