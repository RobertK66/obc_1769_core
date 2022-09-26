/*
===============================================================================
 Name        : l4_thruster.c
 Author      : Jevgeni
 Created on	 : 30.06.2022

 Higher level logic layer to control thruster
===============================================================================
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#include <ado_crc.h>



#include "l7_climb_app.h"
#include "l4_thruster.h"

#include <string.h>
#include <stdlib.h>
#include "l2_debug_com.h"
#include <mod/ado_mram.h>
#include <mod/ado_sdcard.h>

#include "mem/obc_memory.h"
#include "tim/obc_time.h"
#include "l3_sensors.h"
#include "hw_check.h"
#include "tim/climb_gps.h"
#include "thr/thr.h"

#include "tim/obc_time.h"

#include <mod/ado_mram.h>
#include "modules_globals.h"


void thr_wait(int argc, char *argv[]);
void GeneralSetRequest_sequence(int argc, char *argv[]);
void GeneralReadRequest_sequence(int argc, char *argv[]);
void thr_execute_sequence();
void thr_void(int argc, char *argv[]);
void thr_value_ramp(int argc, char *argv[]);
//MEM
void thr_write_mem_callback(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
void thr_write_mem();
void thr_read_mem();
void thr_read_mem_callback(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
uint8_t MMRAM_READ_BUFFER[6];

#define MAX_EXECUTION_SEQUENCE_DEPTH 50 // Maximum size of execution sequence stack
#define MAX_HARDCODED_SEQUENCES 50 // Maximum number of preprogrammed sequences

typedef struct {
	char *thr_argv[6];
	void (*function)(int argc, char *argv[]);
	uint16_t procedure_id;
} thr_sequences_t;
//thr_sequences_t THR_SEQUENCES[MAX_EXECUTION_SEQUENCE_DEPTH]; // array of argv for sequence execution stack

typedef struct {
	thr_sequences_t *sequences;
	uint8_t length; // length of sequence
	uint16_t execution_index; // current index of execution stack
	uint32_t sequence_execution_begin; // timestamp at which sequence begin
	uint32_t sequence_execution_stage; // timestamp at which stage is completed
	bool sequence_trigger;
	bool repeat;
	bool pause;
	bool restart;
	int substage_index;
	double ramp_initial_value;

}thr_hardcoded_sequences_t;
thr_hardcoded_sequences_t THR_HARDCODED_SEQUENCES[5];

// Array of function pointers
//void (*THR_EXECUTION_SEQUENCE[MAX_EXECUTION_SEQUENCE_DEPTH])(int argc, char *argv[]); // sequence execution stack (array of function pointers)



// Conversion multipliers are factors used to transform input variable to an uint16_t value needed to be stored/read in/from THRUSTER REGISTER MAP.

// This is long monkey work to input the CONVERSION array with right factor values.  How its done described in Jevgenis report from 2021.
// You need to  use python feep tools to investigate the correct conversion value.

// We assume that Byte request to thruster that are send by "PYTHON FEEP TOOLS" are the ground truth.
// Use it to verify that byte request sent by OBC is in accordance with output from Python Feep Tools

// Its gonna be a loooong and boooring work to fill in this array......
const uint16_t CONVERSION[108] = {0,1,2,3,4,5,6,7,8,9,10,
		11,12,13,14,15,16,17,18,19,20,
		21,22,23,24,25,26,27,28,29,30,
		31,32,33,34,35,36,37,38,39,40,
		41,42,43,44,45,46,48,48,49,50,
		51,52,53,54,55,56,57,58,59,1,
		1000,62,63,64,10000,66,67,68,1000,70,
		71,72,73,74,75,76,77,78,79,80,
		81,82,83,84,85,86,87,88,89,90,
		91,92,93,94,95,96,100,98,99,100,
		101,102,103,104,105,106,107
		};



// REGISTER_DATA will store physical values after READ request
double REGISTER_DATA[108];

uint8_t VALUE_UINT8;
uint16_t VALUE_UINT16;
uint32_t VALUE_UINT32;
uint8_t LATEST_ACCESSED_REGISTER ;
double ACTUAL_VALUE;
uint8_t READ_REQUEST_OK_FLAG;
uint8_t TYPE_OF_LAST_REQUEST;


// CONVERSION MULTIPLIERS ARRAY. NOTE - VALUE CANNOT BE 0.  IF VALUE IS 0 in the array - meaning no information
//about this register in EMPULSION documentation !!!
const double CONVERSION_DOUBLE[108] = {1.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
		 0.0, 0.0, 0.0, 1.0, 1.0, 10000000.0, 0.0, 10000000.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1000.0, 0.0,
		  1000.0, 0.0, 10000.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 10000000.0, 0.0, 10000000.0, 0.0, 1000.0,
		   0.0, 1000.0, 0.0, 2.55, 2.55, 1.0, 1.0, 0.0, 1.0, 0.0, 100000000.0,
		    0.0, 100000000.0, 0.0, 10000.0, 0.0, 10000.0, 0.0, 2.55, 2.55, 1.0,
		     1000.0, 0.0, 1000.0, 0.0, 10000.0, 0.0, 10000.0, 0.0, 1000.0, 0.0, 1000.0, 0.0, 2.55,
		      2.55, 1.0, 1.0, 1.0, 1.0, 1.0, 100.0, 0.0, 10000.0, 0.0, 10000.0, 0.0, 1000.0, 0.0,
		       1000.0, 0.0, 2.55, 2.55, 10000000.0, 0.0, 10000000.0, 0.0, 1.0, 100.0,
		        0.0, 100.0, 0.0, 100.0, 0.0, 100.0, 0.0, 1000.0, 0.0, 1.0};



// REPRESENT LENGTH OF REGISTER ENTRY at corresponding register address
// NOTE IMPORTANT - REGISTER LENGTH CANNOT BE 0.  0 in the array means that there are no information
// avaliable about that register inside EMPULSION documentation !!!!!! 0 is missing info !!!!
const uint8_t REGISTER_LENGTH[108] = {1, 1, 2, 0, 2, 0, 4, 0, 0, 0, 4, 0, 0, 0, 1, 1, 2, 0, 2,
     0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0,
      1, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2,
       0, 1, 1, 1, 2, 0, 2, 0, 2, 0, 2, 0,
        2, 0, 2, 0, 1, 1, 1, 2, 0, 2, 0, 2,
         0, 2, 0, 2, 0, 2, 0, 1, 1, 1, 1, 1,
          1, 1, 2, 0, 2, 0, 2, 0, 2, 0, 2,
          0, 1, 1, 2, 0, 2, 0, 1, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2};









const uint8_t SENDER_ADRESS = 0x00;
const uint8_t DEVICE = 0xff;


const uint8_t MSGTYPE[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
// 0 - OK
// 1 - ERROR
// 2- READ
// 3 - WRITE
// 4 - DATA
// 5 - RESET
//6 - UPDATE
//7  -CONFIG



int l4_thr_ExpectedReceiveBuffer = 7;  // this is important variable that defines NEXT expected thruster reply buffer size.
// Every request function has defined known length of reply.  This will take lot of time to investigate how many bytes are replied by each request.
// Probably I will make an array of functions requests, and array of expected reply length requests.


int l4_thr_counter = 0; // counter for received bytes

void l4_thruster_init (void *dummy) {


	/// **************** PREPROGRAMM SEQUENCES HERE ****************



	char *wait_between_stages_str ="5000";

	//Build argv array for execution sequence

		static thr_sequences_t temp_sequence[MAX_EXECUTION_SEQUENCE_DEPTH];
		//1 SET SI
		temp_sequence[0].function = thr_wait;
		temp_sequence[0].thr_argv[0]= "0"; // FIRST ARGUMENT ALWAYS PROCES ID
		temp_sequence[0].thr_argv[1]= "20";
		temp_sequence[0].thr_argv[2]= "1500";
		temp_sequence[0].procedure_id = 0;


		//2 wait
		temp_sequence[1].function = thr_wait;
		temp_sequence[1].thr_argv[0]= "0";
		temp_sequence[1].thr_argv[1]= wait_between_stages_str;
		temp_sequence[1].thr_argv[2]= "0";
		temp_sequence[1].procedure_id = 0;

		//3 READ SI
		temp_sequence[2].function = GeneralSetRequest_sequence;
		temp_sequence[2].thr_argv[0]= "0";
		temp_sequence[2].thr_argv[1]= "20";
		temp_sequence[2].thr_argv[2]= "3000";
		temp_sequence[2].procedure_id = 0;

		//4 wait
		temp_sequence[3].function = thr_wait;
		temp_sequence[3].thr_argv[0]= "0";
		temp_sequence[3].thr_argv[1]= wait_between_stages_str;
		temp_sequence[3].thr_argv[2]= "0";
		temp_sequence[3].procedure_id = 0;

		//5 SET
		temp_sequence[4].function = GeneralReadRequest_sequence;
		temp_sequence[4].thr_argv[0]= "0";
		temp_sequence[4].thr_argv[1]= "20";
		temp_sequence[4].thr_argv[2]= "1750";
		temp_sequence[4].procedure_id = 0;

		//6 wait
		temp_sequence[5].function = thr_wait;
		temp_sequence[5].thr_argv[0]= "0";
		temp_sequence[5].thr_argv[1]= "1000";
		temp_sequence[5].thr_argv[2]= "0";
		temp_sequence[5].procedure_id = 0;

		//7 READ
		temp_sequence[6].function = GeneralSetRequest_sequence;
		temp_sequence[6].thr_argv[0]= "0";
		temp_sequence[6].thr_argv[1]= "20";
		temp_sequence[6].thr_argv[2]= "2500";
		temp_sequence[6].procedure_id = 0;

		//8 wait
		temp_sequence[7].function = thr_wait;
		temp_sequence[7].thr_argv[0]= "0";
		temp_sequence[7].thr_argv[1]= wait_between_stages_str;
		temp_sequence[7].thr_argv[2]= "0";
		temp_sequence[7].procedure_id = 0;

		//9 SET
		temp_sequence[8].function = thr_void;
		temp_sequence[8].thr_argv[0]= "0";
		temp_sequence[8].thr_argv[1]= "20";
		temp_sequence[8].thr_argv[2]= "2250";
		temp_sequence[8].procedure_id = 0;

		//10 wait
		temp_sequence[9].function = thr_wait;
		temp_sequence[9].thr_argv[0]= "0";
		temp_sequence[9].thr_argv[1]= wait_between_stages_str;
		temp_sequence[9].thr_argv[2]= "0";
		temp_sequence[9].procedure_id = 0;

		//11 READ
		temp_sequence[10].function = GeneralReadRequest_sequence;
		temp_sequence[10].thr_argv[0]= "0";
		temp_sequence[10].thr_argv[1]= "20";
		temp_sequence[10].thr_argv[2]= "0";
		temp_sequence[10].procedure_id = 0;

		//11 wait
		temp_sequence[11].function = thr_wait;
		temp_sequence[11].thr_argv[0]= "0";
		temp_sequence[11].thr_argv[1]= wait_between_stages_str;
		temp_sequence[11].thr_argv[2]= "0";
		temp_sequence[11].procedure_id = 0;

		//12 void
		temp_sequence[12].function = thr_wait;
		temp_sequence[12].thr_argv[0]= "0";
		temp_sequence[12].thr_argv[1]= "1000";
		temp_sequence[12].thr_argv[2]= "0";
		temp_sequence[12].procedure_id = 0;


		THR_HARDCODED_SEQUENCES[0].sequences = temp_sequence; // save sequence
		THR_HARDCODED_SEQUENCES[0].length = 12; // MANUALLY DEFINE LENGTH OF SEQUENCE //
		THR_HARDCODED_SEQUENCES[0].sequence_trigger = false;
		THR_HARDCODED_SEQUENCES[0].repeat = true;
		THR_HARDCODED_SEQUENCES[0].substage_index = 0; //DEFAULT SUBSTAGE INDEX





	//////////// ******** SEQUENCE 2***************
	static thr_sequences_t temp_sequence2[MAX_EXECUTION_SEQUENCE_DEPTH];


	//11 READ
			temp_sequence2[0].function = GeneralSetRequest_sequence;
			temp_sequence2[0].thr_argv[0]= "1";
			temp_sequence2[0].thr_argv[1]= "20";
			temp_sequence2[0].thr_argv[2]= "2550";
			temp_sequence2[0].procedure_id = 1;

			//11 wait
			temp_sequence2[1].function = thr_wait;
			temp_sequence2[1].thr_argv[0]= "1";
			temp_sequence2[1].thr_argv[1]= "2000";
			temp_sequence2[1].thr_argv[2]= "0";
			temp_sequence2[1].procedure_id = 1;

			//12 void
			temp_sequence2[2].function = GeneralReadRequest_sequence;
			temp_sequence2[2].thr_argv[0]= "1";
			temp_sequence2[2].thr_argv[1]= "20";
			temp_sequence2[2].thr_argv[2]= "0";
			temp_sequence2[2].procedure_id = 1;

			//12 void
			temp_sequence2[3].function = thr_wait;
			temp_sequence2[3].thr_argv[0]= "1";
			temp_sequence2[3].thr_argv[1]= "1000";
			temp_sequence2[3].thr_argv[2]= "0";
			temp_sequence2[3].procedure_id = 1;

			//12 void
			temp_sequence2[4].function = thr_void;
			temp_sequence2[4].thr_argv[0]= "1";
			temp_sequence2[4].thr_argv[1]= "0";
			temp_sequence2[4].thr_argv[2]= "0";
			temp_sequence2[4].procedure_id = 1;




			THR_HARDCODED_SEQUENCES[1].sequences = temp_sequence2; // save sequence
			THR_HARDCODED_SEQUENCES[1].length = 4; // MANUALLY DEFINE LENGTH OF SEQUENCE //
			THR_HARDCODED_SEQUENCES[1].sequence_trigger = false;
			THR_HARDCODED_SEQUENCES[1].repeat = false;
			THR_HARDCODED_SEQUENCES[1].substage_index = 0; //DEFAULT SUBSTAGE INDEX


		//////////// ******** SEQUENCE 3***************

			//thr_sequences_t temp_sequence[MAX_EXECUTION_SEQUENCE_DEPTH];
			static thr_sequences_t temp_sequence3[MAX_EXECUTION_SEQUENCE_DEPTH];

			//11 READ

			//11 wait
			temp_sequence3[0].function = GeneralSetRequest_sequence;
			temp_sequence3[0].thr_argv[0]= "2";
			temp_sequence3[0].thr_argv[1]= "20";
			temp_sequence3[0].thr_argv[2]= "3500";
			temp_sequence3[0].procedure_id = 2;

			//12 void
			temp_sequence3[1].function = thr_wait;
			temp_sequence3[1].thr_argv[0]= "2";
			temp_sequence3[1].thr_argv[1]= "3000";
			temp_sequence3[1].thr_argv[2]= "0";
			temp_sequence3[1].procedure_id = 2;

			temp_sequence3[2].function = thr_value_ramp;  // RAMP DOWN
			temp_sequence3[2].thr_argv[0]= "2"; //procedure id HARDCODED
			temp_sequence3[2].thr_argv[1]= "20"; // access register of a ramp (specific impulse)
			temp_sequence3[2].thr_argv[2]= "3000"; // GOAL of RAMP - manually set to 3000s
			temp_sequence3[2].thr_argv[3]= "100"; // ramp duration 30s
			temp_sequence3[2].thr_argv[4]= "10"; // 10 seconds between set requests
			temp_sequence3[2].procedure_id = 2;

			temp_sequence3[3].function = thr_value_ramp;  // RAMP UP
			temp_sequence3[3].thr_argv[0]= "2"; //procedure id HARDCODED
			temp_sequence3[3].thr_argv[1]= "20"; // access register of a ramp (specific impulse)
			temp_sequence3[3].thr_argv[2]= "3700"; // GOAL of RAMP - manually set to 3000s
			temp_sequence3[3].thr_argv[3]= "100"; // ramp duration 30s
			temp_sequence3[3].thr_argv[4]= "10"; // 10 seconds between set requests
			temp_sequence3[3].procedure_id = 2;



			//12 void
			temp_sequence3[4].function = thr_wait;
			temp_sequence3[4].thr_argv[0]= "2";
			temp_sequence3[4].thr_argv[1]= "2000";
			temp_sequence3[4].thr_argv[2]= "0";
			temp_sequence3[4].procedure_id = 2;

			//12 void
			temp_sequence3[5].function = GeneralReadRequest_sequence;
			temp_sequence3[5].thr_argv[0]= "2";
			temp_sequence3[5].thr_argv[1]= "20";
			temp_sequence3[5].thr_argv[2]= "3000";
			temp_sequence3[5].procedure_id = 2;

			//12 void
			temp_sequence3[6].function = thr_wait;
			temp_sequence3[6].thr_argv[0]= "2";
			temp_sequence3[6].thr_argv[1]= "1000";
			temp_sequence3[6].thr_argv[2]= "0";
			temp_sequence3[6].procedure_id = 2;

			//12 void
			temp_sequence3[7].function = GeneralReadRequest_sequence;
			temp_sequence3[7].thr_argv[0]= "2";
			temp_sequence3[7].thr_argv[1]= "20";
			temp_sequence3[7].thr_argv[2]= "1500";
			temp_sequence3[7].procedure_id = 2;


			THR_HARDCODED_SEQUENCES[2].sequences = temp_sequence3; // save sequence
			THR_HARDCODED_SEQUENCES[2].length = 7; // MANUALLY DEFINE LENGTH OF SEQUENCE //
			THR_HARDCODED_SEQUENCES[2].sequence_trigger = false;
			THR_HARDCODED_SEQUENCES[2].repeat = false;
			THR_HARDCODED_SEQUENCES[2].substage_index = 0; //DEFAULT SUBSTAGE INDEX














}

void l4_thruster_main (void) {
	LAST_STARTED_MODULE = 11;

	// TODO : I am not sure what to do here.
	// Leaving it for now so that l4_thruster_module can be
	// Initialised according as all other modules


	// this way we obtain time ms time ticks
	//obc_systime32_t timestamp= 	timGetSystime();
	//char print_str[20];
	//sprintf(print_str, "t = %d \n", timestamp);
	//uint8_t len = strlen(print_str);
	//deb_print_pure_debug((uint8_t *)print_str, len);




/*
	for (int i=0;i<=2;i++){ // for all preprogrammed sequences

		if (THR_HARDCODED_SEQUENCES[i].sequence_trigger ){ //if trigger for sequence is set to True - execute sequence
			thr_execute_sequence(i);

		}

	}
*/
	if (THR_HARDCODED_SEQUENCES[0].sequence_trigger ){ //if trigger for sequence is set to True - execute sequence
				thr_execute_sequence(0);

			}

	if (THR_HARDCODED_SEQUENCES[1].sequence_trigger ){ //if trigger for sequence is set to True - execute sequence
				thr_execute_sequence(1);

			}


	if (THR_HARDCODED_SEQUENCES[2].sequence_trigger ){ //if trigger for sequence is set to True - execute sequence
				thr_execute_sequence(2);

			}
}










/////  Note for pull request - I deleted all manual functions Example: SetHeaterCurrent()
// Because all of them can be implemented with functions for general requests



void ThrSendVersionRequestCmd(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1101;

	uint8_t request[8];
	request[0]= 0x00;
	request[1]= 0xFF;
	request[2]= 0x03;
	request[3]= 0x14;
	request[4]= 0x02;
	request[5]= 0x00;
	request[6]= 0x00;
	request[7]= 0x01;
	uint8_t len = sizeof(request);

	// every request function should manually set expected RX buffer size and reset byte counter !!!!!!!!!!
	l4_thr_ExpectedReceiveBuffer = 10;
	l4_thr_counter =0;
	thrSendBytes(request, len);
}




void ReadAllRegisters(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1102;


	uint8_t request[8];


	request[0]= 0x00;
	request[1]= 0xFF;
	request[2]= 0x03;
	request[3]= 0x69;
	request[4]= 0x02;
	request[5]= 0x00;
	request[6]= 0x00;
	request[7]= 0x7F;

	int len = sizeof(request);

	// every request function should manually set expected RX buffer size and reset byte counter !!!!!!!!!!
	l4_thr_ExpectedReceiveBuffer = 133;
	l4_thr_counter =0;

	thrSendBytes(request, len);
	LATEST_ACCESSED_REGISTER = 0;




}





// received_buffer of arbitrary size
// len length of actual thruster reply
void ParseReadRequest(uint8_t* received_buffer,int len){
	LAST_STARTED_MODULE = 1103;

	//uint8_t sender = received_buffer[0];
	//uint8_t receiver = received_buffer[1];
	//uint8_t message_type = received_buffer[2];
	uint8_t received_checksum = received_buffer[3];
	uint8_t payload_length_1 = received_buffer[4];
	uint8_t payload_length_2 = received_buffer[5];

	// Combine payload length bytes into uint16_t

	uint16_t uint16_payload_length = (payload_length_2 <<8 )| payload_length_1;

	//printf("\n PAYLOAD LENGTH = %d \n", uint16_payload_length  ); // correct

	/// after payload length is known - it is possible to parse remaining bytes into array

	uint8_t received_data[uint16_payload_length];

	for(int i=0;i<uint16_payload_length;i++){
		//printf("\n i=%d",i);

		received_data[i]=received_buffer[6+i]; //first 6 bytes of received buffer is header.
		//7th byte of received buffer is first byte of received data



	}

	// Now as payload length is know. We can extract remaining bytes from received_buffer, and calculate checksum to verify the received message

	// remember that when checksum is set - byte corresponding to checksum should be 0.
	// need to set byte corresponding to checksum to 0 before calculating checksumm. Received checksum is already stored
	received_buffer[3] = 0x00;
	uint8_t calculated_checksum = CRC8(received_buffer, 6+uint16_payload_length); // length of an actual message is header (6) + payload length
	//printf("\n Calculated checksum %00x ",calculated_checksum);

	if (calculated_checksum == received_checksum){

		// if calculated checksum is equal to received then message is considered validated.
		// Set flag that represent that latest received message was validated
		READ_REQUEST_OK_FLAG = 1;
		//printf("\n CHECKSUMM OK \n");

	}
	else{
		// incorrect checksumm - message should not be proccessed
		READ_REQUEST_OK_FLAG = 0;

		//printf("INCORRECT CHECKSUM \n");
		return;

	}



	// Now we have an array of received data.Which would be either 1,2 or 4 bytes in case of request to single register


	if(uint16_payload_length ==1){

		// if payload length is only 1 byte
		VALUE_UINT8 = received_data[0];
		// then obviously return type is uint8. And array of received_data would be of a single element.
		//printf("\n RECEIVED VALID MESSAGE VALUE_UINT8 = %d \n",VALUE_UINT8);


		// Now after we received correct value. We need to apply conversion multuplier again and convert uint8/uint16 value into double.
		//Store double in array representing REAL values of thruster registers

		// LAST ACCESSED REGISTER should for READ or SET REQUEST should be set by the function which sended the request.
		// It is implied that LAST_ACCESSED_REGISTER was defined correctly before executing ParseReadRequest
		double multiplier = CONVERSION_DOUBLE[LATEST_ACCESSED_REGISTER];
		ACTUAL_VALUE = (double)VALUE_UINT8 / multiplier;

		//printf("\n After conversion multiplier ACTUAL VALUE = %f \n",ACTUAL_VALUE);

		// Finally fill in the global array which will store ALL the register values
		REGISTER_DATA[LATEST_ACCESSED_REGISTER]=ACTUAL_VALUE;

	}

	if(uint16_payload_length ==2){

		//if payload length is 2 bytes, then return value should be stored as uint16_t
		VALUE_UINT16 = (received_data[1] <<8 )| received_data[0];
		//printf("\n RECEIVED VALID MESSAGE VALUE_UINT16 = %d \n ",VALUE_UINT16);

		double multiplier = CONVERSION_DOUBLE[LATEST_ACCESSED_REGISTER];
		ACTUAL_VALUE = (double)VALUE_UINT16 / multiplier;

		//printf("\n After conversion multiplier ACTUAL VALUE = %f \n",ACTUAL_VALUE);
		REGISTER_DATA[LATEST_ACCESSED_REGISTER]=ACTUAL_VALUE;


	}

	if(uint16_payload_length ==4){

		//if payload length is 4 (fuses) then return value should be stored with uint32_t
		//uint32_t i32 = v4[0] | (v4[1] << 8) | (v4[2] << 16) | (v4[3] << 24);

	}


	if(uint16_payload_length >5){

		//if payload is more then 4 then we are reading multiple registers
		//printf("READING MULTIPLE REGISTERS");

		// Lets say that READ REQUEST TO ALL REGISTERS/ READ MORE THEN ONE REGISTER function will set last address to a value
		// of a first address that is intended to be accessed
		uint8_t next_register_index = LATEST_ACCESSED_REGISTER;
		uint8_t length_of_next_register = REGISTER_LENGTH[next_register_index];
		double multiplier = CONVERSION_DOUBLE[next_register_index];

		for(int i=0;i<uint16_payload_length;i++){

			//char print_str[10];
			//sprintf(print_str, "i = %d \n", i);
			//uint8_t len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);

			if(length_of_next_register ==1){

				//printf("\n-1------------------Next Register Index = %d",next_register_index);
				VALUE_UINT8 = received_data[i];
				multiplier = CONVERSION_DOUBLE[next_register_index];
				ACTUAL_VALUE = (double)VALUE_UINT8 / multiplier;
				REGISTER_DATA[next_register_index]=ACTUAL_VALUE;
				next_register_index = next_register_index+1;
				//printf("\nActual Value =%f \n",ACTUAL_VALUE);
				//printf("\n multiplier =%f \n",multiplier);
				//printf("\n INTEGER Value =0x%00x\n",VALUE_UINT8);

			}

			if(length_of_next_register==2){

				//printf("\n 2------------Next Register Index = %d \n",next_register_index);
				VALUE_UINT16 = (received_data[i+1] <<8 )| received_data[i];
				double multiplier = CONVERSION_DOUBLE[next_register_index];
				ACTUAL_VALUE = (double)VALUE_UINT16 / multiplier;
				REGISTER_DATA[next_register_index]=ACTUAL_VALUE;
				next_register_index = next_register_index+2;
				//printf("\nActual Value =%f\n",REGISTER_DATA[next_register_index]);
				//printf("\n multiplier =%f\n",multiplier);
				//printf("\n INTEGER Value =0x%0000x\n",VALUE_UINT16);

			}

			if(length_of_next_register==4){
				//printf("\n Next Register Index = %d",next_register_index);
				next_register_index = next_register_index+4;

			}
			length_of_next_register = REGISTER_LENGTH[next_register_index];
			multiplier = CONVERSION_DOUBLE[next_register_index];

		}

	}





}



void GeneralSetRequest_sequence(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1104;
	/*
	 * _sequence is a wrapper arround General  thruster registor Read/Set request functions
	 *
	 * it passes input argv further into original function
	 *
	 * However after execution of original function pointer of execution sequence stack THR_EXECUTION_INDEX
	 * is increased so that next stage can be executed.
	 *
	 * after completion of sequence - timestamp is recorded
	 */
	uint16_t procedure_id = atoi(argv[0]); // procedure_id is always fist index of argument array
	GeneralSetRequest(3,argv);

	char print_str[200];
	sprintf(print_str, "\nStage SET index= %d completed\n",THR_HARDCODED_SEQUENCES[procedure_id].execution_index);
	int len = strlen(print_str);

	deb_print_pure_debug((uint8_t *)print_str, len);

	THR_HARDCODED_SEQUENCES[procedure_id].execution_index++;
	THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();


}


void GeneralSetRequest(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1105;



	uint8_t len; // will carry total length of request array

	// First argument should ne uint8_t value of register that are attmepted to write to.
	uint8_t access_register = atoi(argv[1]);
	//TYPE_OF_LAST_REQUEST = 4; // I have not yet decided how exactly to use type of last request.
	uint8_t length_of_register = REGISTER_LENGTH[access_register];

	//Check if valid/existing register is accessed
	if(length_of_register ==0){
		return;
		// It means that SET request is attempted with mismatch to actual origin of data register
		// Writing to wrong register position should be avoided.
		// For now lets assume that user never attempts to set wrong register.
		// TODO: Validate is register are allowed to be written/set to. (Based on enpulsion documentation)
	}

	//Request array initialisation
	if(length_of_register ==1){
		// if length of register is 1 byte then total length of request array is
		len=8;
		}

	if(length_of_register ==2){
		len=9;
		}

	if(length_of_register ==4){
		len=11;
		}


	uint8_t request[len];


	//Message header
	request[0] = SENDER_ADRESS;
	request[1] = DEVICE;
	request[2] = MSGTYPE[3]; // WRITE -3
	request[3] = 0x00; //checksumm


	//////Payload length
	if (length_of_register ==1){
		// if register that is intended to be set is one byte
		// then length of payload is register addres(1) + data(1)= 2 bytes
		request[4] = 2; //
	}

	if (length_of_register ==2){
		//if length of register is 2 bytes then
		//length of payload is address(1)+data(2) = 3 bytes
		request[4] =3;

	}

	if (length_of_register ==4){
			/// 4 bytes only for FUSE REGISTERS
			request[4] =5;

		}

	//For payload length uint16_t is used. However we represent uint16_t as two uint8_t bytes
	//IMPORTANT NOTE : For request to set a SINGLE register - payload length would obviously
	// never exceed 255.  Therefore second byte request[5] representing second part of uint16_t
	// would always be 0x00
	request[5] = 0x00;
	request[6]= access_register; // address of register that intended to be set


	// Parse arguments and input transformation using conversion multipliers array
	if (length_of_register ==1){
		uint8_t input_uint8 = atoi(argv[2]);

		//transform input using conversion multipliers array.
		// at all registers with only 1 byte data length conversion multiplier array
		// will also be an integer value. Therefore typecast of conversion multiplier
		//into uint8_t should also work
		input_uint8 = input_uint8* (uint8_t) CONVERSION_DOUBLE[access_register];
		request[7]= input_uint8;

	}

	if (length_of_register ==2){

		// parse argument as double
		// WARNING : Set Project-Settings-Manager linker script - Redlib (nohost) to use atof()
		double input = atof((const char*)argv[2]);
		// in cases where data stored in register has length of 2 bytes. uint16_t would be used
		// to store this data in register. For some registers input may be float
		// example 3.3V  or 3.0 or 3
		// Function should be able to handle both float and integer inputs from user

		// Apply conversion multiplier
		input = input* CONVERSION_DOUBLE[access_register];
		// Convert input to uint16_t
		uint16_t value = (uint16_t) input;
		// Represent uint16_t as two uint8_t bytes to fill the request array.
		request[7]= value & 0xff;
		request[8] = (value >> 8) & 0xff;
	}

	request[3] = CRC8(request, len); // calculate checksum after whole request array is sent
	l4_thr_ExpectedReceiveBuffer = 6;// change expected receive buffer accordingly
	thrSendBytes(request, len);
}


void GeneralReadRequest_sequence(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1106;
	uint16_t procedure_id = atoi(argv[0]); // procedure_id is always fist index of argument array
	GeneralReadRequest(3,argv);

	char print_str[200];
	sprintf(print_str, "\nStage READ index= %d completed\n",THR_HARDCODED_SEQUENCES[procedure_id].execution_index);
	int len = strlen(print_str);

	deb_print_pure_debug((uint8_t *)print_str, len);
	//THR_EXECUTION_INDEX++;
	//THR_HARDCODED_SEQUENCES[THR_PROCEDURE_ID]. = (uint32_t)timGetSystime(); // Save timestamp at which sequence stage finished

	THR_HARDCODED_SEQUENCES[procedure_id].execution_index++;
	THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();

}


//General read request to any register
void GeneralReadRequest(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1107;

		// FIRST ARGUMENT SHOUD BE int VALUE OF REGISTER THAT WOULD BE READ FROM
		uint8_t access_register = atoi(argv[1]);
		uint8_t length_of_register = REGISTER_LENGTH[access_register];

		// TODO : Check if valid/existing register is accessed
		if(length_of_register ==0){
			return;
		}

		uint8_t request[8];
		request[0] = SENDER_ADRESS;
		request[1] = DEVICE;
		request[2] = MSGTYPE[2]; // READ -3
		request[3] = 0x00; // checksum
		request[4] = 0x02; // LENGTH of payload REGISTER and length
		request[5] = 0x00; // Hardcoded because length of payload is defined with two bytes of uint16
		request[6]= access_register; // Access register at address requested by used
		request[7]= length_of_register;

		uint8_t len = sizeof(request);
		request[3] = CRC8(request, len);

		// Reply is n bytes long.
		//Therefore we set global variable that should be used to process the RX buffer to corresponding length.
		l4_thr_ExpectedReceiveBuffer = 6+REGISTER_LENGTH[access_register];
		l4_thr_counter =0;

		thrSendBytes(request, len);
		TYPE_OF_LAST_REQUEST = 0x03;
		LATEST_ACCESSED_REGISTER = access_register;
}




//////
void thr_wait(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1108;

	/*
	 * thr_wait is a "staging" function incorporated in THR_EXECUTION_SEQUENCE
	 * at end of previous stage - timestep of stage completion is recorded
	 *
	 * duration is required input from argv
	 *
	 * function will do nothing untill difference between current and previous timestamps is less then duration
	 *
	 * THR_EXECUTION_INDEX points to next action in the execution sequence stack.
	 * THR_EXECUTION INDEX is increased after current timestamp increased above designed wait duration
	 *
	 */
	uint16_t procedure_id = atoi(argv[0]); // procedure_id is always fist index of argument array
	uint32_t duration = atoi(argv[1]);
	uint32_t now_timestamp = (uint32_t)timGetSystime();


	char print_str[200];
	int len;

	if ( (now_timestamp - THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) < duration ){
		// do nothing
	}
	else{
		THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime(); //save execution finish time of wait stage
		sprintf(print_str, "\nWait Stage= %d Complete t = %d\n", THR_HARDCODED_SEQUENCES[procedure_id].execution_index,now_timestamp);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		THR_HARDCODED_SEQUENCES[procedure_id].execution_index++; // increase sequence execution index so that after wait - next module to be executed
	}

}

void thr_void(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1109;

	/*
	 * Incorporates with sequence execution
	 *
	 * void function that does nothing
	 */

	uint16_t procedure_id = atoi(argv[0]); // procedure_id is always fist index of argument array
	THR_HARDCODED_SEQUENCES[procedure_id].execution_index ++;

}

void thr_value_ramp(int argc, char *argv[]){
	//int substage_index =0;

	uint16_t procedure_id = atoi(argv[0]); // procedure_id is always fist index of argument array
	uint8_t register_index = atoi(argv[1]); // first argument is register index at which SET ramp would be implemented
	double goal = atof((const char*)argv[2]); // goal to which value should be set
	uint32_t ramp_duration = atoi(argv[3]); // time through which value should be changed from initial to goal
	uint32_t ramp_dt = atoi(argv[4]); // wait between SET
	double initial_value;
	uint32_t now_timestamp;

	char print_str[200];
	int len;

	double ramp_iterations = ramp_duration/ ramp_dt; // WARING - HARDCODE IN A WAY THAT THIS IS ALWAYS INT
	double value_step;

	char *temp_argv[3];
	//char argument1[1];
	char argument2[50];
	char argument3[50]; //7 //length of array should always be more then maximum characters for set value !!!!!



	switch (THR_HARDCODED_SEQUENCES[procedure_id].substage_index){

	case 0:// first we make read request to desired register in order to obtain initial register value
		GeneralReadRequest(argc,argv); // make sure that argv[1] is register index

		sprintf(print_str, "\nWInitial read Request Sent\n");
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		THR_HARDCODED_SEQUENCES[procedure_id].substage_index++;
		break;
	case 1: // wait for some time while proccess request is executing
		now_timestamp = (uint32_t)timGetSystime();
		if ( (now_timestamp - THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) <= 5000 ){
					// do nothing
			return;

				}
		else if ( (now_timestamp - THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) > 5000 ){
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime(); //save execution finish time of wait stage
			THR_HARDCODED_SEQUENCES[procedure_id].substage_index++;
			//sprintf(print_str, "\nWaiting to proccess initial READ reques\n");
			//len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);
			return;

		}
		break;
	case 2: // now we are sure that read request was proccessed - read initial value
		//initial_value = REGISTER_DATA[register_index];
		//sprintf(initial_value_string,"%.2f",REGISTER_DATA[register_index]);
		//THR_HARDCODED_SEQUENCES[procedure_id].sequences[THR_HARDCODED_SEQUENCES[procedure_id].execution_index].thr_argv[5] = initial_value_string;
		THR_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value = REGISTER_DATA[register_index];
		THR_HARDCODED_SEQUENCES[procedure_id].substage_index++;
		sprintf(print_str, "\nInitial value save value = %.2f\n",THR_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		break;
	case 3:

		initial_value  = THR_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value;
		value_step = (goal -initial_value)/ramp_iterations;

		if ((REGISTER_DATA[register_index] >= goal && value_step >0)    || (REGISTER_DATA[register_index] <= goal && value_step <0 )    ){
			// GOAL REACHED// EXIT FUNCTION
			THR_HARDCODED_SEQUENCES[procedure_id].execution_index ++;
			THR_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;

			temp_argv[0]= "7";

			sprintf(argument2, "%d",register_index);
			temp_argv[1]= argument2;

			sprintf(argument3, "%.2f",goal);
			temp_argv[2]= argument3;

			//sprintf(temp_argv[2], "%.2f",REGISTER_DATA[register_index]+value_step);
			GeneralSetRequest(3, temp_argv);

			sprintf(print_str, "\nOVER THE GOAL \n");
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
			return;
		}


		initial_value  = THR_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value; //

		sprintf(print_str, "\nRestore saved initial value = %.2f\n",initial_value );
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		value_step = (goal -initial_value)/ramp_iterations;

		sprintf(print_str, "\nSTEP = %.2f\n",value_step);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);


		temp_argv[0]= "7";

		sprintf(argument2, "%d",register_index);
		temp_argv[1]= argument2;

		sprintf(argument3, "%.2f",goal);
		temp_argv[2]= argument3;

		sprintf(argument3, "%.2f",REGISTER_DATA[register_index]+value_step);
		temp_argv[2]= argument3;

		//sprintf(temp_argv[2], "%.2f",REGISTER_DATA[register_index]+value_step); // this also works
		GeneralSetRequest(3, temp_argv);
		REGISTER_DATA[register_index] = REGISTER_DATA[register_index] +value_step;



		sprintf(print_str, "\nSET RAMP value = %.2f\n",REGISTER_DATA[register_index]);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		THR_HARDCODED_SEQUENCES[procedure_id].substage_index++;
		THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();

		break;
	case 4: // WAIT after RAMP set request done
		now_timestamp = (uint32_t)timGetSystime();
		if ( (now_timestamp - THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage) < 5000){
			// do nothing
		}
		else{
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime(); //save execution finish time of wait stage
			//substage_index++;
			THR_HARDCODED_SEQUENCES[procedure_id].substage_index++;
			sprintf(print_str, "\nWaiting after SET RAMP\n");
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
		}

		break;

	case 5:

		// EXIT CONDITIONS

		initial_value  = THR_HARDCODED_SEQUENCES[procedure_id].ramp_initial_value;
		value_step = (goal -initial_value)/ramp_iterations;

		if (value_step > 0){
			if (REGISTER_DATA[register_index] >= goal){
				// GOAL REACHED// EXIT FUNCTION
				THR_HARDCODED_SEQUENCES[procedure_id].execution_index ++;
				THR_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;

			}
			else{
			 	 // IF GOAL VALUE NOT YET REACHED JUMP BACK TO SET REQUEST
				THR_HARDCODED_SEQUENCES[procedure_id].substage_index=3;
				THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
			}
		}

		if(value_step <0){

			if (REGISTER_DATA[register_index] <= goal){
							// GOAL REACHED// EXIT FUNCTION
							THR_HARDCODED_SEQUENCES[procedure_id].execution_index ++;
							THR_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;

						}
						else{
						 	 // IF GOAL VALUE NOT YET REACHED JUMP BACK TO SET REQUEST
							THR_HARDCODED_SEQUENCES[procedure_id].substage_index=3;
							THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
						}



		}

		break;




	}







}

void thr_execute_sequence_cmd(int argc, char *argv[]){
	LAST_STARTED_MODULE = 1110;
	/*
	 * This function is meant to trigger execution of SEQUENCE
	 * Triggered function is thr_execute_sequence()

	*/
	uint8_t procedure_id = atoi(argv[1]);
	uint8_t action = atoi(argv[2]);
	char print_str[200];
	int len;

	switch(action){

	case 0: // start sequenece

		THR_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = true;
		THR_HARDCODED_SEQUENCES[procedure_id].execution_index = 0;
		THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_begin = (uint32_t)timGetSystime();
		THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
		THR_HARDCODED_SEQUENCES[procedure_id].substage_index = 0;
		sprintf(print_str, "\nSequence id=%d start\n",procedure_id);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		break;
	case 1: // restart sequence
		THR_HARDCODED_SEQUENCES[procedure_id].restart = true;
		sprintf(print_str, "\nSequence id=%d will now restart start\n",procedure_id);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		break;
	case 2: //repeat sequence
		THR_HARDCODED_SEQUENCES[procedure_id].repeat = true;
		sprintf(print_str, "\nSequence id=%d will now repeat\n",procedure_id);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		break;
	case 3: //cancel repeat sequence
			THR_HARDCODED_SEQUENCES[procedure_id].repeat = false;
			sprintf(print_str, "\nSequence id=%d stop repeat\n",procedure_id);
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
			break;
	case 4: //Pause sequence
			THR_HARDCODED_SEQUENCES[procedure_id].pause = true;
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
			sprintf(print_str, "\nSequence id=%d paused\n",procedure_id);
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
			break;
	case 5: //resume sequence
			THR_HARDCODED_SEQUENCES[procedure_id].pause = false;
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();
			sprintf(print_str, "\nSequence id=%d resumed\n",procedure_id);
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
			break;
	case 6: //Stop sequence
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = false;
			THR_HARDCODED_SEQUENCES[procedure_id].execution_index = 0;
			sprintf(print_str, "\nSequence id=%d manual stop\n",procedure_id);
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
			break;
	}

	//THR_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = true;
	//THR_HARDCODED_SEQUENCES[procedure_id].execution_index = 0;
	//THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_begin = (uint32_t)timGetSystime();
	//THR_HARDCODED_SEQUENCES[procedure_id].sequence_execution_stage = (uint32_t)timGetSystime();

	//sprintf(print_str, "\nSequence id=%d start\n",procedure_id);
	//len = strlen(print_str);
	//deb_print_pure_debug((uint8_t *)print_str, len);



	//*** THIS IS TEST EXAMPLE TO DEMONSTRATE THAT IT IS POSSIBLE TO CHANGE EXECUTION FUNCTION AND ARGUMENTS IN A SEQUENCE
	//char temp_arg[1];
	//sprintf(temp_arg, "%d",procedure_id);
	//THR_HARDCODED_SEQUENCES[procedure_id].sequences[0].thr_argv[0] = temp_arg;
	//THR_HARDCODED_SEQUENCES[procedure_id].sequences[0].thr_argv[1] = "20";
	//THR_HARDCODED_SEQUENCES[procedure_id].sequences[0].thr_argv[2] = "3000";
	//THR_HARDCODED_SEQUENCES[procedure_id].sequences[0].function = GeneralSetRequest_sequence;
	 //***********************************************************************




}



void thr_execute_sequence(int procedure_id){
	LAST_STARTED_MODULE = 1111;
	/*
	 *
	 * Executes pregrogrammed sequence  defined in THR_EXECUTION_SEQUENCE function pointer array
	 * THR_SEQUENCES array of argv to be input into THR_EXECUTION_SEQUENCE
	 */

	if(THR_HARDCODED_SEQUENCES[procedure_id].pause){
		// if sequence pause set to true
		// do nothing
		return;
		}

	if(THR_HARDCODED_SEQUENCES[procedure_id].restart){
					// Restart sequence
					THR_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = true;
					THR_HARDCODED_SEQUENCES[procedure_id].execution_index =0; // reset execution index to 0
					THR_HARDCODED_SEQUENCES[procedure_id].restart = false;
					return;
			}

	if (THR_HARDCODED_SEQUENCES[procedure_id].execution_index >= THR_HARDCODED_SEQUENCES[procedure_id].length){
		LAST_STARTED_MODULE = 11110; //DEBUG

		char print_str[200];
		sprintf(print_str, "\nSequence complete\n");
		int len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		if(THR_HARDCODED_SEQUENCES[procedure_id].repeat){
			sprintf(print_str, "\nRepeat sequence\n");
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = true;
			THR_HARDCODED_SEQUENCES[procedure_id].execution_index =0;
		}
		else{
			THR_HARDCODED_SEQUENCES[procedure_id].sequence_trigger = false;
			THR_HARDCODED_SEQUENCES[procedure_id].execution_index =0;
			return;

		}


	}

	else{
		void (*f)(int argc, char *argv[]);

		//f = THR_SEQUENCES[THR_EXECUTION_INDEX].function;
		//f(3,THR_SEQUENCES[THR_EXECUTION_INDEX].thr_argv);
		LAST_STARTED_MODULE = 10002; //DEBUG
		f = THR_HARDCODED_SEQUENCES[procedure_id].sequences[THR_HARDCODED_SEQUENCES[procedure_id].execution_index].function;
		LAST_STARTED_MODULE = THR_HARDCODED_SEQUENCES[procedure_id].execution_index; //DEBUG
		f(3,THR_HARDCODED_SEQUENCES[procedure_id].sequences[THR_HARDCODED_SEQUENCES[procedure_id].execution_index].thr_argv);
		LAST_STARTED_MODULE = 10004; //DEBUG
	}





}


//****************** THIS IS JUST TEST AND TRIAL********************************

void thr_write_mem(){
	LAST_STARTED_MODULE = 1112;
	uint8_t testdata[6];
	testdata[0] = 0x68;
	testdata[1] = 0x65;
	testdata[2] = 0x6C;
	testdata[3] = 0x6C;
	testdata[4] = 0x6F;
	testdata[5] = 0x0A;
	MramWriteAsync(0, 5000, (uint8_t*)&testdata, sizeof(testdata), thr_write_mem_callback);

}


void thr_write_mem_callback(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	LAST_STARTED_MODULE = 1113;

	if (result == MRAM_RES_SUCCESS) {
		char print_str[200];
		sprintf(print_str, "\nMemory Write Success\n");
		int len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		thr_read_mem();



	} else {
		// TODO:?? retry counter / timeouts ....
	}
}



void thr_read_mem(){
	LAST_STARTED_MODULE = 1114;
	MramReadAsync(0, 5000, MMRAM_READ_BUFFER, sizeof(MMRAM_READ_BUFFER), thr_read_mem_callback);
	//void MramReadAsync(uint8_t chipIdx, uint32_t adr,  uint8_t *rx_data,  uint32_t len,
	//void (*finishedHandler)(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len))
}

void thr_read_mem_callback(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	LAST_STARTED_MODULE = 1115;

	if (result == MRAM_RES_SUCCESS) {
		char print_str[200];
		sprintf(print_str, "\nMemory Read Success\n");
		int len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		len = strlen((char*)data);
		deb_print_pure_debug(data, len);

		sprintf(print_str, "\n-----msg_end--------\n");
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);



	} else {
		// TODO:?? retry counter / timeouts ....
	}
}


void mem_write_cmd(int argc, char *argv[]){

	thr_write_mem(); //debug test

}
// ******************************************************************************
