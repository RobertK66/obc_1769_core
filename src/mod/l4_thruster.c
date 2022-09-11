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



// THRUSTER FIRING SEQUENCE OPERATION REGISTORS
bool THRUSTER_FIRING_STATUS;
uint32_t THR_FIRE_DURATION;
uint32_t THR_FIRE_START_TIMESTAMP;
uint16_t THR_FIRE_SI;
uint16_t THR_FIRE_THRUST;
bool THRUSTER_FIRE_FIRST_TIME;
uint32_t THR_EXECUTION_TIMESTAMP;

bool THR_SEQUENCE_TRIGGER;
uint16_t THR_EXECUTION_INDEX;
uint32_t THR_SEQUENCE_EXECUTION_BEGIN;
uint32_t THR_SEQUENCE_EXECUTION_STAGE;
uint16_t THR_SEQUENCE_LENGTH;

void thr_wait(int argc, char *argv[]);
void GeneralSetRequest_sequence(int argc, char *argv[]);
void thr_execute_sequence();
void thr_void(int argc, char *argv[]);
//void thr_execute_sequence_cmd(int argc, char *argv[]);

#define MAX_EXECUTION_SEQUENCE_DEPTH 50 // Maximum size of execution sequence stack

typedef struct {
	char *thr_argv[3];
} thr_argv_array_t;
thr_argv_array_t THR_ARGV_SEQUENCE[MAX_EXECUTION_SEQUENCE_DEPTH]; // array of argv for sequence execution stack

// Array of function pointers
void (*THR_EXECUTION_SEQUENCE[MAX_EXECUTION_SEQUENCE_DEPTH])(int argc, char *argv[]); // sequence execution stack (array of function pointers)



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


	// Initialize Thruster Firing sequence operational registers
	THRUSTER_FIRING_STATUS = false;
	THR_FIRE_DURATION= 0;
	THR_FIRE_START_TIMESTAMP =0;
	THRUSTER_FIRE_FIRST_TIME = true;
	THR_EXECUTION_INDEX=0;



	/// **************** PREPROGRAMM SEQUENCES HERE ****************
	THR_SEQUENCE_TRIGGER = false;

	THR_EXECUTION_SEQUENCE[0] = GeneralSetRequest_sequence;
	THR_EXECUTION_SEQUENCE[1] = thr_wait;
	THR_EXECUTION_SEQUENCE[2] = GeneralSetRequest_sequence;
	THR_EXECUTION_SEQUENCE[3] = thr_void; // ALWAYS FINISH SEQUENCE WITH VOID FUNCTION

	THR_SEQUENCE_LENGTH = 3; // MANUALLY DEFINE LENGTH OF SEQUENCE // NOTE : SEQUENCE LENGTH IS MAXIMUM INDEX OF THR_EXECUTION_SEQUENCE ARRAY





	//Build argv array for execution sequence
	// Wait 500 ms
	THR_ARGV_SEQUENCE[0].thr_argv[0]= "7";
	THR_ARGV_SEQUENCE[0].thr_argv[1]= "20";
	THR_ARGV_SEQUENCE[0].thr_argv[2]= "1500";


	// wait
	THR_ARGV_SEQUENCE[1].thr_argv[0]= "1000";
	THR_ARGV_SEQUENCE[1].thr_argv[1]= "0";
	THR_ARGV_SEQUENCE[1].thr_argv[2]= "0";

	// wait
	THR_ARGV_SEQUENCE[2].thr_argv[0]= "7";
	THR_ARGV_SEQUENCE[2].thr_argv[1]= "20";
	THR_ARGV_SEQUENCE[2].thr_argv[2]= "1500";



}

void l4_thruster_main (void) {

	// TODO : I am not sure what to do here.
	// Leaving it for now so that l4_thruster_module can be
	// Initialised according as all other modules


	// this way we obtain time ms time ticks
	//obc_systime32_t timestamp= 	timGetSystime();
	//char print_str[20];
	//sprintf(print_str, "t = %d \n", timestamp);
	//uint8_t len = strlen(print_str);
	//deb_print_pure_debug((uint8_t *)print_str, len);

	if (THRUSTER_FIRING_STATUS){
		thr_fire_exe();

	}

	if (THR_SEQUENCE_TRIGGER){
		thr_execute_sequence();

	}



}






void l4_debugPrintBuffer(uint8_t *buffer,int bufferlen){
	//LPC_UART2 is debug UART
	for (int i=0;i<bufferlen;i++){
		Chip_UART_SendByte(LPC_UART2, buffer[i]);
	}

}



/////  Note for pull request - I deleted all manual functions Example: SetHeaterCurrent()
// Because all of them can be implemented with functions for general requests



void ThrSendVersionRequestCmd(int argc, char *argv[]){

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
	GeneralSetRequest(argc,argv);

	char print_str[200];
	sprintf(print_str, "\nStage index= %d completed\n",THR_EXECUTION_INDEX);
	int len = strlen(print_str);

	deb_print_pure_debug((uint8_t *)print_str, len);
	THR_EXECUTION_INDEX++;
	THR_SEQUENCE_EXECUTION_STAGE = (uint32_t)timGetSystime(); // Save timestamp at which sequence stage finished

}


void GeneralSetRequest(int argc, char *argv[]){



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


//General read request to any register
void GeneralReadRequest(int argc, char *argv[]){

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


void thr_fire_exe(){
	// thruster fire sequence
	uint32_t now_timestamp = (uint32_t)timGetSystime();

	// for debug prints
	char print_str[200];
	uint8_t len;

	// for thruster requests
	char *requests_argv[3];

	// for function internal variables
	double reservoir_temp;


	if(THRUSTER_FIRE_FIRST_TIME){

		THR_EXECUTION_TIMESTAMP = (uint32_t)timGetSystime(); // First time now_timestamp = execution timestamp

		// TODO implement start fire procedure
		sprintf(print_str, "\nFIRE BEGIN  t = %d\n", THR_FIRE_START_TIMESTAMP);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);



		// TODO : 1) Check if THRUSTER MODE = 1 ( temp reached above 179K)
		requests_argv[0] ="6"; // read cmd //not neccesary
		requests_argv[1]="99"; // reservoir temperature register
		GeneralReadRequest(2, requests_argv); // Request sent
		// TODO WAIT UNTILL REQUEST BYTES ARE SENT, WAIT UNTILL REPLY RECEIVED
		// TODO CHECK IF REPLY IS MOST RECENT
		reservoir_temp = REGISTER_DATA[99];
		sprintf(print_str, "\nReservoir  T = %fK\n", reservoir_temp);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		if(reservoir_temp < 160){
			// Do not continue with thrust ! Not yet hot enough reservoir
			//THRUSTER_FIRING_STATUS = false; // this will prevent further execution
			sprintf(print_str, "\nRESERVOIR NOT YET READY\n", reservoir_temp);
			len = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len);
		}

		/*
		if ( (THR_EXECUTION_TIMESTAMP - now_timestamp) <= 10 ){
			THR_EXECUTION_TIMESTAMP = (uint32_t)timGetSystime();
			return;
		}
		*/

		//Set extractor mode 0
		requests_argv[0]="7";
		requests_argv[1]="45";
		requests_argv[2]="0";
		//GeneralSetRequest(3, requests_argv);

		//Set emitter mode 0
		requests_argv[0]="7";
		requests_argv[1]="30";
		requests_argv[2]="0";
		//GeneralSetRequest(3, requests_argv);

		//Set extractor mode 0
		requests_argv[0]="7";
		requests_argv[1]="75";
		requests_argv[2]="0";
		//GeneralSetRequest(3, requests_argv);


		THRUSTER_FIRE_FIRST_TIME = false; // set to false to prevent further execution of initialization block
	}

	if(THRUSTER_FIRING_STATUS){

		if ( (now_timestamp - THR_FIRE_START_TIMESTAMP) <= THR_FIRE_DURATION ){

			//do nothing // or implement monitoring procedure

		}

		else{
			// TODO Implement stop fire procedure
			sprintf(print_str, "\nSTOP t = %d\n", now_timestamp);
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);

			THRUSTER_FIRING_STATUS = false;
			//THR_FIRE_DURATION = 0;
		}

	}






}


void thr_fire_cmd(int argc, char *argv[]){
	// This function will triger execution of thr_fire_exe()
	// inside l4_main()
	// until stopping condition is reached.
	// Stopping condition is defined in thr_fire_exe()

	// THIS FUNCTION IS MENT TO BE JUST A TRIGGER THAT LAUNCHES EXECUTION FUNCTION

	THRUSTER_FIRING_STATUS = true;
	THRUSTER_FIRE_FIRST_TIME = true;
	THR_FIRE_DURATION = atoi(argv[1]);
	THR_FIRE_SI = atoi(argv[2]); // SI [s]
	THR_FIRE_THRUST = atoi(argv[2]); // thrust [microN]
	THR_FIRE_START_TIMESTAMP = (uint32_t)timGetSystime();

	THR_SEQUENCE_EXECUTION_BEGIN = (uint32_t)timGetSystime();
	THR_SEQUENCE_EXECUTION_STAGE = (uint32_t)timGetSystime();


}

//////
void thr_wait(int argc, char *argv[]){

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
	uint32_t duration = atoi(argv[0]);
	uint32_t now_timestamp = (uint32_t)timGetSystime();

	char print_str[200];
	int len;

	if ( (now_timestamp - THR_SEQUENCE_EXECUTION_STAGE) < duration ){
		// do nothing
	}
	else{
		THR_SEQUENCE_EXECUTION_STAGE = (uint32_t)timGetSystime(); //save execution finish time of wait stage
		sprintf(print_str, "\nWait Stage= %d Complete t = %d\n", THR_EXECUTION_INDEX,now_timestamp);
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		THR_EXECUTION_INDEX++; // increase sequence execution index so that after wait - next module to be executed
	}

}

void thr_void(int argc, char *argv[]){

	/*
	 * Incorporates with sequence execution
	 *
	 * void function that does nothing
	 */

	//THR_EXECUTION_INDEX++;

}


void thr_execute_sequence_cmd(int argc, char *argv[]){
	/*
	 * This function is meant to trigger execution of SEQUENCE
	 * Triggered function is thr_execute_sequence()

	*/


	THR_SEQUENCE_TRIGGER = true;
	THR_SEQUENCE_EXECUTION_BEGIN = (uint32_t)timGetSystime();
	THR_SEQUENCE_EXECUTION_STAGE = (uint32_t)timGetSystime();

	char print_str[200];
	sprintf(print_str, "\nSequence start\n");
	int len = strlen(print_str);
	deb_print_pure_debug((uint8_t *)print_str, len);


}



void thr_execute_sequence(){
	/*
	 *
	 * Executes pregrogrammed sequence  defined in THR_EXECUTION_SEQUENCE function pointer array
	 * THR_ARGV_SEQUENCE array of argv to be input into THR_EXECUTION_SEQUENCE
	 */

	if (THR_EXECUTION_INDEX == THR_SEQUENCE_LENGTH){
		char print_str[200];
		sprintf(print_str, "\nSequence complete\n");
		int len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);
		THR_SEQUENCE_TRIGGER = false;
		THR_EXECUTION_INDEX =0;
		return;
	}
	else{
		void (*f)(int argc, char *argv[]);
		f = THR_EXECUTION_SEQUENCE[THR_EXECUTION_INDEX];
		f(3,THR_ARGV_SEQUENCE[THR_EXECUTION_INDEX].thr_argv);
	}





}
