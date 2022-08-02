/*
===============================================================================
 Name        : l4_thruster.c
 Author      : Jevgeni
 Created on	 : 30.06.2022

 Higher level logic layer to controll thruster
===============================================================================
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>



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

#include "i2c_arduino/i2c_arduino.h"


#include "ai2c/obc_i2c.h"
#include "ai2c/obc_i2c_rb.h"
#include "ai2c/obc_i2c_int.h"


#include "crc/obc_checksums.h"






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





const uint8_t REGISTER_VALUES[108] = {0,1,2,3,4,5,6,7,8,9,10,
		11,12,13,14,15,16,17,18,19,20,
		21,22,23,24,25,26,27,28,29,30,
		31,32,33,34,35,36,37,38,39,40,
		41,42,43,44,45,46,48,48,49,50,
		51,52,53,54,55,56,57,58,59,60,
		61,62,63,64,65,66,67,68,69,70,
		71,72,73,74,75,76,77,78,79,80,
		81,82,83,84,85,86,87,88,89,90,
		91,92,93,94,95,96,97,98,99,100,
		101,102,103,104,105,106,107
		};



////////////////////NEW

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
//about this register in EMPULSION doccumentation !!!
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






///// NEW END



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

	char* testvar = "hello thruster";
	SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_STRING, testvar, strlen(testvar));//


	// test run of function

	//char *l4_args[2];
	//l4_args[0] = "45"; // temperature to which we set heater
	//l4_args[1] = "45.1"; // temperature to which we set heater
	//SetReservoirTemperature(1, l4_args);
	//GeneralSetRequest(1, l4_args); // HERE ERROR IN GENERAL SET REQUEST



}

void l4_thruster_main (void) {





}






void l4_debugPrintBuffer(uint8_t *buffer,int bufferlen){

	//LPC_UART2 is debug UART

	for (int i=0;i<bufferlen;i++){
		Chip_UART_SendByte(LPC_UART2, buffer[i]);

	}

}



/////  BELOW WOULD BE FUNCTIONS TO CONTROLL THRUSTER




void SetReservoirTemperature(int argc, char *argv[]){


	uint16_t reff_t = atoi(argv[0]);
	//printf(" REF TEMP SET %d \n",reff_t);

	// APPLY CONVERSION MULTIPLIER CONCEPT
	//uint16_t conversion_mult = 100;
	//reff_t = reff_t*conversion_mult;
	reff_t = reff_t*CONVERSION[97];

	//printf(" CONVERSION  =  %d \n",CONVERSION[97]);

	uint8_t request[9];
	request[0] = SENDER_ADRESS;
	request[1] = DEVICE;
	request[2] = MSGTYPE[3]; // WRITE -3
	//checksumm
	request[3] = 0x00;



	// payload length two bytes 03 00 | two bytes for
	// payload length = register map + value
	request[4] = 0x03; // LENGTH of payload HARDCODED (3)
	request[5] = 0x00; //hardcoded

	request[6]= REGISTER_VALUES[97]; // RESEIRVOUR TEMPERATURE - index 97



	// CONVERT uint16_t value into array of two uint8_t bytes
	uint8_t conversionArray[2];
	conversionArray[0] = reff_t & 0xff;
	conversionArray[1] = (reff_t >> 8) & 0xff;

	request[7]= conversionArray[0];
	request[8] = conversionArray[1];




	int len = sizeof(request);


	request[3] = CRC8(request,len);

	thrSendBytes(request, len);


}




void SetHeaterMode(int argc, char *argv[]){

	//  HEATER MODE 1 or 0
	uint16_t heater_mode = atoi(argv[0]);

	if (heater_mode ==1 || heater_mode ==0){

		// APPLY CONVERSION MULTIPLIER CONCEPT
		//uint16_t conversion_mult = 1;
		//heater_mode = heater_mode*conversion_mult;
		heater_mode = heater_mode * CONVERSION[60];
		//printf(" CONVERSION  =  %d \n",CONVERSION[60]);

		uint8_t request[8];
		request[0] = SENDER_ADRESS;
		request[1] = DEVICE;
		request[2] = MSGTYPE[3]; // WRITE -3
		request[3] = 0x00; //byte for checksum initially set to 0
		request[4] = 0x02; // LENGTH of payload HARDCODED (2)       || payload length = register map + value
		request[5] = 0x00; //hardcoded
		request[6]= REGISTER_VALUES[60]; // HEATER MODE - 60
		request[7]= heater_mode;

		int len = sizeof(request);
		request[3] = CRC8(request,len); // after actual checksum is calculated = byte is filled

		l4_thr_ExpectedReceiveBuffer = 10; // BEFORE SENDING BYTES - CHANGE EXPECTED RECEIVE BUFFER LENGTH
		l4_thr_counter =0; //every request function should reset received bytes counter !!!!

		thrSendBytes(request, len);

	}
	else {
		//printf("HEATER MODE WRONG INPUT %d \n",heater_mode);

		return;
	}


}






void SetHeaterVoltage(int argc, char *argv[]){


	uint16_t voltage = atoi(argv[0]);
	//printf(" SET VOLTAGE  =  %d \n",voltage);
	// APPLY CONVERSION MULTIPLIER CONCEPT
	//uint16_t conversion_mult = 1000;
	//voltage = voltage*conversion_mult;
	voltage = voltage * CONVERSION[61];
	//printf(" CONVERSION  =  %d \n",CONVERSION[61]);

	uint8_t request[9];
	request[0] = SENDER_ADRESS;
	request[1] = DEVICE;
	request[2] = MSGTYPE[3]; // WRITE -3
	request[3] = 0x00; //checksumm
	request[4] = 0x03; // LENGTH of payload HARDCODED (3)
	request[5] = 0x00; //hardcoded
	request[6]= REGISTER_VALUES[61]; // RESEIRVOUR TEMPERATURE - index 97



	// CONVERT uint16_t value into array of two uint8_t bytes
	//uint8_t conversionArray[2];
	//conversionArray[0] = voltage & 0xff;
	//conversionArray[1] = (voltage >> 8) & 0xff;

	request[7]= voltage & 0xff;
	request[8] = (voltage >> 8) & 0xff;

	int len = sizeof(request);


	request[3] = CRC8(request,len);

	thrSendBytes(request, len);


}






void SetHeaterCurrent(int argc, char *argv[]){
	// INPUT RANGE 0-3[A]  !!!!

	// PARSE DOUBLE FROM ARGV
	double input =2.5;
	//sscanf(argv[0], "%lf", &input);  // WARNING sscanf does not work with OBC !!!!!

	//printf(" \n ARGV  =  %s \n",argv[0]);
	//printf(" \n ORIGINAL INPUT  =  %f \n",input);
	//double conversion_factor = (double)CONVERSION[65];

	//APPLY CONVERSION MULTIPLIER
	input = input * (double)CONVERSION[65];
	//input = input / 0.0001;
	//printf(" \n CONVERTED INPUT  =  %f \n",input);
	//printf(" \n MULTIPLIER  =  %f \n",(double)CONVERSION[65]);


	// CONVERT INPUT INTO UINT16
	uint16_t value = (uint16_t) input;
	//printf("\n SET CURRENT  =  %d \n",value);


	uint8_t request[9];
	request[0] = SENDER_ADRESS;
	request[1] = DEVICE;
	request[2] = MSGTYPE[3]; // WRITE -3
	request[3] = 0x00; //checksumm
	request[4] = 0x03; // LENGTH of payload HARDCODED (3)
	request[5] = 0x00; //hardcoded
	request[6]= REGISTER_VALUES[65]; // RESEIRVOUR TEMPERATURE - index 65
	request[7]= value & 0xff;
	request[8] = (value >> 8) & 0xff;

	int len = sizeof(request);


	request[3] = CRC8(request,len);


	l4_thr_ExpectedReceiveBuffer = 7;
	l4_thr_counter =0;

	thrSendBytes(request, len);


}


void SetHeaterPower(int argc, char *argv[]){


	uint16_t value = atoi(argv[0]);
	//printf(" SET POWER  =  %d \n",value);
	// APPLY CONVERSION MULTIPLIER CONCEPT
	//uint16_t conversion_mult = 1000;
	//uint16_t conversion_mult = CONVERSION[69];
	value = value * CONVERSION[69];

	//printf(" CONVERSION  =  %d \n",CONVERSION[69]);

	uint8_t request[9];
	request[0] = SENDER_ADRESS;
	request[1] = DEVICE;
	request[2] = MSGTYPE[3]; // WRITE -3
	request[3] = 0x00; //checksumm
	request[4] = 0x03; // LENGTH of payload HARDCODED (3)
	request[5] = 0x00; //hardcoded
	request[6]= REGISTER_VALUES[69]; // RESEIRVOUR TEMPERATURE - index 69
	request[7]= value & 0xff;
	request[8] = (value >> 8) & 0xff;

	int len = sizeof(request);


	request[3] = CRC8(request,len);

	l4_thr_ExpectedReceiveBuffer = 7;
	l4_thr_counter =0;

	thrSendBytes(request, len);


}





void ReadHeaterCurrent(int argc, char *argv[]){

		uint8_t request[8];
		request[0] = SENDER_ADRESS;
		request[1] = DEVICE;
		request[2] = MSGTYPE[2]; // READ -3
		request[3] = 0x00; //checksumm
		request[4] = 0x02; // LENGTH of payload REGISTER and length
		request[5] = 0x00; //hardcoded
		request[6]= REGISTER_VALUES[67]; // 67 - hex 0x43 corresponds to read heater current register
		request[7]= 2;// number of bytes to read
		//request[8] = (value >> 8) & 0xff;

		int len = sizeof(request);


		request[3] = CRC8(request,len);

		// we know that reply is 7 bytes long. Therefore we set global variable that should be used to process the RX buffer to coresponding length.
		l4_thr_ExpectedReceiveBuffer = 7;
		l4_thr_counter =0;

		thrSendBytes(request, len);



}





void ThrSendVersionRequestCmd(int argc, char *argv[]){


	uint8_t request[8];
	/*
	request[0]= 0x00;
	request[1]= 0xFF;
	request[2]= 0x03;
	request[3]= 0x14;
	request[4]= 0x02;
	request[5]= 0x00;
	request[6]= 0x00;
	request[7]= 0x01;
    */

	request[0]= 0x00;
	request[1]= 0xFF;
	request[2]= 0x03;
	request[3]= 0x14;
	request[4]= 0x02;
	request[5]= 0x00;
	request[6]= 0x00;
	request[7]= 0x01;

	int len = sizeof(request);

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
	l4_thr_ExpectedReceiveBuffer = 50;
	l4_thr_counter =0;

	thrSendBytes(request, len);




}


////////////////// NEW MODULES


// received buffer of arbitrary size
// len length of actual thruster reply
void ParseReadRequest(uint8_t* received_buffer,int len){

	uint8_t sender = received_buffer[0];
	uint8_t receiver = received_buffer[1];
	uint8_t message_type = received_buffer[2];
	uint8_t received_checksum = received_buffer[3];
	uint8_t payload_length_1 = received_buffer[4];
	uint8_t payload_length_2 = received_buffer[5];

	// combine payload length bytes into uint16_t

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

			//printf("\n i=%d",i);
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






void GeneralSetRequest(int argc, char *argv[]){



	uint8_t len; // will cary total length of request array

	// FIRST ARGUMENT SHOUD BE int VALUE OF REGISTER THAT WOULD BE READ FROM
	uint8_t access_register = atoi(argv[0]);
	//TYPE_OF_LAST_REQUEST = 4;

	uint8_t length_of_register = REGISTER_LENGTH[access_register];

	//Check if valid/existing register is accessed

	if(length_of_register ==0){
		return; //as previously discussed - 0 entry in length array means no info
				//therefore not proceed with that request
	}


	////  ------------ REQUEST ARRAY INITIALIZATION  ---------------
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


	///////  ------------------ MESSAGE HEADER  ------------------------------------
	request[0] = SENDER_ADRESS;
	request[1] = DEVICE;
	request[2] = MSGTYPE[3]; // WRITE -3
	request[3] = 0x00; //checksumm


	////// -------------------  PAYLOAD LENGTH --------------
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




	/// -------------------    ----------------------

	request[6]= REGISTER_VALUES[access_register]; // address of register that intended to be set



	/// ----- PARSE ARGUMENTS  and input transformation -----------

	if (length_of_register ==1){
		// if register is only 1 byte

		//parse input as uint8_t
		uint8_t input_uint8 = atoi(argv[1]);

		//transform input using conversion multipliers array.
		// at all registers with only 1 byte data length conversion multiplier array
		// will also be an integer value. Therefore typecast of conversion multiplier
		//into uint8_t should also work
		input_uint8 = input_uint8* (uint8_t) CONVERSION_DOUBLE[access_register];

		request[7]= input_uint8;

	}

	/////////////////////////////////////////here in this block error
	if (length_of_register ==2){

		// parse argument as double
		double input = atof((const char*)argv[1]); // WARNING :::: ATOF DOES NOT WORK
		// in cases where data stored in register has length of 2 bytes. uint16_t would be used
		// to store this data in register. For some registers input may be float
		// example 3.3V  or 3.0 or 3
		// Function should be able to handle both float and integer inputs from user


		//APPLY CONVERSION MULTIPLIER
		input = input* CONVERSION_DOUBLE[access_register];
		// CONVERT INPUT INTO UINT16
		uint16_t value = (uint16_t) input;


		// represent uint16_t as two uint8_t bytes to fill the request array.
		request[7]= value & 0xff;
		request[8] = (value >> 8) & 0xff;

	}
	//////////////////////



	request[3] = CRC8(request,len); // calculate checksum after whole request array is sent
	l4_thr_ExpectedReceiveBuffer = 6;// change expected receive buffer accordingly

	thrSendBytes(request, len);
    //printf("%s",request);




}


/////// GENERAL READ REQUEST TO ANY REGISTER
void GeneralReadRequest(int argc, char *argv[]){

		// FIRST ARGUMENT SHOUD BE int VALUE OF REGISTER THAT WOULD BE READ FROM
		uint8_t access_register = atoi(argv[0]);

		uint8_t length_of_register = REGISTER_LENGTH[access_register];

		//Check if valid/existing register is accessed

		if(length_of_register ==0){
			return; //as previously discussed - 0 entry in length array means no info
			//therefore not proceed with that request
		}


		uint8_t request[8];
		request[0] = SENDER_ADRESS;
		request[1] = DEVICE;
		request[2] = MSGTYPE[2]; // READ -3
		request[3] = 0x00; //checksumm
		request[4] = 0x02; // LENGTH of payload REGISTER and length
		request[5] = 0x00; //hardcoded because length of payload is defined with two bytes of uint16
		request[6]= REGISTER_VALUES[access_register]; // Access register at address requested by used

        //printf("\n register value is set to hex %02X  int %d\n",request[6],request[6]);
		request[7]= length_of_register;

		int len = sizeof(request);


		request[3] = CRC8(request,len);

		// we know that reply is n bytes long. Therefore we set global variable that should be used to process the RX buffer to coresponding length.
		l4_thr_ExpectedReceiveBuffer = 6+REGISTER_LENGTH[access_register];
		l4_thr_counter =0;

		thrSendBytes(request, len);
		TYPE_OF_LAST_REQUEST = 0x03;
        //printf("%s",request);


}


