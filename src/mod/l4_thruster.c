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

	char *l4_args[2];
	l4_args[0] = "45"; // temperature to which we set heater
	//SetReservoirTemperature(1, l4_args);


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
	double input;
	sscanf(argv[0], "%lf", &input);

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
	l4_thr_ExpectedReceiveBuffer = 6;// change expected receive buffer accordingly

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
