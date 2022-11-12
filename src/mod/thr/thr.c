/*
 * based on climb_gps.c
 *
 *  Created on: 27.02.2022
 *      Copy paste by: Jevgeni
 */
#include "thr.h"
#include <ado_uart.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "../l7_climb_app.h"
#include "../l2_debug_com.h"
//#include "../l4_thruster.h" // with that we include variable  l4_thr_ExpectedReceiveBuffer  which defines expected RX buffer length
#include "../modules_globals.h"
#include <ado_crc.h>
#include "../tim/obc_time.h"
//uint16_t LAST_STARTED_MODULE;

thr_register_data_t THR_REGISTER_DATA;

int l4_thr_ExpectedReceiveBuffer = 7;  // this is important variable that defines NEXT expected thruster reply buffer size.
// Every request function has defined known length of reply.  This will take lot of time to investigate how many bytes are replied by each request.
// Probably I will make an array of functions requests, and array of expected reply length requests.


int l4_thr_counter = 0; // counter for received bytes

// Conversion multipliers are factors used to transform input variable to an uint16_t value needed to be stored/read in/from THRUSTER REGISTER MAP.

// This is long monkey work to input the CONVERSION array with right factor values.  How its done described in Jevgenis report from 2021.
// You need to  use python feep tools to investigate the correct conversion value.

// We assume that Byte request to thruster that are send by "PYTHON FEEP TOOLS" are the ground truth.
// Use it to verify that byte request sent by OBC is in accordance with output from Python Feep Tools

// Its gonna be a loooong and boooring work to fill in this array......

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
const double CONVERSION_DOUBLE[108] = {1.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0,
									1.0,0.0, 0.0, 0.0, 1.0, 1.0, 10000000.0, 0.0, 10000000.0, 0.0,
									1.0, 0.0, 1.0, 0.0, 1000.0, 0.0,1000.0, 0.0, 10000.0, 0.0,
									1.0, 1.0, 0.0, 1.0, 0.0, 10000000.0, 0.0, 10000000.0, 0.0, 1000.0,
									0.0, 1000.0, 0.0, 2.55, 2.55, 1.0, 1.0, 0.0, 1.0, 0.0,
									100000000.0, 0.0, 100000000.0, 0.0, 10000.0, 0.0, 10000.0, 0.0, 2.55,
									2.55, 1.0, 1000.0, 0.0, 1000.0, 0.0, 10000.0, 0.0, 10000.0, 0.0,
									1000.0, 0.0, 1000.0, 0.0, 2.55, 2.55, 1.0, 1.0, 1.0, 1.0,
									1.0, 100.0, 0.0, 10000.0, 0.0, 10000.0, 0.0, 1000.0, 0.0,1000.0,
									0.0, 2.55, 2.55, 10000000.0, 0.0, 10000000.0, 0.0, 1.0, 100.0, 0.0,
									100.0, 0.0, 100.0, 0.0, 100.0, 0.0, 1000.0, 0.0, 1.0};



// REPRESENT LENGTH OF REGISTER ENTRY at corresponding register address
// NOTE IMPORTANT - REGISTER LENGTH CANNOT BE 0.  0 in the array means that there are no information
// avaliable about that register inside EMPULSION documentation !!!!!! 0 is missing info !!!!
const uint8_t REGISTER_LENGTH[108] = {1, 1, 2, 0, 2, 0, 4, 0, 0, 0,
									4, 0, 0, 0, 1, 1, 2, 0, 2, 0,
									2, 0, 2, 0, 2, 0, 2, 0, 2, 0,
									1, 2, 0, 2, 0, 2, 0, 2, 0, 2,
									0, 2, 0, 1, 1, 1, 2, 0, 2, 0,
									2, 0, 2, 0, 2, 0, 2, 0, 1, 1,
									1, 2, 0, 2, 0, 2, 0, 2, 0, 2,
									0, 2, 0, 1, 1, 1, 1, 1, 1, 1,
									2, 0, 2, 0, 2, 0, 2, 0, 2, 0,
									1, 1, 2, 0, 2, 0, 1, 2, 0, 2,
									0, 2, 0, 2, 0, 2, 0, 2};

const uint8_t SENDER_ADRESS = 0x00;
const uint8_t DEVICE = 0xff;


const uint8_t MSGTYPE[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
// 0 - ERROR
// 1 - OK
// 2- READ
// 3 - WRITE
// 4 - DATA
// 5 - RESET
//6 - UPDATE
//7  -CONFIG





// prototypes
void thrUartIRQ(LPC_USART_T *pUART);
void thrProcessRxByte(uint8_t rxByte);
void thr_debugPrintBuffer(uint8_t *buffer,int bufferlen);

static thr_initdata_t *thrInitData;

// ************************** TX Circular byte Buffer Begin ********************
#define THR_TX_BUFFERSIZE	200
static uint8_t 		   thrTxBuffer[THR_TX_BUFFERSIZE];
static uint8_t         thrTxWriteIdx = 0;
static uint8_t         thrTxReadIdx  = 0;
static bool		  	   thrTxBufferFull = false;
static bool				thrFirstByteAfterReset=true;


#define THR_BUFFERLEN 700 // maximum buffer length
//int thr_counter = 0; would be defined in l4_thruster
char thr_receiveBuffer[THR_BUFFERLEN] ="";
//l4_thr_counter = 0; // set received bytes counter to 0 initialy // defined in l4_thruster.c


static bool inline thrTxBufferEmpty() {
	if (thrTxBufferFull) {
		return false;
	} else {
		return (thrTxReadIdx == thrTxWriteIdx);
	}
}

static bool inline thrTxAddByte(uint8_t b) {
	if (!thrTxBufferFull) {
		thrTxBuffer[thrTxWriteIdx++] = b;
		if (thrTxWriteIdx >= THR_TX_BUFFERSIZE) {
			thrTxWriteIdx = 0;
		}
		if (thrTxWriteIdx == thrTxReadIdx) {
			thrTxBufferFull = true;
		}
		return true;
	} else {
		return false;
	}
}

static uint8_t inline thrTxGetByte(void) {
	uint8_t retVal = thrTxBuffer[thrTxReadIdx++];
	if (thrTxReadIdx >= THR_TX_BUFFERSIZE) {
		thrTxReadIdx = 0;
	}
	thrTxBufferFull = false;
	return retVal;
}

// ************************** TX Circular byte Buffer End ********************


void thrInit (void *initData) {
	thrInitData = (thr_initdata_t*) initData;

	// Switch the 'enable' pin to low (no internal Volage regulator needed
	//Chip_GPIO_SetPinOutLow(LPC_GPIO, thrInitData->pEnablePin->pingrp, thrInitData->pEnablePin->pinnum);
	//// IMPORTANT PINIDX_RS485_TX_RX   HIGH - TRANSMIT              LOW - RECEIVE
	// SET TO RECEIVE UPON INITIALIZATION
	//Chip_GPIO_SetPinOutLow(LPC_GPIO, 2, 5); //  This is PINIDX_RS485_TX_RX RECEIVE
	//Chip_GPIO_SetPinOutHigh(LPC_GPIO, 2, 5); //  This is PINIDX_RS485_TX_RX TRANSMIT

	// Init UART
	InitUart(thrInitData->pUart, 115200, thrUartIRQ);
	thrFirstByteAfterReset = true;

	// Enable dirrection controll
	thrInitData->pUart->RS485CTRL |= UART_RS485CTRL_DCTRL_EN; // Enable Auto Direction Control

	// If direction control is enabled (bit DCTRL = 1), pin DTR is used for direction control
	thrInitData->pUart->RS485CTRL |= UART_RS485CTRL_SEL_DTR;

	//This bit reverses the polarity of the direction control signal on the RTS (or DTR) pin. The direction control pin
	 //will be driven to logic "1" when the transmitter has data to be sent
	thrInitData->pUart->RS485CTRL |= UART_RS485CTRL_OINV_1;



}

void thrMain (void) {
	LAST_STARTED_MODULE=10;
	// Uart Rx
	int32_t stat = Chip_UART_ReadLineStatus(thrInitData->pUart);
	if (stat & UART_LSR_RDR) {
		// there is a byte available. Lets read and process it.


		uint8_t b = Chip_UART_ReadByte(thrInitData->pUart);
		thrProcessRxByte(b);
	}
}

void thrUartIRQ(LPC_USART_T *pUART) {
	LAST_STARTED_MODULE=1001;
	if (thrInitData->pUart->IER & UART_IER_THREINT) {
		// Transmit register is empty now (byte was sent out)
		if (thrTxBufferEmpty() == false) {
			// Send next byte
			uint8_t nextByte = thrTxGetByte();
			Chip_UART_SendByte(thrInitData->pUart, nextByte);
		} else {
			// No more bytes available -> stop the THRE IRQ.
			Chip_UART_IntDisable(thrInitData->pUart, UART_IER_THREINT);

			// switch back to receive when all bytes are transmited
			Chip_GPIO_SetPinOutLow(LPC_GPIO, 2, 5); //  This is PINIDX_RS485_TX_RX RECEIVE
		}
	}
}

void thrSendByte(uint8_t b) {
	LAST_STARTED_MODULE=1002;

	// block irq while handling tx buffer
	Chip_UART_IntDisable(thrInitData->pUart, UART_IER_THREINT);



	//if (thrTxBufferEmpty()) {
		// First Byte: Store in Buffer and initiate TX
		//thrTxAddByte(b);
		// Put this byte on the UART Line.
	if(thrTxBufferEmpty()){
				// first time after reset the THRE IRQ will not be triggered by just enabling it here
				// So we have to really send the byte here and do not put this into buffer. From next byte on
				// we always put bytes to the TX buffer and enabling the IRQ will trigger it when THR (transmit hold register)
				// gets empty (or also if it was and is still empty!)
				// see UM10360 Datasheet rev 4.1  page 315 first paragraph for details of this behavior!
				thrFirstByteAfterReset = false;
				Chip_UART_SendByte(thrInitData->pUart, b);
	} else {
		// add byte to buffer if there is room left
			// and wait for IRQ to fetch it
		if (thrTxAddByte(b) == false) {
			// Buffer Full Error. Byte is skipped -> count errors or signal event ?????
			// .....
		}
	}

	// enable irq after handling tx buffer
	Chip_UART_IntEnable(thrInitData->pUart, UART_IER_THREINT);

}

void thrSendBytes(uint8_t *data, uint8_t len) {
	LAST_STARTED_MODULE=1003;
	//set to transmit when sending data package
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, 2, 5); //  This is PINIDX_RS485_TX_RX TRANSMIT
	for (int i=0;i<len;i++) {
		thrSendByte(data[i]);
	}

}


void thrProcessRxByte(uint8_t rxByte) {
	LAST_STARTED_MODULE=1004;
	// do your processing of RX here....

	if (l4_thr_counter< l4_thr_ExpectedReceiveBuffer){ // change it to expected buffer length SET by REQUEST functions

		thr_receiveBuffer[l4_thr_counter]=(char) rxByte;
		//Chip_UART_SendByte(LPC_UART2, rxByte); // print received byte
		l4_thr_counter++;
	}
	if (l4_thr_counter== l4_thr_ExpectedReceiveBuffer) {

		thr_receiveBuffer[l4_thr_counter]=(char) rxByte;
		//Chip_UART_SendByte(LPC_UART2, rxByte); // print received byte

		l4_thr_counter =0;
		ParseReadRequest((uint8_t*)&thr_receiveBuffer,l4_thr_ExpectedReceiveBuffer);

		char print_str[200];
		sprintf(print_str, "\n Thruster Reply : \n");
		int len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);

		deb_print_pure_debug((uint8_t*)&thr_receiveBuffer,l4_thr_ExpectedReceiveBuffer);

		sprintf(print_str, "\n");
		len = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len);


		// TODO : TEST ParseReadRequest. Make request to read all registers.
		// Parse reply with ParseReadRequest and verify that array of data values are stored correctly

	}

}


// *****************           ********************





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


void PrintAllRegisters(){
	char print_str[200];
	int len;
	for (int i=0;i<=108;i++){

		if (REGISTER_LENGTH != 0){
			sprintf(print_str, "\n [%d] Value = %.6f \n",i, REGISTER_DATA[i]  );
			len = strlen(print_str);
			deb_print_pure_debug((uint8_t *)print_str, len);
		}

	}
}



// received_buffer of arbitrary size
// len length of actual thruster reply
void ParseReadRequest(uint8_t* received_buffer,int len){
	LAST_STARTED_MODULE = 1103;

	char print_str[200];
	int len_print;


	//uint8_t sender = received_buffer[0];
	//uint8_t receiver = received_buffer[1];
	uint8_t message_type = received_buffer[2];
	uint8_t received_checksum = received_buffer[3];
	uint8_t payload_length_1 = received_buffer[4];
	uint8_t payload_length_2 = received_buffer[5];

	// Combine payload length bytes into uint16_t
	uint16_t uint16_payload_length = (payload_length_2 <<8 )| payload_length_1;

	if (message_type == 1){
		sprintf(print_str, "\n Parse Request : Thruster OK reply received \n",uint16_payload_length  );
		len_print = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len_print);
		return; // dont do any further parsing because it is simply OK message after SET request

	}


	/// after payload length is known - it is possible to parse remaining bytes into array
	uint8_t *received_data;
	received_data = &received_buffer[6]; // copy data

	/*
	for(int i=0;i<uint16_payload_length;i++){
		//printf("\n i=%d",i);

		received_data[i]=received_buffer[6+i]; //first 6 bytes of received buffer is header.
		//7th byte of received buffer is first byte of received data



	}
	*/

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

	}
	else{
		// incorrect checksumm - message should not be proccessed
		READ_REQUEST_OK_FLAG = 0;
		return;

	}



	// Now we have an array of received data.Which would be either 1,2 or 4 bytes in case of request to single register


	if(uint16_payload_length ==1){

		// if payload length is only 1 byte
		VALUE_UINT8 = received_data[0];
		// then obviously return type is uint8. And array of received_data would be of a single element.
		// Now after we received correct value. We need to apply conversion multuplier again and convert uint8/uint16 value into double.
		//Store double in array representing REAL values of thruster registers

		// LAST ACCESSED REGISTER should for READ or SET REQUEST should be set by the function which sended the request.
		// It is implied that LAST_ACCESSED_REGISTER was defined correctly before executing ParseReadRequest
		double multiplier = CONVERSION_DOUBLE[LATEST_ACCESSED_REGISTER];
		ACTUAL_VALUE = (double)VALUE_UINT8 / multiplier;

		//printf("\n After conversion multiplier ACTUAL VALUE = %f \n",ACTUAL_VALUE);

		// Finally fill in the global array which will store ALL the register values
		REGISTER_DATA[LATEST_ACCESSED_REGISTER]=ACTUAL_VALUE;

		sprintf(print_str, "\n Parse Read Request: [%d] ACTUAL_VALUE= %.6f \n",LATEST_ACCESSED_REGISTER,ACTUAL_VALUE );
		len_print = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len_print);

	}

	if(uint16_payload_length ==2){

		//if payload length is 2 bytes, then return value should be stored as uint16_t
		VALUE_UINT16 = (received_data[1] <<8 )| received_data[0];
		double multiplier = CONVERSION_DOUBLE[LATEST_ACCESSED_REGISTER];
		ACTUAL_VALUE = (double)VALUE_UINT16 / multiplier;
		REGISTER_DATA[LATEST_ACCESSED_REGISTER]=ACTUAL_VALUE;

		sprintf(print_str, "\n Parse Read Request: [%d] ACTUAL_VALUE= %.6f \n",LATEST_ACCESSED_REGISTER,ACTUAL_VALUE );
		len_print = strlen(print_str);
		deb_print_pure_debug((uint8_t *)print_str, len_print);


	}

	if(uint16_payload_length ==4){
		//TODO: Implement parser for fuse status. Fuses to be stored as uint32_t variable.
		// This TODO is for later design iteration : Thruster test validation, and monitoring loop.
		// Monitoring module for space craft operational variables would be implemented at next design stage.

		//if payload length is 4 (fuses) then return value should be stored with uint32_t
		//uint32_t i32 = v4[0] | (v4[1] << 8) | (v4[2] << 16) | (v4[3] << 24);

	}


	if(uint16_payload_length >5){


		//******************************* NEW CODE ********************8

		memcpy(&THR_REGISTER_DATA.version_major,&received_data[0],uint16_payload_length);
		// ********************************NEW IMPLEMENTATION *********************

		//if payload is more then 4 then we are reading multiple registers

		//sprintf(print_str, "\n Parse Request : Payload Length = %d \n",uint16_payload_length  );
		//len_print = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len_print);


		//sprintf(print_str, "\n Reading Multiple Registers \n"  );
		//len_print = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len_print);

		// Lets say that READ REQUEST TO ALL REGISTERS/ READ MORE THEN ONE REGISTER function will set last address to a value
		// of a first address that is intended to be accessed
		uint8_t next_register_index = LATEST_ACCESSED_REGISTER;
		uint8_t length_of_next_register = REGISTER_LENGTH[next_register_index];
		double multiplier = CONVERSION_DOUBLE[next_register_index];
		//uint8_t starting_register = LATEST_ACCESSED_REGISTER; // first time

		///sprintf(print_str, "\n Start RI= %d_RI_len=%d  Conversion_multiplier = %.2f \n",next_register_index,length_of_next_register,multiplier  );
		//len_print = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len_print);

		int i = 0; // i referres to position in received_datavector, not register_index. Starting i is always 0.
		uint32_t parse_start_time = (uint32_t)timGetSystime();
		uint32_t current_time;
		while( next_register_index<108 ){ // Untill last usable register !  WARNING !!! CHECK LENGTH OF ReadAllRegistersRequest !! This might trigger infitite loop
			// It will trigger infinite loop if thruser reply with length of message that is greaater then 5 and less then enough to cover 101 registers !
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
				VALUE_UINT8 = received_data[i];
				multiplier = CONVERSION_DOUBLE[next_register_index];
				ACTUAL_VALUE = (double)VALUE_UINT8 / multiplier;
				REGISTER_DATA[next_register_index]=ACTUAL_VALUE;

				//sprintf(print_str, "\n [%d] ACTUAL_VALUE= %.6f Conversion=%.6f  uint8_value =%d  \n",next_register_index,ACTUAL_VALUE,multiplier,VALUE_UINT8  );
				//len_print = strlen(print_str);
				//deb_print_pure_debug((uint8_t *)print_str, len_print);

				next_register_index = next_register_index+1;
				i = i+1;

			}

			if(length_of_next_register==2){
				VALUE_UINT16 = (received_data[i+1] <<8 )| received_data[i]; // here problem / Compiles value out of wrong bytes
				double multiplier = CONVERSION_DOUBLE[next_register_index];
				ACTUAL_VALUE = (double)VALUE_UINT16 / multiplier;
				REGISTER_DATA[next_register_index]=ACTUAL_VALUE;

				//sprintf(print_str, "\n [%d] ACTUAL_VALUE= %.6f Conversion=%.6f  uint16_value =%d  \n",next_register_index,ACTUAL_VALUE,multiplier,VALUE_UINT16  );
				//len_print = strlen(print_str);
				//deb_print_pure_debug((uint8_t *)print_str, len_print);

				next_register_index = next_register_index+2;
				i = i+2;


			}

			if(length_of_next_register==4){

				//sprintf(print_str, "\n [%d] Skip Fuse \n",next_register_index  );
				//len_print = strlen(print_str);
				//deb_print_pure_debug((uint8_t *)print_str, len_print);

				next_register_index = next_register_index+4;
				i=i+4;

			}
			length_of_next_register = REGISTER_LENGTH[next_register_index];
			multiplier = CONVERSION_DOUBLE[next_register_index];

			//sprintf(print_str, "\n Parse Iteration End: len_next_RI=%d   next_RI=[%d]  \n",length_of_next_register,next_register_index );
			//len_print = strlen(print_str);
			//deb_print_pure_debug((uint8_t *)print_str, len_print);

		}
	}





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
	request[3] = 0x00; //checksum


	//////Payload length
	if (length_of_register ==1){
		// if register that is intended to be set is one byte
		// then length of payload is register address(1) + data(1)= 2 bytes
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
	LAST_STARTED_MODULE = 1107;

		// FIRST ARGUMENT SHOUD BE uint8_t VALUE OF REGISTER THAT WOULD BE READ FROM
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

