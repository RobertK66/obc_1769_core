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


uint8_t LATEST_ACCESSED_REGISTER ;
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
	// Uart Rx
	int32_t stat = Chip_UART_ReadLineStatus(thrInitData->pUart);
	if (stat & UART_LSR_RDR) {
		// there is a byte available. Lets read and process it.


		uint8_t b = Chip_UART_ReadByte(thrInitData->pUart);
		thrProcessRxByte(b);
	}
}

void thrUartIRQ(LPC_USART_T *pUART) {
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
	//set to transmit when sending data package
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, 2, 5); //  This is PINIDX_RS485_TX_RX TRANSMIT
	for (int i=0;i<len;i++) {
		thrSendByte(data[i]);
	}

}


void thrProcessRxByte(uint8_t rxByte) {
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
		//int len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

		//deb_print_pure_debug((uint8_t*)&thr_receiveBuffer,l4_thr_ExpectedReceiveBuffer);
		SysEvent(0, EVENT_INFO, 3, (uint8_t*)&thr_receiveBuffer, l4_thr_ExpectedReceiveBuffer);

		sprintf(print_str, "\n");
		//len = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));


	}

}


// *****************           ********************





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

/*
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
*/


// received_buffer of arbitrary size
// len length of actual thruster reply
void ParseReadRequest(uint8_t* received_buffer,int len){

	char print_str[200];
	//int len_print;


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
		//len_print = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len_print);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));
		return; // dont do any further parsing because it is simply OK message after SET request

	}


	/// after payload length is known - it is possible to parse remaining bytes into array
	uint8_t *received_data;
	received_data = &received_buffer[6]; // copy data



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
		//void WriteThrRegDataStruct(uint8_t value_uint8, uint16_t value_uint16, uint32_t value_uint32, uint8_t register_index)
		WriteThrRegDataStruct(received_data[0],0,0,LATEST_ACCESSED_REGISTER);
		sprintf(print_str, "\n Parse Read Request: [%d] ACTUAL_VALUE= %.6f \n",LATEST_ACCESSED_REGISTER,ReadThrRegData(LATEST_ACCESSED_REGISTER) );
		//len_print = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len_print);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));

	}

	if(uint16_payload_length ==2){

		//if payload length is 2 bytes, then return value should be stored as uint16_t

		uint16_t temp_value_uint16 = (received_data[1] <<8 )| received_data[0];
		WriteThrRegDataStruct(0,temp_value_uint16,0,LATEST_ACCESSED_REGISTER);

		sprintf(print_str, "\n Parse Read Request 2: [%d] ACTUAL_VALUE= %.6f \n",LATEST_ACCESSED_REGISTER,ReadThrRegData(LATEST_ACCESSED_REGISTER) );
		//len_print = strlen(print_str);
		//deb_print_pure_debug((uint8_t *)print_str, len_print);
		SysEvent(0, EVENT_INFO, 3, (uint8_t *)print_str, strlen(print_str));


	}

	if(uint16_payload_length ==4){
		//Fuse status to be stored as uint32_t variable.
		//if payload length is 4 (fuses) then return value should be stored with uint32_t
		uint32_t temp_value_uint32 = received_data[0] | (received_data[1] << 8) | (received_data[2] << 16) | (received_data[3] << 24);
		WriteThrRegDataStruct(0,0,temp_value_uint32,LATEST_ACCESSED_REGISTER);

	}


	if( uint16_payload_length >5 ){

		memcpy(&THR_REGISTER_DATA.version_major,&received_data[0],sizeof(thr_register_data_t));
	}





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



void WriteThrRegDataStruct(uint8_t value_uint8, uint16_t value_uint16, uint32_t value_uint32, uint8_t register_index){
	//:\n\t\t\tTHR_REGISTER_DATA. = value_uint16;\n\t\t\tbreak;

switch (register_index){

case THR_VERSION_MAJOR_REG:
		THR_REGISTER_DATA.version_major = value_uint8;
		break;
case THR_VERSION_MINOR_REG:
		THR_REGISTER_DATA.version_minor = value_uint16;
		break;
case THR_SERIAL_REG:
		THR_REGISTER_DATA.serial = value_uint16;
		break;
case THR_CYCLES_REG:
		THR_REGISTER_DATA.cycles = value_uint16;
		break;
case THR_FUSE_MASK_REG:
		THR_REGISTER_DATA.fuse_mask = value_uint32;
		break;
case THR_FUSE_STATUS_REG:
		THR_REGISTER_DATA.fuse_status = value_uint32;
		break;
case THR_MODE_REG:
		THR_REGISTER_DATA.mode = value_uint8;
		break;
case THR_STATUS_REG:
		THR_REGISTER_DATA.status = value_uint8;
		break;
case THR_THRUST_REF_REG:
		THR_REGISTER_DATA.thrust_ref = value_uint16;
		break;
case THR_THRUST_REG:
		THR_REGISTER_DATA.thrust = value_uint16;
		break;
case THR_SPECIFIC_IMPULSE_REF_REG:
		THR_REGISTER_DATA.specific_impulse_ref = value_uint16;
		break;
case THR_SPECIFIC_IMPULSE_REG:
		THR_REGISTER_DATA.specific_impulse = value_uint16;
		break;
case THR_BUS_VOLTAGE_REG:
		THR_REGISTER_DATA.bus_voltage = value_uint16;
		break;
case THR_DRIVER_VOLTAGE_REG:
		THR_REGISTER_DATA.driver_voltage = value_uint16;
		break;
case THR_BUS_CURRENT_REG:
		THR_REGISTER_DATA.bus_current = value_uint16;
		break;
case THR_EMITTER_MODE_REG:
		THR_REGISTER_DATA.emitter_mode = value_uint8;
		break;
case THR_EMITTER_VOLTAGE_REF_REG:
		THR_REGISTER_DATA.emitter_voltage_ref = value_uint16;
		break;
case THR_EMITTER_VOLTAGE_REG:
		THR_REGISTER_DATA.emitter_voltage = value_uint16;
		break;
case THR_EMITTER_CURRENT_REF_REG:
		THR_REGISTER_DATA.emitter_current_ref = value_uint16;
		break;
case THR_EMITTER_CURRENT_REG:
		THR_REGISTER_DATA.emitter_current = value_uint16;
		break;
case THR_EMITTER_POWER_REF_REG:
		THR_REGISTER_DATA.emitter_power_ref = value_uint16;
		break;
case THR_EMITTER_POWER_REG:
		THR_REGISTER_DATA.emitter_power = value_uint16;
		break;
case THR_EMITTER_DUTY_CYCLE_REF_REG:
		THR_REGISTER_DATA.emitter_duty_cycle_ref = value_uint8;
		break;
case THR_EMITTER_DUTY_CYCLE_REG:
		THR_REGISTER_DATA.emitter_duty_cycle = value_uint8;
		break;
case THR_EXTRACTOR_MODE_REG:
		THR_REGISTER_DATA.extractor_mode = value_uint8;
		break;
case THR_EXTRACTOR_VOLTAGE_REF_REG:
		THR_REGISTER_DATA.extractor_voltage_ref = value_uint16;
		break;
case THR_EXTRACTOR_VOLTAGE_REG:
		THR_REGISTER_DATA.extractor_voltage = value_uint16;
		break;
case THR_EXTRACTOR_CURRENT_REF_REG:
		THR_REGISTER_DATA.extractor_current_ref = value_uint16;
		break;
case THR_EXTRACTOR_CURRENT_REG:
		THR_REGISTER_DATA.extractor_current = value_uint16;
		break;
case THR_EXTRACTOR_POWER_REF_REG:
		THR_REGISTER_DATA.extractor_power_ref = value_uint16;
		break;
case THR_EXTRACTOR_POWER_REG:
		THR_REGISTER_DATA.extractor_power = value_uint16;
		break;
case THR_EXTRACTOR_DUTY_CYCLE_REF_REG:
		THR_REGISTER_DATA.extractor_duty_cycle_ref = value_uint8;
		break;
case THR_EXTRACTOR_DUTY_CYCLE_REG:
		THR_REGISTER_DATA.extractor_duty_cycle = value_uint8;
		break;
case THR_HEATER_MODE_REG:
		THR_REGISTER_DATA.heater_mode = value_uint8;
		break;
case THR_HEATER_VOLTAGE_REF_REG:
		THR_REGISTER_DATA.heater_voltage_ref = value_uint16;
		break;
case THR_HEATER_VOLTAGE_REG:
		THR_REGISTER_DATA.heater_voltage = value_uint16;
		break;
case THR_HEATER_CURRENT_REF_REG:
		THR_REGISTER_DATA.heater_current_ref = value_uint16;
		break;
case THR_HEATER_CURRENT_REG:
		THR_REGISTER_DATA.heater_current = value_uint16;
		break;
case THR_HEATER_POWER_REF_REG:
		THR_REGISTER_DATA.heater_power_ref = value_uint16;
		break;
case THR_HEATER_POWER_REG:
		THR_REGISTER_DATA.heater_power = value_uint16;
		break;
case THR_HEATER_DUTY_CYCLE_REF_REG:
		THR_REGISTER_DATA.heater_duty_cycle_ref = value_uint8;
		break;
case THR_HEATER_DUTY_CYCLE_REG:
		THR_REGISTER_DATA.heater_duty_cycle = value_uint8;
		break;
case THR_NEUTRALIZER_MODE_REG:
		THR_REGISTER_DATA.neutralizer_mode = value_uint8;
		break;
case THR_NEUTRALIZER_FILAMENT_REF_REG:
		THR_REGISTER_DATA.neutralizer_filament_ref = value_uint8;
		break;
case THR_NEUTRALIZER_FILAMENT_REG:
		THR_REGISTER_DATA.neutralizer_filament = value_uint8;
		break;
case THR_NEUTRALIZER_BIAS_REF_REG:
		THR_REGISTER_DATA.neutralizer_bias_ref = value_uint8;
		break;
case THR_NEUTRALIZER_BIAS_REG:
		THR_REGISTER_DATA.neutralizer_bias = value_uint8;
		break;
case THR_NEUTRALIZER_BIAS_VOLTAGE_REG:
		THR_REGISTER_DATA.neutralizer_bias_voltage = value_uint16;
		break;
case THR_NEUTRALIZER_CURRENT_REF_REG:
		THR_REGISTER_DATA.neutralizer_current_ref = value_uint16;
		break;
case THR_NEUTRALIZER_CURRENT_REG:
		THR_REGISTER_DATA.neutralizer_current = value_uint16;
		break;
case THR_NEUTRALIZER_POWER_REF_REG:
		THR_REGISTER_DATA.neutralizer_power_ref = value_uint16;
		break;
case THR_NEUTRALIZER_POWER_REG:
		THR_REGISTER_DATA.neutralizer_power = value_uint16;
		break;
case THR_NEUTRALIZER_DUTY_CYCLE_REF_REG:
		THR_REGISTER_DATA.neutralizer_duty_cycle_ref = value_uint8;
		break;
case THR_NEUTRALIZER_DUTY_CYCLE_REG:
		THR_REGISTER_DATA.neutralizer_duty_cycle = value_uint8;
		break;
case THR_NEUTRALIZER_BEAM_CURRENT_REF_REG:
		THR_REGISTER_DATA.neutralizer_beam_current_ref = value_uint16;
		break;
case THR_NEUTRALIZER_BEAM_CURRENT_REG:
		THR_REGISTER_DATA.neutralizer_beam_current = value_uint16;
		break;
case THR_TEMPERATURE_MODE_REG:
		THR_REGISTER_DATA.temperature_mode = value_uint8;
		break;
case THR_TEMPERATURE_RESERVOIR_REF_REG:
		THR_REGISTER_DATA.temperature_reservoir_ref = value_uint16;
		break;
case THR_TEMPERATURE_RESERVOIR_REG:
		THR_REGISTER_DATA.temperature_reservoir = value_uint16;
		break;
case THR_TEMPERATURE_HOUSING_REG:
		THR_REGISTER_DATA.temperature_housing = value_uint16;
		break;
case THR_TEMPERATURE_BOARD_REG:
		THR_REGISTER_DATA.temperature_board = value_uint16;
		break;
//case THR_TEMPERATURE_THERMOPILE_REG:
//		THR_REGISTER_DATA.thermopile = value_uint16;
//		break;
//case THR_TEMPERATURE_CALIBRATION_REG:
//		THR_REGISTER_DATA.calibration = value_uint16;
//		break;

			}

}







void SetEncodedThrRegValue(double value, uint8_t register_index){
	//:\n\t\t\tTHR_REGISTER_DATA. = value_uint16;\n\t\t\tbreak;




switch (register_index){

case THR_VERSION_MAJOR_REG:
		THR_REGISTER_DATA.version_major = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_VERSION_MINOR_REG:
		THR_REGISTER_DATA.version_minor = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_SERIAL_REG:
		THR_REGISTER_DATA.serial = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_CYCLES_REG:
		THR_REGISTER_DATA.cycles = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_FUSE_MASK_REG:
		THR_REGISTER_DATA.fuse_mask = (uint32_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_FUSE_STATUS_REG:
		THR_REGISTER_DATA.fuse_status = (uint32_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_MODE_REG:
		THR_REGISTER_DATA.mode = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_STATUS_REG:
		THR_REGISTER_DATA.status = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_THRUST_REF_REG:
		THR_REGISTER_DATA.thrust_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_THRUST_REG:
		THR_REGISTER_DATA.thrust = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_SPECIFIC_IMPULSE_REF_REG:
		THR_REGISTER_DATA.specific_impulse_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_SPECIFIC_IMPULSE_REG:
		THR_REGISTER_DATA.specific_impulse = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_BUS_VOLTAGE_REG:
		THR_REGISTER_DATA.bus_voltage = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_DRIVER_VOLTAGE_REG:
		THR_REGISTER_DATA.driver_voltage = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_BUS_CURRENT_REG:
		THR_REGISTER_DATA.bus_current = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EMITTER_MODE_REG:
		THR_REGISTER_DATA.emitter_mode = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EMITTER_VOLTAGE_REF_REG:
		THR_REGISTER_DATA.emitter_voltage_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EMITTER_VOLTAGE_REG:
		THR_REGISTER_DATA.emitter_voltage = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EMITTER_CURRENT_REF_REG:
		THR_REGISTER_DATA.emitter_current_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EMITTER_CURRENT_REG:
		THR_REGISTER_DATA.emitter_current = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EMITTER_POWER_REF_REG:
		THR_REGISTER_DATA.emitter_power_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EMITTER_POWER_REG:
		THR_REGISTER_DATA.emitter_power = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EMITTER_DUTY_CYCLE_REF_REG:
		THR_REGISTER_DATA.emitter_duty_cycle_ref = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EMITTER_DUTY_CYCLE_REG:
		THR_REGISTER_DATA.emitter_duty_cycle = (uint8_t)( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EXTRACTOR_MODE_REG:
		THR_REGISTER_DATA.extractor_mode = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EXTRACTOR_VOLTAGE_REF_REG:
		THR_REGISTER_DATA.extractor_voltage_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EXTRACTOR_VOLTAGE_REG:
		THR_REGISTER_DATA.extractor_voltage = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EXTRACTOR_CURRENT_REF_REG:
		THR_REGISTER_DATA.extractor_current_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EXTRACTOR_CURRENT_REG:
		THR_REGISTER_DATA.extractor_current = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EXTRACTOR_POWER_REF_REG:
		THR_REGISTER_DATA.extractor_power_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EXTRACTOR_POWER_REG:
		THR_REGISTER_DATA.extractor_power = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EXTRACTOR_DUTY_CYCLE_REF_REG:
		THR_REGISTER_DATA.extractor_duty_cycle_ref = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_EXTRACTOR_DUTY_CYCLE_REG:
		THR_REGISTER_DATA.extractor_duty_cycle = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_HEATER_MODE_REG:
		THR_REGISTER_DATA.heater_mode = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_HEATER_VOLTAGE_REF_REG:
		THR_REGISTER_DATA.heater_voltage_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_HEATER_VOLTAGE_REG:
		THR_REGISTER_DATA.heater_voltage = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_HEATER_CURRENT_REF_REG:
		THR_REGISTER_DATA.heater_current_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_HEATER_CURRENT_REG:
		THR_REGISTER_DATA.heater_current = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_HEATER_POWER_REF_REG:
		THR_REGISTER_DATA.heater_power_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_HEATER_POWER_REG:
		THR_REGISTER_DATA.heater_power = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_HEATER_DUTY_CYCLE_REF_REG:
		THR_REGISTER_DATA.heater_duty_cycle_ref = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_HEATER_DUTY_CYCLE_REG:
		THR_REGISTER_DATA.heater_duty_cycle = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_MODE_REG:
		THR_REGISTER_DATA.neutralizer_mode = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_FILAMENT_REF_REG:
		THR_REGISTER_DATA.neutralizer_filament_ref = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_FILAMENT_REG:
		THR_REGISTER_DATA.neutralizer_filament = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_BIAS_REF_REG:
		THR_REGISTER_DATA.neutralizer_bias_ref = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_BIAS_REG:
		THR_REGISTER_DATA.neutralizer_bias = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_BIAS_VOLTAGE_REG:
		THR_REGISTER_DATA.neutralizer_bias_voltage = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_CURRENT_REF_REG:
		THR_REGISTER_DATA.neutralizer_current_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_CURRENT_REG:
		THR_REGISTER_DATA.neutralizer_current = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_POWER_REF_REG:
		THR_REGISTER_DATA.neutralizer_power_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_POWER_REG:
		THR_REGISTER_DATA.neutralizer_power = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_DUTY_CYCLE_REF_REG:
		THR_REGISTER_DATA.neutralizer_duty_cycle_ref = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_DUTY_CYCLE_REG:
		THR_REGISTER_DATA.neutralizer_duty_cycle = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_BEAM_CURRENT_REF_REG:
		THR_REGISTER_DATA.neutralizer_beam_current_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_NEUTRALIZER_BEAM_CURRENT_REG:
		THR_REGISTER_DATA.neutralizer_beam_current = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_TEMPERATURE_MODE_REG:
		THR_REGISTER_DATA.temperature_mode = (uint8_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_TEMPERATURE_RESERVOIR_REF_REG:
		THR_REGISTER_DATA.temperature_reservoir_ref = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_TEMPERATURE_RESERVOIR_REG:
		THR_REGISTER_DATA.temperature_reservoir = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_TEMPERATURE_HOUSING_REG:
		THR_REGISTER_DATA.temperature_housing = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
case THR_TEMPERATURE_BOARD_REG:
		THR_REGISTER_DATA.temperature_board = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
		break;
//case THR_TEMPERATURE_THERMOPILE_REG:
//		THR_REGISTER_DATA.thermopile = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
//		break;
//case THR_TEMPERATURE_CALIBRATION_REG:
//		THR_REGISTER_DATA.calibration = (uint16_t) ( value * CONVERSION_DOUBLE[register_index]);
//		break;

			}

}




double ReadThrRegData(uint8_t register_index){

	double ret;

switch (register_index){

case THR_VERSION_MAJOR_REG:
		ret = (double)THR_REGISTER_DATA.version_major /1;
		break;
case THR_VERSION_MINOR_REG:
		ret = (double)THR_REGISTER_DATA.version_minor/1;
		break;
case THR_SERIAL_REG:
		ret = (double)THR_REGISTER_DATA.serial/1;
		break;
case THR_CYCLES_REG:
		ret = (double)THR_REGISTER_DATA.cycles/1;
		break;
case THR_FUSE_MASK_REG:
		ret = (double)THR_REGISTER_DATA.fuse_mask/1;
		break;
case THR_FUSE_STATUS_REG:
		ret = (double)THR_REGISTER_DATA.fuse_status/1;
		break;
case THR_MODE_REG:
		ret = (double)THR_REGISTER_DATA.mode/1;
		break;
case THR_STATUS_REG:
		ret = (double)THR_REGISTER_DATA.status/1;
		break;
case THR_THRUST_REF_REG:
		ret = (double)THR_REGISTER_DATA.thrust_ref/10000000.0;
		break;
case THR_THRUST_REG:
		ret = (double)THR_REGISTER_DATA.thrust/10000000.0;
		break;
case THR_SPECIFIC_IMPULSE_REF_REG:
		ret = (double)THR_REGISTER_DATA.specific_impulse_ref/1;
		break;
case THR_SPECIFIC_IMPULSE_REG:
		ret = (double)THR_REGISTER_DATA.specific_impulse /1;
		break;
case THR_BUS_VOLTAGE_REG:
		ret = (double)THR_REGISTER_DATA.bus_voltage /1000.0;
		break;
case THR_DRIVER_VOLTAGE_REG:
		ret = (double)THR_REGISTER_DATA.driver_voltage /1000.0;
		break;
case THR_BUS_CURRENT_REG:
		ret = (double)THR_REGISTER_DATA.bus_current /10000.0;
		break;
case THR_EMITTER_MODE_REG:
		ret = (double)THR_REGISTER_DATA.emitter_mode /1.0;
		break;
case THR_EMITTER_VOLTAGE_REF_REG:
		ret = (double)THR_REGISTER_DATA.emitter_voltage_ref /1.0;
		break;
case THR_EMITTER_VOLTAGE_REG:
		ret = (double)THR_REGISTER_DATA.emitter_voltage /1;
		break;
case THR_EMITTER_CURRENT_REF_REG:
		ret = (double)THR_REGISTER_DATA.emitter_current_ref /10000000.0;
		break;
case THR_EMITTER_CURRENT_REG:
		ret = (double)THR_REGISTER_DATA.emitter_current /10000000.0;
		break;
case THR_EMITTER_POWER_REF_REG:
		ret = (double)THR_REGISTER_DATA.emitter_power_ref /1000.0;
		break;
case THR_EMITTER_POWER_REG:
		ret = (double)THR_REGISTER_DATA.emitter_power /1000.0;
		break;
case THR_EMITTER_DUTY_CYCLE_REF_REG:
		ret = (double)THR_REGISTER_DATA.emitter_duty_cycle_ref /2.5499999999999874;
		break;
case THR_EMITTER_DUTY_CYCLE_REG:
		ret = (double)THR_REGISTER_DATA.emitter_duty_cycle / 2.5499999999999874;
		break;
case THR_EXTRACTOR_MODE_REG:
		ret = (double)THR_REGISTER_DATA.extractor_mode /1.0;
		break;
case THR_EXTRACTOR_VOLTAGE_REF_REG:
		ret = (double)THR_REGISTER_DATA.extractor_voltage_ref /1.0;
		break;
case THR_EXTRACTOR_VOLTAGE_REG:
		ret = (double)THR_REGISTER_DATA.extractor_voltage /1.0;
		break;
case THR_EXTRACTOR_CURRENT_REF_REG:
		ret = (double)THR_REGISTER_DATA.extractor_current_ref /100000000.0;
		break;
case THR_EXTRACTOR_CURRENT_REG:
		ret = (double)THR_REGISTER_DATA.extractor_current / 100000000.0;
		break;
case THR_EXTRACTOR_POWER_REF_REG:
		ret = (double)THR_REGISTER_DATA.extractor_power_ref /10000.0;
		break;
case THR_EXTRACTOR_POWER_REG:
		ret = (double)THR_REGISTER_DATA.extractor_power / 10000.0;
		break;
case THR_EXTRACTOR_DUTY_CYCLE_REF_REG:
		ret = (double)THR_REGISTER_DATA.extractor_duty_cycle_ref /2.5499999999999874;
		break;
case THR_EXTRACTOR_DUTY_CYCLE_REG:
		ret = (double)THR_REGISTER_DATA.extractor_duty_cycle / 2.5499999999999874;
		break;
case THR_HEATER_MODE_REG:
		ret = (double)THR_REGISTER_DATA.heater_mode / 1.0;
		break;
case THR_HEATER_VOLTAGE_REF_REG:
		ret = (double)THR_REGISTER_DATA.heater_voltage_ref / 1000.0;
		break;
case THR_HEATER_VOLTAGE_REG:
		ret = (double)THR_REGISTER_DATA.heater_voltage / 1000.0;
		break;
case THR_HEATER_CURRENT_REF_REG:
		ret = (double)THR_REGISTER_DATA.heater_current_ref / 10000.0;
		break;
case THR_HEATER_CURRENT_REG:
		ret = (double)THR_REGISTER_DATA.heater_current / 10000.0;
		break;
case THR_HEATER_POWER_REF_REG:
		ret = (double)THR_REGISTER_DATA.heater_power_ref / 1000.0;
		break;
case THR_HEATER_POWER_REG:
		ret = (double)THR_REGISTER_DATA.heater_power / 1000.0;
		break;
case THR_HEATER_DUTY_CYCLE_REF_REG:
		ret = (double)THR_REGISTER_DATA.heater_duty_cycle_ref / 2.5499999999999874;
		break;
case THR_HEATER_DUTY_CYCLE_REG:
		ret = (double)THR_REGISTER_DATA.heater_duty_cycle / 2.5499999999999874;
		break;
case THR_NEUTRALIZER_MODE_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_mode / 1.0;
		break;
case THR_NEUTRALIZER_FILAMENT_REF_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_filament_ref / 1.0;
		break;
case THR_NEUTRALIZER_FILAMENT_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_filament /1.0;
		break;
case THR_NEUTRALIZER_BIAS_REF_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_bias_ref / 1.0;
		break;
case THR_NEUTRALIZER_BIAS_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_bias / 1.0;
		break;
case THR_NEUTRALIZER_BIAS_VOLTAGE_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_bias_voltage / 100.0;
		break;
case THR_NEUTRALIZER_CURRENT_REF_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_current_ref / 10000.0;
		break;
case THR_NEUTRALIZER_CURRENT_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_current / 10000.0;
		break;
case THR_NEUTRALIZER_POWER_REF_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_power_ref / 1000.0;
		break;
case THR_NEUTRALIZER_POWER_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_power / 1000.0;
		break;
case THR_NEUTRALIZER_DUTY_CYCLE_REF_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_duty_cycle_ref / 2.5499999999999874;
		break;
case THR_NEUTRALIZER_DUTY_CYCLE_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_duty_cycle  / 2.5499999999999874;
		break;
case THR_NEUTRALIZER_BEAM_CURRENT_REF_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_beam_current_ref / 10000000.0;
		break;
case THR_NEUTRALIZER_BEAM_CURRENT_REG:
		ret = (double)THR_REGISTER_DATA.neutralizer_beam_current / 10000000.0;
		break;
case THR_TEMPERATURE_MODE_REG:
		ret = (double)THR_REGISTER_DATA.temperature_mode / 1.0;
		break;
case THR_TEMPERATURE_RESERVOIR_REF_REG:
		ret = (double)THR_REGISTER_DATA.temperature_reservoir_ref / 100.0;
		break;
case THR_TEMPERATURE_RESERVOIR_REG:
		ret = (double)THR_REGISTER_DATA.temperature_reservoir / 100.0;
		break;
case THR_TEMPERATURE_HOUSING_REG:
		ret = (double)THR_REGISTER_DATA.temperature_housing / 100.0;
		break;
case THR_TEMPERATURE_BOARD_REG:
		ret = (double)THR_REGISTER_DATA.temperature_board / 100.0;
		break;
//case THR_TEMPERATURE_THERMOPILE_REG:
//		THR_REGISTER_DATA.thermopile / 1000.0;
//		break;
//case THR_TEMPERATURE_CALIBRATION_REG:
//		THR_REGISTER_DATA.calibration / 1.0;
//		break;

			}
			return ret;
}

