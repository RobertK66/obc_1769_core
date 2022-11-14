/*
 * thr.h
 *
 *  Created on: 01.03.2022
 *      Copy Paste by: Jevgeni
 */

//
#ifndef MOD_TIM_CLIMB_THR_H_
#define MOD_TIM_CLIMB_THR_H_

#include <chip.h>



// Variables below will store value returned by read single register request
// Output from READ SINGLE REGISTER can be a variable of either 1,2 or 4 bytes long.
// Those global variables will represent LATEST RECEIVED VALUE returned by READ SINGLE REGISTER request
//extern uint8_t VALUE_UINT8;
//extern uint16_t VALUE_UINT16;
//extern uint32_t VALUE_UINT32;
extern uint8_t READ_REQUEST_OK_FLAG;  // if checksum of parsed message was verified, then message is considered valid. Corresponding flag is set to 1
extern uint8_t LATEST_ACCESSED_REGISTER; // Latest register to which SET or READ request have been made
//extern double ACTUAL_VALUE; // //ACTUAL VALUE represents real physical value returned by READ SINGLE REGISTER REQUEST
//ACTUAL VALUE represents real physical value returned by READ SINGLE REGISTER REQUEST
extern uint8_t TYPE_OF_LAST_REQUEST;
extern const uint16_t CONVERSION[108];
extern const uint8_t SENDER_ADRESS;
extern const uint8_t DEVICE;
extern const uint8_t MSGTYPE[8];

// REGISTER_DATA will store physical values after READ request
//extern double REGISTER_DATA[108];
extern const uint8_t REGISTER_LENGTH[108];
extern const double CONVERSION_DOUBLE[108];

extern int l4_thr_ExpectedReceiveBuffer;
extern int l4_thr_counter;







typedef struct thr_register_data_t{

	// Structure is declared if we ever intend to store register values as a user friendly name
	// TODO: Decide if we do it this way. If yes - update ParseReadRequest() function to set data into this structure

uint8_t version_major;
uint8_t version_minor;
uint16_t serial;
uint16_t cycles;
uint32_t fuse_mask;
uint32_t fuse_status;
uint8_t mode;
uint8_t status;
uint16_t thrust_ref;
uint16_t thrust;
uint16_t specific_impulse_ref;
uint16_t specific_impulse;
uint16_t bus_voltage;
uint16_t driver_voltage;
uint16_t bus_current;
uint8_t emitter_mode;
uint16_t emitter_voltage_ref;
uint16_t emitter_voltage;
uint16_t emitter_current_ref;
uint16_t emitter_current;
uint16_t emitter_power_ref;
uint16_t emitter_power;
uint8_t emitter_duty_cycle_ref;
uint8_t emitter_duty_cycle;
uint8_t extractor_mode;
uint16_t extractor_voltage_ref;
uint16_t extractor_voltage;
uint16_t extractor_current_ref;
uint16_t extractor_current;
uint16_t extractor_power_ref;
uint16_t extractor_power;
uint8_t extractor_duty_cycle_ref;
uint8_t extractor_duty_cycle;
uint8_t heater_mode;
uint16_t heater_voltage_ref;
uint16_t heater_voltage;
uint16_t heater_current_ref;
uint16_t heater_current;
uint16_t heater_power_ref;
uint16_t heater_power;
uint8_t heater_duty_cycle_ref;
uint8_t heater_duty_cycle;
uint8_t neutralizer_mode;
uint8_t neutralizer_filament_ref;
uint8_t neutralizer_filament;
uint8_t neutralizer_bias_ref;
uint8_t neutralizer_bias;
uint16_t neutralizer_bias_voltage;
uint16_t neutralizer_current_ref;
uint16_t neutralizer_current;
uint16_t neutralizer_power_ref;
uint16_t neutralizer_power;
uint8_t neutralizer_duty_cycle_ref;
uint8_t neutralizer_duty_cycle;
uint16_t neutralizer_beam_current_ref;
uint16_t neutralizer_beam_current;
uint8_t temperature_mode;
uint16_t temperature_reservoir_ref;
uint16_t temperature_reservoir;
uint16_t temperature_housing;
uint16_t temperature_board;
uint16_t temperature_thermopile;
uint16_t temperature_calibration;
}__attribute__((packed)) thr_register_data_t;

extern thr_register_data_t THR_REGISTER_DATA;

#define	THR_VERSION_MAJOR_REG 			0
#define	THR_VERSION_MINOR_REG 			1
#define	THR_SERIAL_REG 					2
#define	THR_CYCLES_REG 					4
#define	THR_FUSE_MASK_REG 				6
#define	THR_FUSE_STATUS_REG 			10
#define	THR_MODE_REG 					14
#define	THR_STATUS_REG 					15
#define	THR_THRUST_REF_REG 				16
#define	THR_THRUST_REG 					18
#define	THR_SPECIFIC_IMPULSE_REF_REG 	20
#define	THR_SPECIFIC_IMPULSE_REG 		22
#define	THR_BUS_VOLTAGE_REG 			24
#define	THR_DRIVER_VOLTAGE_REG 			26
#define	THR_BUS_CURRENT_REG 			28
#define	THR_EMITTER_MODE_REG 			30
#define	THR_EMITTER_VOLTAGE_REF_REG 	31
#define	THR_EMITTER_VOLTAGE_REG 		33
#define	THR_EMITTER_CURRENT_REF_REG 	35
#define	THR_EMITTER_CURRENT_REG 		37
#define	THR_EMITTER_POWER_REF_REG 		39
#define	THR_EMITTER_POWER_REG 			41
#define	THR_EMITTER_DUTY_CYCLE_REF_REG 	43
#define	THR_EMITTER_DUTY_CYCLE_REG 		44
#define	THR_EXTRACTOR_MODE_REG 			45
#define	THR_EXTRACTOR_VOLTAGE_REF_REG 	46
#define	THR_EXTRACTOR_VOLTAGE_REG 		48
#define	THR_EXTRACTOR_CURRENT_REF_REG 	50
#define	THR_EXTRACTOR_CURRENT_REG		52
#define	THR_EXTRACTOR_POWER_REF_REG		54
#define	THR_EXTRACTOR_POWER_REG			56
#define	THR_EXTRACTOR_DUTY_CYCLE_REF_REG	58
#define	THR_EXTRACTOR_DUTY_CYCLE_REG	59
#define	THR_HEATER_MODE_REG				60
#define	THR_HEATER_VOLTAGE_REF_REG		61
#define	THR_HEATER_VOLTAGE_REG			63
#define	THR_HEATER_CURRENT_REF_REG		65
#define	THR_HEATER_CURRENT_REG			67
#define	THR_HEATER_POWER_REF_REG		69
#define	THR_HEATER_POWER_REG			71
#define	THR_HEATER_DUTY_CYCLE_REF_REG	73
#define	THR_HEATER_DUTY_CYCLE_REG		74
#define	THR_NEUTRALIZER_MODE_REG		75
#define	THR_NEUTRALIZER_FILAMENT_REF_REG	76
#define	THR_NEUTRALIZER_FILAMENT_REG	77
#define	THR_NEUTRALIZER_BIAS_REF_REG	78
#define	THR_NEUTRALIZER_BIAS_REG		79
#define	THR_NEUTRALIZER_BIAS_VOLTAGE_REG	80
#define	THR_NEUTRALIZER_CURRENT_REF_REG	82
#define	THR_NEUTRALIZER_CURRENT_REG		84
#define	THR_NEUTRALIZER_POWER_REF_REG	86
#define	THR_NEUTRALIZER_POWER_REG		88
#define	THR_NEUTRALIZER_DUTY_CYCLE_REF_REG	90
#define	THR_NEUTRALIZER_DUTY_CYCLE_REG	91
#define	THR_NEUTRALIZER_BEAM_CURRENT_REF_REG	92
#define	THR_NEUTRALIZER_BEAM_CURRENT_REG	94
#define	THR_TEMPERATURE_MODE_REG		96
#define	THR_TEMPERATURE_RESERVOIR_REF_REG	97
#define	THR_TEMPERATURE_RESERVOIR_REG	99
#define	THR_TEMPERATURE_HOUSING_REG		101
#define	THR_TEMPERATURE_BOARD_REG		103
#define	THR_TEMPERATURE_THERMOPILE_REG	105
#define	THR_TEMPERATURE_CALIBRATION_REG	107





// init data needed. Choose UARt and 2 GPIO Pins to be used
typedef struct {
	LPC_USART_T 	*pUart;			// default is 9600baud
} thr_initdata_t;

// API module functions
void thrInit (void *initData);
void thrMain (void);

void thrSendBytes(uint8_t *data, uint8_t len);

//////Thruster automated request functions
void GeneralSetRequest(int argc, char *argv[]);
void GeneralReadRequest(int argc, char *argv[]);
void ParseReadRequest(uint8_t* received_buffer,int len);
//// Thruster request manual functions
void ReadAllRegisters(int argc, char *argv[]);
void PrintAllRegisters();

void WriteThrRegDataStruct(uint8_t value_uint8, uint16_t value_uint16, uint32_t value_uint32, uint8_t register_index);
double ReadThrRegData(uint8_t register_index);
void SetEncodedThrRegValue(double value, uint8_t register_index);



#endif /* MOD_TIM_CLIMB_GPS_H_ */
