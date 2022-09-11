/*
===============================================================================
 Name        : l7_climb_app.h
 Author      : Robert
 Created on	 : 07.09.2021
===============================================================================
*/

#ifndef MOD_L4_THRUSTER_H_
#define MOD_L4_THRUSTER_H_

#include <chip.h>


// Variables below will store value returned by read single register request
// Output from READ SINGLE REGISTER can be a variable of either 1,2 or 4 bytes long.
// Those global variables will represent LATEST RECEIVED VALUE returned by READ SINGLE REGISTER request
extern uint8_t VALUE_UINT8;
extern uint16_t VALUE_UINT16;
extern uint32_t VALUE_UINT32;
extern uint8_t READ_REQUEST_OK_FLAG;  // if checksum of parsed message was verified, then message is considered valid. Corresponding flag is set to 1
extern uint8_t LATEST_ACCESSED_REGISTER; // Latest register to which SET or READ request have been made
extern double ACTUAL_VALUE; // //ACTUAL VALUE represents real physical value returned by READ SINGLE REGISTER REQUEST
//ACTUAL VALUE represents real physical value returned by READ SINGLE REGISTER REQUEST
extern uint8_t TYPE_OF_LAST_REQUEST;
extern const uint16_t CONVERSION[108];
extern const uint8_t SENDER_ADRESS;
extern const uint8_t DEVICE;
extern const uint8_t MSGTYPE[8];

// REGISTER_DATA will store physical values after READ request
extern double REGISTER_DATA[108];
extern const uint8_t REGISTER_LENGTH[108];
extern const double CONVERSION_DOUBLE[108];


extern int l4_thr_ExpectedReceiveBuffer;
extern int l4_thr_counter; // counter for received bytes

///////////////// Main and Init
void l4_thruster_init (void *dummy);
void l4_thruster_main (void);
void l4_debugPrintBuffer(uint8_t *buffer,int bufferlen);
////////////

//// Thruster request manual functions
void ReadAllRegisters(int argc, char *argv[]);


//////Thruster automated request functions
void GeneralSetRequest(int argc, char *argv[]);
void GeneralReadRequest(int argc, char *argv[]);
void ParseReadRequest(uint8_t* received_buffer,int len);

//Thruster sequences
//void thr_fire_cmd(int argc, char *argv[]);
//void thr_fire_exe();

void thr_execute_sequence_cmd(int argc, char *argv[]);

// Thruster operational registors
//extern bool THRUSTER_FIRING_STATUS;
//extern uint32_t THR_FIRE_DURATION;
//extern uint32_t THR_FIRE_START_TIMESTAMP;
//extern bool THRUSTER_FIRE_FIRST_TIME;
//extern uint16_t THR_FIRE_SI;
//extern uint16_t THR_FIRE_THRUST;


typedef struct thr_variables_t{

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
}thr_variables_t;

#endif /* MOD_L7_CLIMB_APP_H_ */
