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




extern const uint16_t CONVERSION[108];
extern const uint8_t REGISTER_VALUES[108];
extern const uint8_t SENDER_ADRESS;
extern const uint8_t DEVICE;
extern const uint8_t MSGTYPE[8];

extern int l4_thr_ExpectedReceiveBuffer;
extern int l4_thr_counter; // counter for received bytes


void l4_thruster_init (void *dummy);
void l4_thruster_main (void);
void l4_debugPrintBuffer(uint8_t *buffer,int bufferlen);


void SetHeaterMode(int argc, char *argv[]);
void SetReservoirTemperature(int argc, char *argv[]);
void ThrSendVersionRequestCmd(int argc, char *argv[]);
void ReadHeaterCurrent(int argc, char *argv[]);
void SetHeaterPower(int argc, char *argv[]);
void SetHeaterCurrent(int argc, char *argv[]);

void SetHeaterVoltage(int argc, char *argv[]);

void ReadAllRegisters(int argc, char *argv[]);



#endif /* MOD_L7_CLIMB_APP_H_ */
