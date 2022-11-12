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






///////////////// Main and Init
void l4_thruster_init (void *dummy);
void l4_thruster_main (void);
void l4_debugPrintBuffer(uint8_t *buffer,int bufferlen);
////////////





void thr_execute_sequence_cmd(int argc, char *argv[]);
void mem_write_cmd(int argc, char *argv[]);




#endif /* MOD_L7_CLIMB_APP_H_ */
