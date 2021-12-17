/*
===============================================================================
 Name        : l7_climb_app.h
 Author      : Robert
 Created on	 : 07.09.2021
===============================================================================
*/

#ifndef MOD_L7_CLIMB_APP_H_
#define MOD_L7_CLIMB_APP_H_

#include <Chip.h>

void app_init (void *dummy);
void app_main (void);

#define MODULE_ID_CLIMBAPP		0x00
#define EID_APP_SENSORVALUES	1
#define EID_APP_RAWDATA			2
#define EID_APP_STRING			3

#endif /* MOD_L7_CLIMB_APP_H_ */
