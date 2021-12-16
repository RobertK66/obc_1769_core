/*
===============================================================================
 Name        : l3_sensors.h
 Author      : Robert
 Created on	 : 14.12.2021
===============================================================================
*/

#ifndef MOD_L3_SENSORS_H_
#define MOD_L3_SENSORS_H_

#include <Chip.h>

// Module base
void sen_init(void *);
void sen_main();

// Module Functions API
typedef struct {
	float	SupplyVoltage;
	float	SupplyCurrentBoard;
	float	SupplyCurrentSidepanels;
	float	Temperature;
} sensor_values_t;

sensor_values_t SenReadAllValues();

#endif /* MOD_L3_SENSORS_H_ */
