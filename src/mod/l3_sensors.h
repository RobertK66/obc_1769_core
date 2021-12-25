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

#define MODULE_ID_SENSORS			0x02
#define EID_SEN_MEASSUREMENT		1


// Module Functions API
typedef struct {
	float	SupplyVoltage;
	float	SupplyCurrentBoard;
	float	SupplyCurrentSidepanels;
	float	Temperature;
	float   TempSHT30;
	float   HumidityPercent;
} sensor_values_t;

void SenReadAllValues();

#endif /* MOD_L3_SENSORS_H_ */
