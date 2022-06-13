/*
===============================================================================
 Copy paste from l3_sensor by       : Jevgeni Potulov
 Created on	 : 07.06.2022
===============================================================================
*/

#ifndef MOD_I2C_ARDUINO_H_
#define MOD_I2C_ARDUINO_H_

#include <chip.h>

#include "../ai2c/obc_i2c.h"
#include "../l7_climb_app.h"
#include "../l2_debug_com.h"
#include "../../ClimbObc.h"


// init data needed. Choose UARt and 2 GPIO Pins to be used
typedef struct {
	LPC_I2C_T 	*pI2C;
	uint16_t frequency;
} i2c_arduino_initdata_t;


//////
static bool readInProgress = false;
///////

// Module base
void i2c_arduino_init(void *);
void i2c_arduino_main();




bool i2cArduino_Read();
bool i2cArduino_SendReadRequest();
void i2c_debugPrintBuffer(uint8_t *buffer,int bufferlen);
void i2c_Proccess_Received_Buffer(I2C_Data i2cJob, uint8_t *i2c_buffer,uint8_t i2c_buffer_len);

#endif /* MOD_L3_SENSORS_H_ */
