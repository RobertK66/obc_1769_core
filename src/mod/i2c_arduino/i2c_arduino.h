/*
===============================================================================
 Copy paste from l3_sensor by       : Jevgeni Potulov
 Created on	 : 07.06.2022
===============================================================================
*/

#ifndef MOD_I2C_ARDUINO_H_
#define MOD_I2C_ARDUINO_H_

#include <chip.h>


// init data needed. Choose UARt and 2 GPIO Pins to be used
typedef struct {
	LPC_I2C_T 	*pI2C;
	uint16_t frequency;
} i2c_arduino_initdata_t;


// Module base
void i2c_arduino_init(void *);
void i2c_arduino_main();





bool i2cArduino_SendReadRequest();

#endif /* MOD_L3_SENSORS_H_ */
