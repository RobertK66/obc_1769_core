/*
 * radsensor.h
 *
 *  Created on: 19.05.2023
 *      Author: Robert
 */

#ifndef MOD_SRS_RADSENSOR_H_
#define MOD_SRS_RADSENSOR_H_

#include <chip.h>

// init data needed. Choose I2C bus to use
typedef struct {
	LPC_I2C_T 	*pI2C;
	uint8_t		i2cAdr;
} srs_initdata_t;

// Module base
void srs_init(void *);
void srs_main();


// Module API
void srs_test_cmd(int argc, char *argv[]);


#define MODULE_ID_RADSENSOR		0x05

#define EID_SRS_RAWDATA			0x01


#endif /* MOD_SRS_RADSENSOR_H_ */
