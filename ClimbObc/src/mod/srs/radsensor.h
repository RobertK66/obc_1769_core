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
//	uint8_t		i2cDataAdr;
//	uint8_t		i2cPowerAdr;
} srs_initdata_t;

// Module base
void srs_init(void *);
void srs_main();

// Module API
void srs_enable(void);
void srs_disable(void);

void srs_cmd(int argc, char *argv[]);

// Module Events
#define MODULE_ID_RADSENSOR		0x06
#define EID_SRS_DATATX			0x00
#define EID_SRS_DATARX			0x01
#define EID_SRS_POWERON			0x02
#define EID_SRS_POWEROFF		0x03
#define EID_SRS_TIME			0x04
#define EID_SRS_STATUS			0x05
#define EID_SRS_INTERVALS		0x06
#define EID_SRS_INTERVAL		0x07
#define EID_SRS_SHUTDOWN		0x08
#define EID_SRS_ERROR			0x80


#endif /* MOD_SRS_RADSENSOR_H_ */
