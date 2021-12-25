/*
 * obc_i2c_int.h
 *
 *  Copied over from Pegasus Flight Software on: 2019-11-21
 *  Copied over from Climb Hwtest on: 2021-12-25
 *
 */

#ifndef OBC_I2C_INT_H
#define OBC_I2C_INT_H

#include <chip.h>
#include "obc_i2c.h"

// Module (private) definitions
typedef struct i2c_status_s
{
	/* OBC status bits block 2 - 32 Bits */
	unsigned int i2c_initialized 				:1;  /* Bit 0 */
	unsigned int i2c_interrupt_handler_error 	:1;  /* Bit 1 */
	unsigned int unused 		  				:28; /* Bit 2..31 */

	/* OBC error and overflow counters */
	/* --- Block 1 --- */
	uint8_t i2c_error_counter;
}
volatile i2c_status_t;

// module prototypes
void I2C_send(LPC_I2C_T *device, uint8_t busNr);
void I2C_Handler(LPC_I2C_T *I2Cx);
uint8_t I2C_getNum(LPC_I2C_T *I2Cx);

#endif
