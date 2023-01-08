/*
 * obc_i2c_int.h
 *
 *  Copied over from Pegasus Flight Software on: 2019-11-21
 *  Copied over from Climb Hwtest on: 2021-12-25
 */

#ifndef OBC_I2C_H
#define OBC_I2C_H

// Feature defines: Choose the Onboard I2C. Its one of the 3 available (on OBC: I2C1 on LPCX: also I2C1 !)
#define ONBOARD_CLOCKRATE 100    	/* I2C onboard, in kHz */
#define InitOnboardI2C(I2Cx) 		init_i2c(I2Cx, ONBOARD_CLOCKRATE)

// This was somehow configured via RTOS header files ....
//#define configMAX_LIBRARY_INTERRUPT_PRIORITY    ( 5 )
#define I2C0_INTERRUPT_PRIORITY         (9)
#define I2C1_INTERRUPT_PRIORITY         (8)
#define I2C2_INTERRUPT_PRIORITY         (9)

// Public interface to I2C routines
void init_i2c(LPC_I2C_T *I2Cx, uint32_t clockrate);

enum i2c_errors_e
{
	I2C_ERROR_NO_ERROR = 0, I2C_ERROR_RX_OVERFLOW, I2C_ERROR_BUS_ERROR, I2C_ERROR_SM_ERROR, I2C_ERROR_JOB_NOT_FINISHED
};


typedef struct
{
	uint8_t tx_count;
	uint8_t rx_count;
	uint8_t dir;
	uint8_t status;
	uint8_t tx_size;
	uint8_t rx_size;
	uint8_t* tx_data;
	uint8_t* rx_data;
	uint8_t job_done;
	uint8_t adress;
	enum i2c_errors_e error;
	LPC_I2C_T *device;
} volatile I2C_Data;

uint8_t i2c_add_job(I2C_Data* data);

#endif /* OBC_I2C_H */

