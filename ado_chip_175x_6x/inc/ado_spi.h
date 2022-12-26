/*
 * stc_spi.h
 *
 *  Created on: 30.08.2021
 *      Author: Robert
 */

#ifndef ADO_SPI_H_
#define ADO_SPI_H_

#include <stdint.h>
#include <chip.h>

#define ADO_SPI_MAXJOBS 		(13)

//typedef enum ado_sspid_e
//{
//	ADO_SSP0 = 0, ADO_SSP1 = 1
//} ado_sspid_t;

typedef enum ado_spistatus_e
{
	SPI_JOB_STATE_NEW = 0, SPI_JOBDONE, SPI_JOB_STATE_ERROR, SPI_JOB_BUFFER_OVERFLOW
} ado_spistatus_t;

#define AdoSPI_FinishedHandler(name) void(*name)(uint32_t context, ado_spistatus_t jobStatus, uint8_t *rxData, uint16_t rxSize)
#define AdoSPI_ActivateHandler(name) void(*name)(uint32_t context)

/**
 * @brief Initializes the legacy SPI bus
 *
 * @param clockDivider  : Clock frequency of SPI bus is PIOClock / clockDevider
 *                          - for master mode this must be even and grater or equal 0x08
 */
void ADO_SPI_Init(uint8_t clockDivider, SPI_CLOCK_MODE_T mode);

/**
 * @brief Adds a job to the queue of SPI bus.
 *
 * @param context       : free optional uint32 value to be used by caller to get any value/reference
 *                        into the callback.
 * @param txData        : pointer to data for first (TX) part of ssp transmission
 * @param rxData        : pointer to data for second (RX) part of ssp transmission
 * @param bytes_to_send : length of first transmission part
 * @param bytes_to_read : length of second transmission part
 * @param AdoSPI_FinishedHandler(finished):
 *                        callback used when the job has finished completely.
 * @param AdoSPI_ActivateHandler(activate):
 *                        callback used before and after job execution is triggered.
 *                        Use this to control a specific CS-line if needed.
 */
void ADO_SPI_AddJob( uint32_t 	context,
					 uint8_t     *txData,
					 uint8_t     *rxData,
					 uint16_t	bytes_to_send,
					 uint16_t 	bytes_to_read,
					 AdoSPI_FinishedHandler(finished),
					 AdoSPI_ActivateHandler(activate) );


#endif /* ADO_SPI_H_ */
