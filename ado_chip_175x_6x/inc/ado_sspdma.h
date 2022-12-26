/*
 * stc_spi.h
 *
 *  Created on: 07.10.2012
 *      Author: Andi, Robert
 */

#ifndef ADO_SSPDMA_H_
#define ADO_SSPDMA_H_

#include <stdint.h>
#include <chip.h>

#define ADO_SSP_MAXJOBS 		(13)
#define ADO_SSP0_RXDMACHANNEL	7
#define ADO_SSP0_TXDMACHANNEL	6
#define ADO_SSP1_RXDMACHANNEL	5
#define ADO_SSP1_TXDMACHANNEL	4

typedef enum ado_sspid_e
{
	ADO_SSP0 = 0, ADO_SSP1 = 1
} ado_sspid_t;

typedef enum ado_sbus_id_e
{
	ADO_SBUS_SSP0 = 0, ADO_SBUS_SSP1 = 1, ADO_SBUS_SPI = 99
} ado_sbus_id_t;

typedef enum ado_sspstatus_e
{
	SSP_JOB_STATE_NEW = 0, ADO_SSP_JOBDONE, SSP_JOB_STATE_ERROR, SSP_JOB_BUFFER_OVERFLOW
} ado_sspstatus_t;

#define AdoSSP_FinishedHandler(name) void(*name)(uint32_t context, ado_sspstatus_t jobStatus, uint8_t *rxData, uint16_t rxSize)
#define AdoSSP_ActivateHandler(name) void(*name)(uint32_t context)

/**
 * @brief Initializes one of the 2 SPI buses
 *
 * @param sspId         : SSP bus to be used:
 *                          - ADO_SSP0
 *                          - ADO_SSP1
 * @param bitRate       : Clock frequency of SSP bus in Herz
 *                          - it calculates a near fit depending of system clock settings
 *                          - for sys clck 96MHz: Possible steps are: 12MHz, 16Mhz, 24Mhz, 48Mhz
 * @param clockMode     : use one of
 *                          - SSP_CLOCK_MODE3
 *                          - SSP_CLOCK_MODE0
 */
void ADO_SSP_Init(ado_sspid_t sspId, uint32_t bitRate, CHIP_SSP_CLOCK_MODE_T clockMode);

/**
 * @brief Adds a job to the queue of the SSPx bus.
 *
 * @param context       : free optional uint32 value to be used by caller to get any value/reference
 *                        into the callback.
 * @param sspId         : SSP bus to be used:
 *                          - ADO_SSP0
 *                          - ADO_SSP1
 * @param txData        : pointer to data for first (TX) part of ssp transmission
 * @param rxData        : pointer to data for second (RX) part of ssp transmission
 * @param bytes_to_send : length of first transmission part
 * @param bytes_to_read : length of second transmission part
 * @param AdoSSP_FinishedHandler(finished):
 *                        callback used when the job has finished completely.
 * @param AdoSSP_ActivateHandler(activate):
 *                        callback used before and after job execution is triggered.
 *                        Use this to control a specific CS-line if needed.
 */
void ADO_SSP_AddJob( uint32_t 	context,
					 ado_sspid_t sspId,
					 uint8_t     *txData,
					 uint8_t     *rxData,
					 uint16_t	bytes_to_send,
					 uint16_t 	bytes_to_read,
					 AdoSSP_FinishedHandler(finished),
					 AdoSSP_ActivateHandler(activate) );


#endif /* ADO_SSPDMA_H_ */
