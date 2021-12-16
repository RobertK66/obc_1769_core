/*
 * ado_sdcard.h
 *
 *  Created on: 16.05.2020
 *      Author: Robert
 */

#ifndef MOD_ADO_SDCARD_H_
#define MOD_ADO_SDCARD_H_

#include <ado_sspdma.h>

typedef enum sdc_res_e
{
    SDC_RES_SUCCESS = 0,
    SDC_RES_ERROR,
    SDC_RES_CRCERROR
} sdc_res_t;

typedef enum ado_sbus_id_e
{
	ADO_SBUS_SSP0 = 0, ADO_SBUS_SSP1 = 1, ADO_SBUS_SPI = 99
} ado_sbus_id_t;


typedef struct {
	ado_sbus_id_t busnr;
	void(*csHandler)(bool select);
} sdcard_init_t;

typedef struct {
	uint8_t		 entryCount;
	const sdcard_init_t* chipinits;
} sdcard_init_array_t;



// ADO Mudule API
void *SdcInit(ado_sspid_t sspId, void(*csHandler)(bool select));
void *SdcInitSPI(void(*csHandler)(bool select));
void SdcInitAll(void* cards);
void _SdcInitAll(sdcard_init_array_t* cards);

void SdcMain();

// SDC-Client API
void SdcCardinitialize(uint8_t cardIdx);
void SdcReadBlockAsync(uint8_t cardIdx, uint32_t blockNr, uint8_t *data, void (*finishedHandler)(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len));
void SdcWriteBlockAsync(uint8_t cardIdx, uint32_t blockNr, uint8_t *data, void (*finishedHandler)(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len));


#endif /* MOD_ADO_SDCARD_H_ */
