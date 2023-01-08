/*
 * ado_sdcard.h
 *
 *  Created on: 16.05.2020
 *      Author: Robert
 */

#ifndef MOD_ADO_SDCARD_H_
#define MOD_ADO_SDCARD_H_

//#include <chip.h>
#include <ado_sspdma.h>

#define MODULE_ID_SDCARD				0x81
#define EID_SDCARD_STATUS				1
#define EID_SDCARD_UNIMPLEMENTED_TYPE 	2

typedef enum sdc_res_e
{
    SDC_RES_SUCCESS = 0,
    SDC_RES_ERROR,
    SDC_RES_CRCERROR
} sdc_res_t;

typedef struct {
	ado_sbus_id_t 			busnr;
	const PINMUX_GRP_T2*  	csPin;
} sdcard_init_t;

typedef struct {
	uint8_t		 entryCount;
	const sdcard_init_t* chipinits;
} sdcard_init_array_t;

typedef struct {
	uint8_t  token;
	uint8_t  data[512];
	uint8_t  crc[2];
} sdcard_block512;

// ADO Mudule API
void _SdcInitAll(sdcard_init_array_t* cards);
void SdcInitAll(void* cards);
void SdcMain();

// SDC-Client API
void SdcCardinitialize(uint8_t cardIdx);
bool SdcIsCardinitialized(uint8_t cardIdx);
void SdcReadBlockAsync(uint8_t cardIdx, uint32_t blockNr, uint8_t *data, void (*finishedHandler)(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len));
//void SdcWriteBlockAsync(uint8_t cardIdx, uint32_t blockNr, uint8_t *data, void (*finishedHandler)(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len));
void SdcWriteBlockAsync(uint8_t cardIdx, uint32_t blockNr, sdcard_block512 *data, void (*finishedHandler)(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len));

#endif /* MOD_ADO_SDCARD_H_ */
