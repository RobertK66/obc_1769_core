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

// ADO Mudule API
void *SdcInit(ado_sspid_t sspId, void(*csHandler)(bool select));
void *SdcInitSPI(void(*csHandler)(bool select));

void SdcMain(void *cardPtr);

// SDC-Client API
void SdcCardinitialize(void *cardPtr);
void SdcReadBlockAsync(void *cardPtr, uint32_t blockNr, uint8_t *data, void (*finishedHandler)(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len));
void SdcWriteBlockAsync(void *cardPtr, uint32_t blockNr, uint8_t *data, void (*finishedHandler)(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len));


#endif /* MOD_ADO_SDCARD_H_ */
