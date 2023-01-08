/*
 * mram.h
 *
 *  Created on: 27.12.2019
 *
 */

#ifndef MOD_ADO_MRAM_H_
#define MOD_ADO_MRAM_H_

#include <ado_sspdma.h>

#define MRAM_MAX_READ_SIZE	1024
#define MRAM_MAX_WRITE_SIZE	1024

#define MRAM_CHIP_CNT       6

#define MODULE_ID_MRAM		0x80
#define EID_MRAM_JOBERROR	1

typedef struct {
	ado_sbus_id_t	 		busnr;
	const PINMUX_GRP_T2*  	csPin;
} mram_chipinit_t;

typedef struct {
	uint8_t		 entryCount;
	const mram_chipinit_t* chipinits;
} mram_chipinit_array_t;

typedef enum mram_res_e
{
	MRAM_RES_SUCCESS = 0,
	MRAM_RES_BUSY,
	MRAM_RES_DATA_PTR_INVALID,
	MRAM_RES_RX_LEN_OVERFLOW,
	MRAM_RES_INVALID_ADR,
	MRAM_RES_JOB_ADD_ERROR,
	MRAM_RES_JOB_ADDED_OK,
	MRAM_RES_INVALID_CHIPIDX,
	MRAM_RES_NOTINITIALIZED
} mram_res_t;

// Module API
void _MramInitAll(mram_chipinit_array_t *chips);
void MramInitAll(void *chips);
void MramMain();

void MramReadAsync(uint8_t chipIdx, uint32_t adr,  uint8_t *rx_data,  uint32_t len, void (*finishedHandler)(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len));
void MramWriteAsync(uint8_t chipIdx, uint32_t adr, uint8_t *data, uint32_t len,  void (*finishedHandler)(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len));
mram_res_t MramIsChipItialized(uint8_t chipIdx);

#endif /* MOD_MEM_MRAM_H_ */
