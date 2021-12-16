/*
 * mram.h
 *
 *  Created on: 27.12.2019
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
	ado_sspid_t		 busnr;
	void(*csHandler)(bool select);
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
	MRAM_RES_INVALID_CHIPIDX
} mram_res_t;


// Module API
void _MramInitAll(mram_chipinit_array_t *chips);
void MramInitAll(void *chips);
void MramMain();

void ReadMramAsync(uint8_t chipIdx, uint32_t adr,  uint8_t *rx_data,  uint32_t len, void (*finishedHandler)(mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len));
void WriteMramAsync(uint8_t chipIdx, uint32_t adr, uint8_t *data, uint32_t len,  void (*finishedHandler)(mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len));

// Depricated
void MramInit(uint8_t chipIdx, ado_sspid_t busNr, void(*csHandler)(bool select));	  // Module Init called once for each connected chip prior mainloop
void MramInitSPI(uint8_t chipIdx, void(*csHandler)(bool select));             // Module Init called once for each connected chip prior mainloop



#endif /* MOD_MEM_MRAM_H_ */
