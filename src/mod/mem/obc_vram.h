/*
 * obc_vram.h
 *
 *  Created on: 21.03.2022
 *      Author: Robert
 */

#ifndef MOD_MEM_OBC_VRAM_H_
#define MOD_MEM_OBC_VRAM_H_

#include <chip.h>

typedef enum {
	VRAM_HSVS,				// Not implemented yet "hyper safe virtual storage" uses all available channels for redundancy -> TODO. use as 'page0' when implemented
	VRAM_SVS,				// "save virtual storage" Not designed yet (use 2/3 channels for redundancy!?)
	VRAM_VS,				// "virtual storage" use one defined MRAM channel. No redundancy available.
	VRAM_VS_BLOCKDEVICE,	// "virtual storage on SDCard" not implemented yet.
} vram_type_t ;

typedef struct __attribute__((packed)) {
	uint8_t 	idxId;
	vram_type_t sectionType;
	uint8_t 	chanelMask;
	uint32_t	baseAddress;
	uint32_t	sizeBytes;
} vram_sectiondef_t;

typedef struct __attribute__((packed)) {
	vram_sectiondef_t 	*sectionArray;
	uint8_t				sectionCount;
} vram_initdata_t;

void vramInit(void *initdata);
void vramMain(void);

typedef struct __attribute__((packed)) {
	bool finished;
	uint16_t bytesProcesssed;
} vram_future_t;

vram_future_t vramWriteAsync(uint8_t sectionId,uint32_t address, uint8_t *data, uint16_t size);
vram_future_t vramReadAsync(uint8_t sectionId,uint32_t address,  uint8_t *data, uint16_t size);

#endif /* MOD_MEM_OBC_VRAM_H_ */
